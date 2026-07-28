package frc.robot.recovery;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Comparator;
import java.util.stream.Stream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Throwaway harness for the Autonomous Disturbance Simulation experiment
 * (docs/Autonomous_Disturbance_Simulation_Report.md) -- NOT part of the permanent regression
 * suite (see {@code frc.robot.auto.AutoRegressionTestBase} for that). Deliberately runs a single
 * PathPlanner path in isolation via {@code AutoBuilder.followPath}, not a whole registered
 * {@code .auto}, to avoid the already-documented ~7m baseline tracking-error bug on the compound
 * "LT Neutral" auto (see docs/Autonomous_Recovery_Audit.md and this repo's PID-retune-milestone
 * history) swamping the disturbance signal this experiment is trying to isolate.
 *
 * <p>At a fixed elapsed time, teleports ONLY the maple-sim physics body -- never the CTRE pose
 * estimator -- via {@code CommandSwerveDrivetrain.getMapleSimDrive().setSimulationWorldPose()}.
 * This is the maple-sim primitive {@code CommandSwerveDrivetrain.resetPose()} itself reuses, but
 * called here without also calling {@code super.resetPose()}, so the wheel odometry and pose
 * estimator never "see" the jump -- exactly like a real collision, where another robot's momentum
 * changes the true pose but the encoders keep integrating from wherever they already were.
 *
 * <p>Entire {@code frc.robot.recovery} package is disposable: delete it once
 * docs/Autonomous_Disturbance_Simulation_Report.md is finished with it. No production file is
 * touched by anything in this package.
 */
abstract class PathDisturbanceSimTestBase {
    static final String kPathName = "Left Trench Neutral";
    static final double kInjectionElapsedSeconds = 1.5;
    static final double kMaxRuntimeSeconds = 10.0;
    static final double kStepSeconds = 0.02;

    private Robot robot;
    private Thread robotThread;
    private boolean toreDown;
    private boolean timingResumed;

    /** Robot-relative lateral (perpendicular to heading) displacement to inject, in meters. 0 = control. */
    protected abstract double displacementMeters();

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
        DriverStationSim.resetData();
        SimHooks.pauseTiming();

        robot = new Robot();
        robotThread = new Thread(robot::startCompetition, getClass().getSimpleName() + "-competition");
        robotThread.setDaemon(true);
        robotThread.start();
        SimHooks.waitForProgramStart();
        toreDown = false;
        timingResumed = false;
    }

    @AfterEach
    void tearDown() throws InterruptedException {
        if (!toreDown) {
            finishCompetitionAndClose();
        }
        if (!timingResumed) {
            SimHooks.resumeTiming();
        }
    }

    private void finishCompetitionAndClose() throws InterruptedException {
        robot.endCompetition();
        robotThread.join(1000);
        assertTrue(!robotThread.isAlive(),
                "startCompetition() loop should have exited within 1s of endCompetition()");
        robot.close();
        toreDown = true;
    }

    @Test
    @Timeout(30)
    void disturbanceRun() throws Exception {
        assertTrue(robotThread.isAlive(),
                "startCompetition() loop should still be running after waitForProgramStart()");

        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        PathPlannerPath path = PathPlannerPath.fromPathFile(kPathName);
        Pose2d startPose = path.getStartingHolonomicPose()
                .orElseThrow(() -> new IllegalStateException(kPathName + " has no starting holonomic pose"));
        // Mirrors what a real .auto file's "resetOdom": true does -- seeds both the CTRE pose
        // estimator and the maple-sim physics body to the path's nominal start.
        RobotContainer.drivetrain.resetPose(startPose);

        Command followCommand = AutoBuilder.followPath(path);
        CommandScheduler.getInstance().schedule(followCommand);
        SimHooks.stepTiming(kStepSeconds);
        SimHooks.resumeTiming();
        timingResumed = true;

        boolean injected = false;
        long stepMillis = Math.round(kStepSeconds * 1000.0);
        long startNanos = System.nanoTime();
        double elapsedSeconds = 0.0;
        boolean completed = false;

        while (elapsedSeconds < kMaxRuntimeSeconds) {
            Thread.sleep(stepMillis);
            elapsedSeconds = (System.nanoTime() - startNanos) / 1e9;

            if (!injected && elapsedSeconds >= kInjectionElapsedSeconds) {
                Pose2d groundTruth = RobotContainer.drivetrain.getSimulatedGroundTruthPose();
                Translation2d lateralShove =
                        new Translation2d(0, displacementMeters()).rotateBy(groundTruth.getRotation());
                Pose2d displaced = new Pose2d(
                        groundTruth.getTranslation().plus(lateralShove), groundTruth.getRotation());
                RobotContainer.drivetrain.getMapleSimDrive().setSimulationWorldPose(displaced);
                injected = true;
                System.out.println("[disturbance] injected " + displacementMeters() + "m at t="
                        + elapsedSeconds + "s : " + groundTruth + " -> " + displaced);
            }

            if (!CommandScheduler.getInstance().isScheduled(followCommand)) {
                completed = true;
                break;
            }
        }

        System.out.println("[disturbance] run ended: completed=" + completed + " elapsed=" + elapsedSeconds
                + "s displacement=" + displacementMeters() + "m");

        finishCompetitionAndClose();

        Path wpilog = findNewestWpilog();
        assertTrue(wpilog != null, "No .wpilog found in logs/ after the run.");
        System.out.println("[disturbance] wpilog: " + wpilog.toAbsolutePath());
    }

    private static Path findNewestWpilog() throws IOException {
        Path logsDir = Paths.get("logs");
        if (!Files.isDirectory(logsDir)) {
            return null;
        }
        try (Stream<Path> files = Files.list(logsDir)) {
            return files.filter(p -> p.toString().endsWith(".wpilog"))
                    .max(Comparator.comparingLong(p -> p.toFile().lastModified()))
                    .orElse(null);
        }
    }
}
