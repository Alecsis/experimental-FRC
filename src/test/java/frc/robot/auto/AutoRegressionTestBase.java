package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.BuildConstants;
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
 * Shared harness for the PathPlanner auto regression suite. One concrete subclass per auto (see
 * docs/superpowers/specs/2026-07-18-auto-regression-suite-design.md for why one class per auto,
 * not a single {@code @ParameterizedTest} -- {@code forkEvery=1} plus the Vision/Intake/Shooter/
 * Superstructure singletons require it, same reasoning as {@code RobotLifecycleTest}).
 *
 * <p>Regression, not accuracy: each run is compared against a checked-in golden snapshot of
 * completed/runtime/max-tracking-error, never against a hand-authored "ideal" pose. Deliberately
 * stays inside tier 3 of docs/claudex/design/sim-timing-determinism.md (coarse pass/fail that
 * tolerates known sim-timing jitter) -- this suite must never be used for PID/gain comparisons.
 */
abstract class AutoRegressionTestBase {
    private Robot robot;
    private Thread robotThread;
    private boolean toreDown;
    private boolean timingResumed;

    /** The exact PathPlanner auto name, e.g. "LT Neutral". */
    protected abstract String autoName();

    /** File-safe golden resource name, e.g. "LT_Neutral.json". */
    protected abstract String goldenFileName();

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
        assertFalse(robotThread.isAlive(),
                "startCompetition() loop should have exited within 1s of endCompetition()");
        robot.close();
        toreDown = true;
    }

    @Test
    @Timeout(45)
    void regressionCheck() throws IOException, InterruptedException {
        assertTrue(robotThread.isAlive(),
                "startCompetition() loop should still be running after waitForProgramStart()");

        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        // Built and scheduled directly (not via the SendableChooser over NetworkTables), matching
        // Milestone 1's own validated approach (docs/claudex/history.md, eleventh session).
        // RobotContainer's constructor -- run synchronously inside `new Robot()` in @BeforeEach,
        // which has already returned by this point -- registers every NamedCommand and
        // configureAutoBuilder()'s callbacks before this line ever runs, so AutoBuilder.buildAuto
        // sees a fully-configured registry.
        //
        // Robot.autonomousInit() will *also* fire on the next periodic tick and separately
        // schedule RobotContainer.getAutonomousCommand() (= autoChooser.getSelected()). Verified
        // against PathPlannerLib's own source (AutoBuilder.buildAutoChooserWithOptionsModifier,
        // 2026.1.2): an empty default name -- exactly what RobotContainer's no-arg
        // buildAutoChooser() passes -- makes "None"/Commands.none() the chooser's default
        // selection. Commands.none() has zero subsystem requirements and finishes on the very
        // first scheduler tick, so it cannot interrupt or conflict with the command scheduled
        // below.
        //
        // schedule() is called here while SimHooks is still paused (from @BeforeEach), i.e. the
        // robot thread is blocked -- calling CommandScheduler.getInstance().schedule() from the
        // test thread while the robot thread concurrently calls CommandScheduler.run() in real
        // time is a genuine data race on a non-thread-safe Set (confirmed empirically: it produced
        // a run that "completed" after ~0.02s of measured time yet ended 5m from its start pose --
        // the robot thread kept driving the command in the background while the race corrupted
        // this thread's isScheduled() view of it). The stepTiming() call right after schedule()
        // deterministically runs exactly one robotPeriodic tick (still paused, so the robot thread
        // blocks again immediately after) to actually initialize() the command, then resumeTiming()
        // switches to real time for the rest of the run -- from that point on this thread only
        // reads scheduler/pose state, never mutates it, matching the read-only cross-thread access
        // RobotLifecycleTest/SuperstructureEjectingTest already rely on.
        Command autoCommand = AutoBuilder.buildAuto(autoName());
        CommandScheduler.getInstance().schedule(autoCommand);
        SimHooks.stepTiming(AutoRegressionTolerances.kStepSeconds);
        SimHooks.resumeTiming();
        timingResumed = true;

        // This loop only decides WHEN to stop (completed vs. real-time cap) and samples a
        // diagnostic pose -- it does not make any stall/error judgement itself. Both live here
        // because they're immune to timing aliasing: isScheduled() is a persistent state (not a
        // single-tick pulse), and the real-time wall-clock cap is exactly what it measures.
        // Stall detection and max tracking error are both computed post-hoc from the wpilog
        // instead (see WpilogStallAnalyzer's class doc for why: a live NetworkTables poll of the
        // single-tick /RealOutputs/Trajectory/SetpointFresh pulse was tried first and empirically
        // missed it due to poll/tick aliasing, even though the wpilog itself captured it fine).
        Pose2d startPose = RobotContainer.drivetrain.getState().Pose;
        Pose2d lastPose = startPose;
        boolean completed = false;

        long stepMillis = Math.round(AutoRegressionTolerances.kStepSeconds * 1000.0);
        long startNanos = System.nanoTime();
        double elapsedSeconds = 0.0;
        while (elapsedSeconds < AutoRegressionTolerances.kMaxRuntimeSeconds) {
            Thread.sleep(stepMillis);
            elapsedSeconds = (System.nanoTime() - startNanos) / 1e9;
            lastPose = RobotContainer.drivetrain.getState().Pose;

            if (!CommandScheduler.getInstance().isScheduled(autoCommand)) {
                completed = true;
                break;
            }
        }

        // Flush before reading: WPILOGWriter only finalizes on close(), same ordering
        // RobotLifecycleTest's teardown depends on for its own log-based assertions.
        finishCompetitionAndClose();

        Path wpilog = findNewestWpilog();
        assertTrue(wpilog != null,
                "No .wpilog found in logs/ after the run -- SIM mode should have written one.");

        WpilogStallAnalyzer.Result stallResult = WpilogStallAnalyzer.checkForStall(wpilog,
                AutoRegressionTolerances.kStallWindowSeconds,
                AutoRegressionTolerances.kStallTranslationThresholdMeters,
                AutoRegressionTolerances.kStalePathSetpointGraceSeconds);
        assertFalse(stallResult.stalled(),
                () -> autoName() + " " + stallResult.diagnostic());

        WpilogTrajectoryErrorReader.MaxErrors errors = WpilogTrajectoryErrorReader.readMaxErrors(wpilog);

        AutoRegressionGolden actual = new AutoRegressionGolden();
        actual.autoName = autoName();
        actual.recordedAtGitSha = BuildConstants.GIT_SHA;
        actual.completed = completed;
        actual.runtimeSeconds = elapsedSeconds;
        actual.maxLateralErrorMeters = errors.maxLateralErrorMeters();
        actual.maxLongitudinalErrorMeters = errors.maxLongitudinalErrorMeters();
        actual.diagnosticStartPose = toDiagnosticPose(startPose);
        actual.diagnosticFinalPose = toDiagnosticPose(lastPose);

        boolean updateRequested = "true".equals(System.getProperty("updateAutoGolden"));
        boolean updateConfirmed = "true".equals(System.getProperty("confirmGoldenUpdate"));
        if (updateRequested != updateConfirmed) {
            fail("Golden update requires BOTH -DupdateAutoGolden=true and -DconfirmGoldenUpdate=true; "
                    + "only one was set (updateAutoGolden=" + updateRequested
                    + ", confirmGoldenUpdate=" + updateConfirmed + "). Refusing to guess which you meant.");
        }

        Path goldenPath = goldenPath();
        ObjectMapper mapper = new ObjectMapper();

        if (updateRequested) {
            mapper.writerWithDefaultPrettyPrinter().writeValue(goldenPath.toFile(), actual);
            System.out.println("Wrote golden for '" + autoName() + "' to " + goldenPath);
            return;
        }

        if (!Files.isRegularFile(goldenPath)) {
            fail("No golden file at " + goldenPath + " for auto '" + autoName() + "'. Run with "
                    + "-DupdateAutoGolden=true -DconfirmGoldenUpdate=true to establish a baseline.");
        }

        AutoRegressionGolden golden = mapper.readValue(goldenPath.toFile(), AutoRegressionGolden.class);

        assertEquals(golden.completed, actual.completed, () -> autoName() + ": completed regressed "
                + "(golden=" + golden.completed + ", actual=" + actual.completed + ")");
        assertTrue(
                Math.abs(actual.runtimeSeconds - golden.runtimeSeconds)
                        <= AutoRegressionTolerances.kRuntimeToleranceSeconds,
                () -> autoName() + ": runtimeSeconds " + actual.runtimeSeconds + " outside golden "
                        + golden.runtimeSeconds + " +/- " + AutoRegressionTolerances.kRuntimeToleranceSeconds + "s");
        assertTrue(
                actual.maxLateralErrorMeters
                        <= golden.maxLateralErrorMeters + AutoRegressionTolerances.kLateralErrorHeadroomMeters,
                () -> autoName() + ": maxLateralErrorMeters " + actual.maxLateralErrorMeters
                        + " exceeds golden " + golden.maxLateralErrorMeters + " + "
                        + AutoRegressionTolerances.kLateralErrorHeadroomMeters + "m headroom");
        assertTrue(
                actual.maxLongitudinalErrorMeters
                        <= golden.maxLongitudinalErrorMeters + AutoRegressionTolerances.kLongitudinalErrorHeadroomMeters,
                () -> autoName() + ": maxLongitudinalErrorMeters " + actual.maxLongitudinalErrorMeters
                        + " exceeds golden " + golden.maxLongitudinalErrorMeters + " + "
                        + AutoRegressionTolerances.kLongitudinalErrorHeadroomMeters + "m headroom");
    }

    private Path goldenPath() {
        return Paths.get("src", "test", "resources", "autoRegression", goldenFileName());
    }

    private static AutoRegressionGolden.DiagnosticPose toDiagnosticPose(Pose2d pose) {
        return new AutoRegressionGolden.DiagnosticPose(
                pose.getX(), pose.getY(), pose.getRotation().getRadians());
    }

    /**
     * Newest-by-mtime, matching SKILLS/run_headless_sim.py's own convention. Safe without a
     * before/after diff because forkEvery=1 plus Gradle's default maxParallelForks=1 (confirmed
     * absent from build.gradle's test{} block) mean exactly one Robot() -- and therefore exactly
     * one WPILOGWriter -- is ever active per JVM, and this suite schedules one @Test per class.
     */
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
