package frc.robot.recovery;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Comparator;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
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
 * <p><b>World ownership.</b> That teleport mutates the dyn4j world, so it must run on the thread
 * that owns it -- the robot competition thread, the only caller of
 * {@code SimulatedArena.simulationPeriodic()} since the MapleSim notifier was removed. It used to
 * be issued directly from the JUnit thread inside the polling loop below, after
 * {@code resumeTiming()}, i.e. concurrently with the robot loop's physics step: the last remaining
 * cross-thread world mutation in the codebase.
 *
 * <p>The split now is: the JUnit thread <em>requests</em> a disturbance (a plain
 * {@link AtomicBoolean} write -- not world state), and a one-shot command scheduled while the
 * simulation is still PAUSED -- the same window {@code followCommand} is scheduled in, and the only
 * window in which touching the non-thread-safe {@code CommandScheduler} from the test thread is
 * safe -- picks that request up on the next scheduler tick and performs the mutation from
 * {@code robotPeriodic()}. The JUnit thread's trigger condition, and therefore the disturbance's
 * timing, is unchanged; only the executing thread moved. {@link #injectDisturbance()} records the
 * thread it actually ran on as its first statement, and {@link #disturbanceRun()} asserts that
 * thread is identically the competition thread -- so a regression back to the JUnit thread fails
 * this test rather than silently reintroducing the race.
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

    /**
     * JUnit thread -> robot thread handoff. Set by the polling loop when its existing wall-clock
     * trigger fires; observed by the pre-scheduled disturbance command on the next scheduler tick.
     * Carries no world state, so writing it from the test thread is not a world mutation.
     */
    private final AtomicBoolean disturbanceRequested = new AtomicBoolean();

    /**
     * The thread the FIRST {@link #injectDisturbance()} call ran on, captured inside that method
     * rather than inferred. Null means the disturbance never fired, which is itself a failure.
     *
     * <p>Latched with {@code compareAndSet}, never {@code set}: a regression that mutates from the
     * JUnit thread <em>and</em> leaves the robot-thread command in place would otherwise have its
     * violating first write overwritten by the compliant second one, and the ownership assertion
     * would pass on a broken harness. Measured -- that exact negative control passed against a
     * {@code set()} implementation of this field.
     */
    private final AtomicReference<Thread> mutationThread = new AtomicReference<>();

    /** How many times the teleport ran. Anything but 1 means the harness mutates twice. */
    private final AtomicInteger mutationCount = new AtomicInteger();

    /** Diagnostic record of the teleport, published from the robot thread for the JUnit thread. */
    private volatile Pose2d injectedFromPose;
    private volatile Pose2d injectedToPose;
    private volatile long injectedAtNanos;

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

        // Scheduled HERE, while still paused, for the same reason followCommand is: scheduling from
        // the test thread while the robot thread is concurrently running CommandScheduler.run() is a
        // data race on a non-thread-safe Set (see AutoRegressionTestBase's own comment). It declares
        // no requirements, so it cannot interrupt followCommand or the chooser's Commands.none().
        // Its whole job is to move the setSimulationWorldPose() call onto the robot loop.
        Command disturbanceCommand = Commands.waitUntil(disturbanceRequested::get)
                .andThen(Commands.runOnce(this::injectDisturbance))
                .withName("DisturbanceInjection");
        CommandScheduler.getInstance().schedule(disturbanceCommand);

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

            // Same trigger condition and same wall-clock instant as before. What changed is what
            // happens next: this thread only raises the request flag, and the robot loop performs
            // the world mutation on its next tick.
            if (!injected && elapsedSeconds >= kInjectionElapsedSeconds) {
                disturbanceRequested.set(true);
                injected = true;
                System.out.println("[disturbance] requested " + displacementMeters() + "m at t="
                        + elapsedSeconds + "s from " + Thread.currentThread().getName());
            }

            if (!CommandScheduler.getInstance().isScheduled(followCommand)) {
                completed = true;
                break;
            }
        }

        // The request is asynchronous by construction, so wait for the robot loop to actually
        // consume it before tearing that loop down. Bounded: a disturbance that never lands is a
        // failure to report, not a hang. In practice this returns on the first poll -- the run
        // continues for seconds after the 1.5 s trigger.
        long deadlineNanos = System.nanoTime() + 1_000_000_000L;
        while (mutationThread.get() == null && System.nanoTime() < deadlineNanos) {
            Thread.sleep(5);
        }

        System.out.println("[disturbance] run ended: completed=" + completed + " elapsed=" + elapsedSeconds
                + "s displacement=" + displacementMeters() + "m");

        finishCompetitionAndClose();

        // --- world-ownership proof -------------------------------------------------------------
        // Direct, not inferred: injectDisturbance() records Thread.currentThread() as its first
        // statement, in the same method body as the setSimulationWorldPose() call, so the two
        // cannot drift apart without a deliberate edit. Identity comparison, not name comparison.
        Thread mutator = mutationThread.get();
        assertTrue(mutator != null,
                "the disturbance never executed -- setSimulationWorldPose() was never reached, so this"
                        + " run proves nothing about world ownership (requested=" + injected
                        + ", completed=" + completed + ", elapsed=" + elapsedSeconds + "s)");
        assertSame(robotThread, mutator,
                "MapleSim/dyn4j world mutation must run on the robot competition thread, but ran on '"
                        + mutator.getName() + "' (expected '" + robotThread.getName() + "')");
        assertNotSame(Thread.currentThread(), mutator,
                "world mutation ran on the JUnit test thread '" + mutator.getName()
                        + "' -- the cross-thread ownership violation has regressed");
        assertTrue(!CommandScheduler.getInstance().isScheduled(disturbanceCommand),
                "the one-shot disturbance command should have completed, but is still scheduled");
        assertEquals(1, mutationCount.get(),
                "the disturbance teleport must happen exactly once; a second call means the harness"
                        + " mutates the world from more than one place");

        System.out.println("[disturbance] injected " + displacementMeters() + "m on thread '"
                + mutator.getName() + "' at t=" + ((injectedAtNanos - startNanos) / 1e9) + "s : "
                + injectedFromPose + " -> " + injectedToPose);

        Path wpilog = findNewestWpilog();
        assertTrue(wpilog != null, "No .wpilog found in logs/ after the run.");
        System.out.println("[disturbance] wpilog: " + wpilog.toAbsolutePath());
    }

    /**
     * Performs the disturbance teleport. Runs from {@code CommandScheduler.run()} inside
     * {@code robotPeriodic()} -- i.e. on the robot competition thread, the owner of the dyn4j world
     * -- so it cannot race the physics step that {@code Robot.simulationPeriodic()} runs later in
     * the same loop.
     *
     * <p>The thread capture is deliberately the FIRST statement, in the same method as the mutation
     * it is vouching for. Anything that moves the {@code setSimulationWorldPose()} call back onto
     * another thread has to move this line with it, or {@link #disturbanceRun()}'s
     * {@code assertSame} fails.
     *
     * <p>Reading the ground-truth pose moved here too, as a consequence rather than as a goal: it
     * used to be an unsynchronized cross-thread read of the same dyn4j body, taken from the JUnit
     * thread while the robot loop was stepping physics.
     */
    private void injectDisturbance() {
        mutationThread.compareAndSet(null, Thread.currentThread());
        mutationCount.incrementAndGet();

        Pose2d groundTruth = RobotContainer.drivetrain.getSimulatedGroundTruthPose();
        Translation2d lateralShove =
                new Translation2d(0, displacementMeters()).rotateBy(groundTruth.getRotation());
        Pose2d displaced = new Pose2d(
                groundTruth.getTranslation().plus(lateralShove), groundTruth.getRotation());
        RobotContainer.drivetrain.getMapleSimDrive().setSimulationWorldPose(displaced);

        injectedFromPose = groundTruth;
        injectedToPose = displaced;
        injectedAtNanos = System.nanoTime();
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
