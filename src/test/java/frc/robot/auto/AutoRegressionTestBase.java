package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
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
import java.util.concurrent.atomic.AtomicReference;
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
        // Robot-thread-written, test-thread-read: the exact simulated timestamp at which the auto
        // command ended. This is what makes `completed` deterministic -- see the boundary rule
        // documented on the sampling loop below. finallyDo() runs synchronously on the robot thread
        // in the same scheduler tick the command ends, so this records the real end instant rather
        // than whenever this thread next happens to look.
        AtomicReference<Double> autoEndFpgaSeconds = new AtomicReference<>(null);
        Command instrumentedAuto = autoCommand.finallyDo(
                () -> autoEndFpgaSeconds.compareAndSet(null, Timer.getFPGATimestamp()));
        CommandScheduler.getInstance().schedule(instrumentedAuto);
        SimHooks.stepTiming(AutoRegressionTolerances.kStepSeconds);
        // Origin for the cap, captured on the SAME clock the command's own timing runs on, at the
        // tick that just initialized it. Deliberately not System.nanoTime(): see below.
        double autoStartFpgaSeconds = Timer.getFPGATimestamp();
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

        // ---- Deterministic boundary rule ----
        //
        // completed == "the auto command ended at or before kMaxRuntimeSeconds of SIMULATED
        // autonomous time, measured from the tick that initialized it".
        //
        // Both halves of that sentence are now read from the same clock, and the verdict never
        // depends on when this thread happens to sample. Two independent defects made it
        // nondeterministic before, and "LT Neutral" sat exactly on top of both: its command ends
        // 15.02-15.04 s into autonomous against a 15.0 s cap, and it flipped in 2 of 6 measured
        // runs.
        //
        //   1. TWO CLOCKS WITH DIFFERENT ORIGINS. The command's own timing (path following, and
        //      the trailing Commands.waitSeconds deadline) runs on the WPILib/HAL simulated clock,
        //      which started advancing at the stepTiming() tick above that initialized it. The cap
        //      used System.nanoTime() captured AFTER resumeTiming(), i.e. strictly later. Elapsed
        //      therefore under-counted the command's own autonomous time by the cost of
        //      stepTiming() plus resume -- tens of milliseconds, exactly the scale of the flip.
        //      Fixed by measuring elapsed with Timer.getFPGATimestamp() from autoStartFpgaSeconds.
        //
        //   2. THE CAP WAS NOT APPLIED TO THE VERDICT. The old loop tested the cap in its while
        //      condition, BEFORE sleeping; after the sleep elapsed could already be past the cap,
        //      yet a command found finished at that sample was still credited completed = true.
        //      So a command that ended after the cap could be recorded as having completed within
        //      it. Fixed by classifying from the recorded end timestamp, not from loop exit.
        //
        // Rejected alternative: converting this harness to paused SimHooks.stepTiming() stepping
        // (the SimRobotLoop pattern the operator/intake suites use) would make the whole run a pure
        // function of the timestep. It is rejected here because it also changes the EXECUTION
        // model -- CTRE's 250 Hz sim odometry notifier and MapleSim would be advanced in 20 ms
        // batches instead of real time -- which would move tracking error, final pose and runtime
        // for every route and force all 15 goldens to be re-recorded. This fix keeps execution
        // real-time and changes only how the boundary is measured.
        //
        // Residual: this loop still only decides WHEN TO STOP watching, so a route that ends
        // within a poll interval below the cap is stopped on slightly late, which is why the
        // settle below exists rather than deciding at loop exit.
        long stepMillis = Math.round(AutoRegressionTolerances.kStepSeconds * 1000.0);
        double elapsedSeconds = 0.0;
        while (autoEndFpgaSeconds.get() == null
                && elapsedSeconds < AutoRegressionTolerances.kMaxRuntimeSeconds) {
            Thread.sleep(stepMillis);
            elapsedSeconds = Timer.getFPGATimestamp() - autoStartFpgaSeconds;
            lastPose = RobotContainer.drivetrain.getState().Pose;
        }

        // Close the last race: a command that ended just before the cap may not have had its
        // finallyDo observed by the sample that exited the loop. A few extra poll intervals let
        // that write land. It cannot manufacture a false positive -- a command that ends after the
        // cap records an end timestamp that is still after the cap, and is classified accordingly.
        for (int i = 0; i < 3 && autoEndFpgaSeconds.get() == null; i++) {
            Thread.sleep(stepMillis);
            elapsedSeconds = Timer.getFPGATimestamp() - autoStartFpgaSeconds;
            lastPose = RobotContainer.drivetrain.getState().Pose;
        }

        Double autoEnd = autoEndFpgaSeconds.get();
        boolean completed = completedWithinCap(autoEnd, autoStartFpgaSeconds,
                AutoRegressionTolerances.kMaxRuntimeSeconds);
        elapsedSeconds = autoEnd == null ? elapsedSeconds : autoEnd - autoStartFpgaSeconds;

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

        assertMatchesGolden(autoName(), goldenFileName(), golden, actual);
    }

    /**
     * The deterministic boundary rule, extracted so it is provable by a plain unit test
     * ({@code AutoRegressionBoundaryTest}) without booting a Robot.
     *
     * <p><b>Rule.</b> {@code completed} is true if and only if the auto command actually ended, and
     * the simulated time between the tick that initialized it and the tick that ended it is at most
     * {@code capSeconds}. Both timestamps come from the WPILib clock the command's own timing runs
     * on, and the end timestamp is recorded by the robot thread inside the command's own
     * {@code finallyDo}. Nothing here depends on host CPU scheduling, {@code System.nanoTime()}, or
     * on when the polling loop happened to look: a command that ends after the cap is
     * {@code false} even if the harness noticed it late, and a command that ends before the cap is
     * {@code true} even if the harness noticed it late.
     *
     * @param autoEndFpgaSeconds simulated timestamp the command ended, or null if it never did
     * @param autoStartFpgaSeconds simulated timestamp of the tick that initialized it
     */
    static boolean completedWithinCap(Double autoEndFpgaSeconds, double autoStartFpgaSeconds,
            double capSeconds) {
        return autoEndFpgaSeconds != null
                && autoEndFpgaSeconds - autoStartFpgaSeconds <= capSeconds;
    }

    /**
     * Grades one run against its golden. Extracted from {@link #regressionCheck()} so the ordering
     * below is provable by a plain unit test ({@code AutoRegressionGoldenIdentityTest}) instead of
     * by reading the method top-to-bottom -- booting a Robot is not required to check it.
     *
     * <p><b>Identity is checked first, deliberately.</b> Every later comparison presumes the golden
     * describes the route that just ran. When it does not -- as happens after an auto is renamed in
     * PathPlanner and the test is re-pointed at the new name while the golden file keeps the old
     * snapshot -- those comparisons are still arithmetically valid and can still pass, because the
     * tolerances (+/-3s runtime, +0.75m lateral, +1.0m longitudinal) are wide enough to swallow a
     * different route. That produces a GREEN test standing on a snapshot of something else, which
     * is a worse failure mode than a red one. Measured 2026-08-20: all four
     * "* Recollect *" tests passed that way against goldens recorded at a start pose 0.200 m and
     * 180.0 deg away from the route they were grading.
     *
     * <p>{@code diagnosticStartPose} would have caught it, but it is informational and never
     * compared (see {@link AutoRegressionGolden}); {@code autoName} is the field that carries
     * identity, so it is the one asserted.
     */
    static void assertMatchesGolden(String autoName, String goldenFileName,
            AutoRegressionGolden golden, AutoRegressionGolden actual) {
        assertEquals(autoName, golden.autoName,
                () -> "golden " + goldenFileName + " was recorded for auto '" + golden.autoName
                        + "', not '" + autoName + "' -- it does not describe this route, so every"
                        + " comparison below it is meaningless. Re-record it once the route is"
                        + " approved: -DupdateAutoGolden=true -DconfirmGoldenUpdate=true.");

        // Direction matters for diagnosis, so it is reported rather than flattened into one word.
        // 11 of the 13 recorded goldens bank completed=false: those routes run out the harness's
        // real-time cap instead of finishing. A false->true delta is therefore a route that STARTED
        // finishing -- progress to confirm and re-record, not a defect to hunt. Calling both
        // directions "regressed" pointed the next reader at a break that isn't there.
        if (golden.completed != actual.completed) {
            fail(autoName + ": completed changed (golden=" + golden.completed
                    + ", actual=" + actual.completed + "). "
                    + (actual.completed
                            ? "The route now finishes inside the harness window -- an IMPROVEMENT."
                                    + " Confirm it is intended, then re-record this golden with"
                                    + " -DupdateAutoGolden=true -DconfirmGoldenUpdate=true."
                            : "The route stopped finishing inside the harness window -- a"
                                    + " REGRESSION. Diagnose the cause before touching the golden."));
        }
        assertTrue(
                Math.abs(actual.runtimeSeconds - golden.runtimeSeconds)
                        <= AutoRegressionTolerances.kRuntimeToleranceSeconds,
                () -> autoName + ": runtimeSeconds " + actual.runtimeSeconds + " outside golden "
                        + golden.runtimeSeconds + " +/- " + AutoRegressionTolerances.kRuntimeToleranceSeconds + "s");
        assertTrue(
                actual.maxLateralErrorMeters
                        <= golden.maxLateralErrorMeters + AutoRegressionTolerances.kLateralErrorHeadroomMeters,
                () -> autoName + ": maxLateralErrorMeters " + actual.maxLateralErrorMeters
                        + " exceeds golden " + golden.maxLateralErrorMeters + " + "
                        + AutoRegressionTolerances.kLateralErrorHeadroomMeters + "m headroom");
        assertTrue(
                actual.maxLongitudinalErrorMeters
                        <= golden.maxLongitudinalErrorMeters + AutoRegressionTolerances.kLongitudinalErrorHeadroomMeters,
                () -> autoName + ": maxLongitudinalErrorMeters " + actual.maxLongitudinalErrorMeters
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
