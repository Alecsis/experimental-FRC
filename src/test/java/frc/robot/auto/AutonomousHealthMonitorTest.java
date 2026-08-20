package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Phase 0.5 (Autonomous Observability Layer) of
 * docs/Autonomous_Recovery_Readiness_Assessment.md -- AutonomousHealthMonitor aggregates
 * already-existing autonomous telemetry signals into read-only health queries. It never touches
 * CommandScheduler; every method below either exercises pure computation via injected suppliers
 * (no robot boot needed, matching TrajectoryErrorTracker's own testable design) or, for the one
 * integration check at the bottom, boots a real Robot() the same way RobotAutoTerminationTelemetryTest
 * does -- only ONE @Test method here constructs a Robot(), since build.gradle's forkEvery = 1
 * means every AdvantageKit Logger.start() call in the same test class must be unique per class,
 * not per method (discovered building CommandSwerveDrivetrainSysIdReverseSimWorkflowTest).
 */
class AutonomousHealthMonitorTest {
    private static final double kNominalLateralErrorMeters = 0.1;
    private static final double kNominalLongitudinalErrorMeters = 0.1;

    private double clockSeconds;
    private double lateralErrorMeters;
    private double longitudinalErrorMeters;
    private Pose2d measuredPose = new Pose2d();
    // Defaults to "not currently path-following" so tests unrelated to Stalled (e.g. the
    // VisionUnhealthy tests below, which never advance clockSeconds) can't accidentally also
    // trip the stall detector as an unintended side effect -- tests exercising Stalled override
    // this explicitly.
    private double secondsSinceLastFreshTargetPose = Double.POSITIVE_INFINITY;
    private double rejectedJumpMeters;

    private final AutonomousHealthMonitor monitor = new AutonomousHealthMonitor(
            () -> lateralErrorMeters,
            () -> longitudinalErrorMeters,
            () -> measuredPose,
            () -> clockSeconds,
            () -> secondsSinceLastFreshTargetPose,
            () -> rejectedJumpMeters);

    // ---- Defaults / healthy path ----

    @Test
    void defaultsToHealthyBeforeUpdateIsEverCalled() {
        assertFalse(monitor.isTrackingDegraded());
        assertFalse(monitor.isStalled());
        assertFalse(monitor.isVisionUnhealthy());
        assertEquals(AutonomousHealthMonitor.RecoveryReason.NONE, monitor.getRecoveryReason());
    }

    @Test
    void staysHealthyWithNominalSignals() {
        lateralErrorMeters = kNominalLateralErrorMeters;
        longitudinalErrorMeters = kNominalLongitudinalErrorMeters;
        secondsSinceLastFreshTargetPose = Double.POSITIVE_INFINITY; // never path-following
        rejectedJumpMeters = 0.0;

        monitor.update();

        assertFalse(monitor.isTrackingDegraded());
        assertFalse(monitor.isStalled());
        assertFalse(monitor.isVisionUnhealthy());
        assertEquals(AutonomousHealthMonitor.RecoveryReason.NONE, monitor.getRecoveryReason());
    }

    // ---- TrackingDegraded ----

    @Test
    void trackingDegradedWhenLateralErrorExceedsThreshold() {
        lateralErrorMeters = AutonomousHealthMonitor.kTrackingDegradedLateralMeters + 0.01;
        longitudinalErrorMeters = 0.0;

        monitor.update();

        assertTrue(monitor.isTrackingDegraded());
        assertEquals(AutonomousHealthMonitor.RecoveryReason.TRACKING_DEGRADED, monitor.getRecoveryReason());
    }

    @Test
    void trackingDegradedWhenLongitudinalErrorExceedsThreshold() {
        lateralErrorMeters = 0.0;
        longitudinalErrorMeters = AutonomousHealthMonitor.kTrackingDegradedLongitudinalMeters + 0.01;

        monitor.update();

        assertTrue(monitor.isTrackingDegraded());
    }

    @Test
    void notTrackingDegradedWhenErrorIsNaN() {
        // TrajectoryErrorTracker reports NaN before any setpoint has ever arrived -- must not be
        // misread as "infinitely degraded".
        lateralErrorMeters = Double.NaN;
        longitudinalErrorMeters = Double.NaN;

        monitor.update();

        assertFalse(monitor.isTrackingDegraded());
    }

    // ---- Stalled ----

    @Test
    void notStalledWhenNotCurrentlyPathFollowing() {
        secondsSinceLastFreshTargetPose = 999.0; // long past AutonomousHealthMonitor's grace window
        measuredPose = new Pose2d();
        clockSeconds = 0.0;
        monitor.update();

        clockSeconds = 2.0; // robot hasn't moved at all
        monitor.update();

        assertFalse(monitor.isStalled(), "no motion is expected/correct outside path-following");
    }

    @Test
    void stalledAfterSustainedNoMotionDuringPathFollowing() {
        // Dense sample stream (every 0.1s), matching how robotPeriodic() actually drives this in
        // production (every ~0.02s) -- a single before/after jump can't exercise the rolling
        // window's own "do we have enough history yet" bookkeeping correctly.
        secondsSinceLastFreshTargetPose = 0.0; // actively path-following every tick below
        measuredPose = new Pose2d(1.0, 1.0, measuredPose.getRotation()); // never moves
        for (clockSeconds = 0.0; clockSeconds <= AutonomousHealthMonitor.kStallWindowSeconds + 0.5;
                clockSeconds += 0.1) {
            monitor.update();
        }

        assertTrue(monitor.isStalled());
        assertEquals(AutonomousHealthMonitor.RecoveryReason.STALLED, monitor.getRecoveryReason());
    }

    @Test
    void notStalledWhenMovingEnoughDuringPathFollowing() {
        secondsSinceLastFreshTargetPose = 0.0;
        for (clockSeconds = 0.0; clockSeconds <= AutonomousHealthMonitor.kStallWindowSeconds + 0.5;
                clockSeconds += 0.1) {
            // Moves steadily -- well over kStallTranslationThresholdMeters by the time a full
            // window of history has accumulated.
            measuredPose = new Pose2d(clockSeconds, 0.0, measuredPose.getRotation());
            monitor.update();
        }

        assertFalse(monitor.isStalled());
    }

    @Test
    void notStalledBeforeAFullStallWindowOfHistoryExists() {
        // kStallTranslationThresholdMeters is "travel required over a whole window". Judging a
        // partial window against it applies a far stricter rate (0.05 m over 0.2 s is 0.25 m/s,
        // 5x the intended bar) and is what produced the measured startup pulse -- Stalled=true from
        // t=0.204s to t=0.364s in all three recorded RB Neutral runs.
        secondsSinceLastFreshTargetPose = 0.0;
        measuredPose = new Pose2d(1.0, 1.0, measuredPose.getRotation()); // never moves
        boolean everStalled = false;
        for (clockSeconds = 0.0;
                clockSeconds < AutonomousHealthMonitor.kStallWindowSeconds - 1e-9;
                clockSeconds += 0.02) {
            monitor.update();
            everStalled |= monitor.isStalled();
        }

        assertFalse(everStalled,
                "no verdict may be produced until a full window of path-following history exists,"
                        + " even for a robot that never moved");
        assertEquals(AutonomousHealthMonitor.RecoveryReason.NONE, monitor.getRecoveryReason());
    }

    @Test
    void stalledOnceTheWindowFillsEvenThoughItWasSuppressedEarlier() {
        // The guard must DELAY the verdict, not suppress it. Same motionless robot as above, run
        // past the window boundary.
        secondsSinceLastFreshTargetPose = 0.0;
        measuredPose = new Pose2d(1.0, 1.0, measuredPose.getRotation());
        for (clockSeconds = 0.0;
                clockSeconds <= AutonomousHealthMonitor.kStallWindowSeconds + 0.5;
                clockSeconds += 0.02) {
            monitor.update();
        }

        assertTrue(monitor.isStalled());
        assertEquals(AutonomousHealthMonitor.RecoveryReason.STALLED, monitor.getRecoveryReason());
    }

    @Test
    void notStalledAfterAFullWindowOfRealTravel() {
        secondsSinceLastFreshTargetPose = 0.0;
        for (clockSeconds = 0.0;
                clockSeconds <= AutonomousHealthMonitor.kStallWindowSeconds + 0.5;
                clockSeconds += 0.02) {
            measuredPose = new Pose2d(clockSeconds * 0.5, 0.0, measuredPose.getRotation());
            monitor.update();
        }

        assertFalse(monitor.isStalled(), "0.5 m of travel per window is ten times the threshold");
    }

    @Test
    void windowRestartsAfterPathFollowingLapses() {
        // Losing path-following clears the history, so the guard must re-arm: a stall cannot be
        // declared on the strength of samples gathered before the gap.
        secondsSinceLastFreshTargetPose = 0.0;
        measuredPose = new Pose2d(1.0, 1.0, measuredPose.getRotation());
        for (clockSeconds = 0.0; clockSeconds <= 2.0; clockSeconds += 0.02) {
            monitor.update();
        }
        assertTrue(monitor.isStalled(), "precondition: a full motionless window is a stall");

        secondsSinceLastFreshTargetPose = 999.0; // path-following lapses, history cleared
        monitor.update();
        assertFalse(monitor.isStalled());

        secondsSinceLastFreshTargetPose = 0.0; // a new path starts
        for (clockSeconds = 2.02; clockSeconds <= 2.60; clockSeconds += 0.02) {
            monitor.update();
        }

        assertFalse(monitor.isStalled(),
                "only ~0.6 s into the new path-following stretch -- the window must have re-armed");
    }

    @Test
    void notStalledWhenReversingDirectionDuringPathFollowing() {
        // RB Neutral's shape: out to Neutral Position 3, then straight back along
        // "Bump Neutral Right". Endpoint displacement across the trailing window collapses toward
        // zero mid-turnaround even though the robot is driving the whole time, which is exactly
        // how this monitor logged Stalled=true at t=2.98s in 3 of 3 recorded RB Neutral runs.
        // Accumulated path length sees the ~2 m actually travelled. The real recorded samples are
        // replayed through this same monitor in StallDetectorMirroringTest.
        //
        // Graded across every tick, including the startup ramp: the full-window guard in
        // updateStalled() means no verdict is produced at all until 1.0 s of path-following history
        // exists, so there is no startup transient left to exclude here.
        secondsSinceLastFreshTargetPose = 0.0;
        boolean everStalled = false;
        for (clockSeconds = 0.0; clockSeconds <= 2.0 + 1e-9; clockSeconds += 0.02) {
            double x = clockSeconds <= 1.0 ? clockSeconds : 2.0 - clockSeconds;
            measuredPose = new Pose2d(x, 0.0, measuredPose.getRotation());
            monitor.update();
            everStalled |= monitor.isStalled();
        }

        assertFalse(everStalled, "a direction reversal is motion, not a stall");
    }

    @Test
    void stalledWhenOnlyPoseNoiseAccumulatesDuringPathFollowing() {
        // Guards the cost of summing consecutive samples: pose noise integrates too. Per-sample
        // steps at or below kStallSampleNoiseFloorMeters contribute nothing, so a motionless but
        // noisy robot cannot accumulate its way out of a genuine stall.
        secondsSinceLastFreshTargetPose = 0.0;
        double amplitude = AutonomousHealthMonitor.kStallSampleNoiseFloorMeters / 4.0;
        int tick = 0;
        for (clockSeconds = 0.0; clockSeconds <= AutonomousHealthMonitor.kStallWindowSeconds + 0.5;
                clockSeconds += 0.02, tick++) {
            measuredPose = new Pose2d(1.0 + (tick % 2 == 0 ? amplitude : -amplitude), 1.0,
                    measuredPose.getRotation());
            monitor.update();
        }

        assertTrue(monitor.isStalled(),
                "pose noise below the per-sample floor must not be mistaken for travel");
        assertEquals(AutonomousHealthMonitor.RecoveryReason.STALLED, monitor.getRecoveryReason());
    }

    // ---- VisionUnhealthy ----

    @Test
    void visionUnhealthyAfterConsecutiveRejections() {
        rejectedJumpMeters = 1.5; // > 0 means "rejected this cycle"
        for (int i = 0; i < AutonomousHealthMonitor.kVisionUnhealthyConsecutiveRejections; i++) {
            monitor.update();
        }

        assertTrue(monitor.isVisionUnhealthy());
        assertEquals(AutonomousHealthMonitor.RecoveryReason.VISION_UNHEALTHY, monitor.getRecoveryReason());
    }

    @Test
    void notVisionUnhealthyBelowTheConsecutiveRejectionThreshold() {
        rejectedJumpMeters = 1.5;
        for (int i = 0; i < AutonomousHealthMonitor.kVisionUnhealthyConsecutiveRejections - 1; i++) {
            monitor.update();
        }

        assertFalse(monitor.isVisionUnhealthy());
    }

    @Test
    void visionHealthyStreakResetsAfterAnAcceptedMeasurement() {
        rejectedJumpMeters = 1.5;
        for (int i = 0; i < AutonomousHealthMonitor.kVisionUnhealthyConsecutiveRejections; i++) {
            monitor.update();
        }
        assertTrue(monitor.isVisionUnhealthy());

        rejectedJumpMeters = 0.0; // one accepted (or simply non-rejecting) cycle
        monitor.update();

        assertFalse(monitor.isVisionUnhealthy());
    }

    // ---- RecoveryReason priority (only matters when several signals are true at once) ----

    @Test
    void recoveryReasonPrefersTrackingDegradedOverOtherActiveSignals() {
        lateralErrorMeters = AutonomousHealthMonitor.kTrackingDegradedLateralMeters + 0.01;
        rejectedJumpMeters = 1.5;
        for (int i = 0; i < AutonomousHealthMonitor.kVisionUnhealthyConsecutiveRejections; i++) {
            monitor.update();
        }

        assertEquals(AutonomousHealthMonitor.RecoveryReason.TRACKING_DEGRADED, monitor.getRecoveryReason());
    }

    // ---- update() must never touch CommandScheduler ----

    @Test
    void updateNeverSchedulesOrCancelsCommands() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        WaitCommand probe = new WaitCommand(5);
        scheduler.schedule(probe);

        lateralErrorMeters = 999.0;
        rejectedJumpMeters = 5.0;
        for (int i = 0; i < 20; i++) {
            monitor.update();
        }

        assertTrue(scheduler.isScheduled(probe),
                "AutonomousHealthMonitor.update() must be read-only -- it must never cancel a command");
        scheduler.cancel(probe);
    }

    // ---- Integration: real Robot() boot, proves the wiring + telemetry hook + Vision getter ----

    @Test
    @Timeout(30)
    void periodicLogsHealthTelemetryWithoutAlteringCommandLifecycle() throws IOException, InterruptedException {
        assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
        DriverStationSim.resetData();
        SimHooks.pauseTiming();

        Robot robot = new Robot();
        Thread robotThread = new Thread(robot::startCompetition, "AutonomousHealthMonitorTest-competition");
        robotThread.setDaemon(true);
        robotThread.start();
        SimHooks.waitForProgramStart();

        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        WaitCommand probe = new WaitCommand(0.2);
        CommandScheduler.getInstance().schedule(probe);
        SimHooks.stepTiming(0.02);
        SimHooks.resumeTiming();

        Thread.sleep(600);

        assertFalse(CommandScheduler.getInstance().isScheduled(probe),
                "a plain 0.2s WaitCommand should still finish naturally with the health-monitor hook wired in");

        robot.endCompetition();
        robotThread.join(1000);
        assertFalse(robotThread.isAlive());
        robot.close();

        Path wpilog = findNewestWpilog();
        assertTrue(wpilog != null, "No .wpilog found in logs/ after the run");

        Set<String> expectedEntries = new HashSet<>(Set.of(
                "/RealOutputs/Auto/Health/TrackingDegraded",
                "/RealOutputs/Auto/Health/Stalled",
                "/RealOutputs/Auto/Health/VisionUnhealthy",
                "/RealOutputs/Auto/Health/RecoveryReason"));
        Set<String> foundEntries = new HashSet<>();
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IOException("Not a valid WPILOG file: " + wpilog);
        }
        for (DataLogRecord record : reader) {
            if (record.isStart()) {
                String name = record.getStartData().name;
                if (expectedEntries.contains(name)) {
                    foundEntries.add(name);
                }
            }
        }
        assertEquals(expectedEntries, foundEntries,
                "every Auto/Health/* telemetry key should have been logged at least once");
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
