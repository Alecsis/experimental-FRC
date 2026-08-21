package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Phase 0.5 of docs/Autonomous_Recovery_Readiness_Assessment.md ("Autonomous Observability
 * Layer") -- aggregates already-existing autonomous telemetry signals (trajectory tracking
 * error, pose motion, vision rejection) into read-only health queries and logs them under
 * {@code Auto/Health/*}. Deliberately observability-only: {@link #update()}/{@link #periodic()}
 * never reference {@code CommandScheduler} and never cancel or alter any command -- that is
 * explicitly out of scope until the chassis-PID divergence bug the readiness assessment
 * identifies as a blocker is addressed (see that doc's §1 and §4).
 *
 * <p>Every threshold below is a placeholder, not a tuned value -- per the readiness assessment's
 * own conclusion, none of these signals should be trusted to drive a real decision until the
 * chassis-PID bug is fixed and the disturbance experiment is re-run against a stable baseline.
 * This class only builds the plumbing that decision would eventually read from.
 *
 * <p>Supplier-injected, matching {@link frc.robot.utility.TrajectoryErrorTracker}'s own testable
 * design -- {@link #update()} has no HAL/robot-boot dependency and is fully unit-testable with
 * synthetic suppliers; only {@link #periodic()}'s {@code Logger.recordOutput} calls need a real
 * AdvantageKit pipeline.
 */
public class AutonomousHealthMonitor {

    /** Why a health-degraded condition (if any) is currently active. Not a recovery trigger --
     * nothing reads this to cancel or alter a command yet. */
    public enum RecoveryReason {
        NONE, TRACKING_DEGRADED, STALLED, VISION_UNHEALTHY
    }

    // Placeholder thresholds -- see class javadoc. Mirrors AutoRegressionTolerances' existing
    // stall window/threshold values (src/test/java/frc/robot/auto/AutoRegressionTolerances.java)
    // rather than inventing new ones, but declared fresh here since production code must not
    // depend on the test source set.
    static final double kTrackingDegradedLateralMeters = 1.0;
    static final double kTrackingDegradedLongitudinalMeters = 1.0;
    static final double kStallWindowSeconds = 1.0;
    static final double kStallTranslationThresholdMeters = 0.05;
    /**
     * Per-sample translation at or below this contributes nothing to the accumulated path length
     * below -- a pose-noise floor, not a tuning knob.
     *
     * <p>Summing consecutive-sample distances (rather than comparing the window's endpoints) is
     * what makes a direction reversal count as the travel it actually is, but the same sum also
     * integrates pose noise: N noisy samples each off by j accumulate N*j of fictitious travel,
     * which would mask a genuine stall. This floor is the guard, and it is sized so it cannot cut
     * the other way: a robot moving at exactly the stall rate this class exists to detect
     * (kStallTranslationThresholdMeters over kStallWindowSeconds) covers 1.0 mm per 20 ms
     * robotPeriodic tick, so 0.1 mm leaves a full order of magnitude of headroom.
     *
     * <p>Measured, this is dead code in simulation: MapleSim odometry reports a stopped robot as
     * bit-identical, 0.000000 m per sample (logs/akit_26-08-20_01-46-03.wpilog, t >= 11.5 s, 71
     * consecutive samples). It exists for real hardware. Note the inherent limit it cannot fix --
     * per-sample jitter above ~1 mm defeats ANY accumulated-length stall rule at this threshold,
     * because the fictitious travel then exceeds the threshold on its own.
     */
    static final double kStallSampleNoiseFloorMeters = 1.0e-4;
    static final double kStalePathSetpointGraceSeconds = 0.5;
    static final int kVisionUnhealthyConsecutiveRejections = 5;

    private final DoubleSupplier lateralErrorMetersSupplier;
    private final DoubleSupplier longitudinalErrorMetersSupplier;
    private final Supplier<Pose2d> measuredPoseSupplier;
    private final DoubleSupplier timestampSecondsSupplier;
    private final DoubleSupplier secondsSinceLastFreshTargetPoseSupplier;
    private final DoubleSupplier rejectedJumpMetersSupplier;

    // {t, x, y} samples within the trailing kStallWindowSeconds, oldest first. Cleared whenever
    // path-following isn't currently active, so a stationary named-command phase (every auto's
    // final aim-and-shoot segment) never gets judged against motion from the prior path segment.
    private final Deque<double[]> poseHistory = new ArrayDeque<>();
    /**
     * Timestamp of the first sample of the current uninterrupted path-following stretch, or NaN
     * while path-following is inactive. Exists only so {@link #updateStalled()} can tell whether a
     * full {@link #kStallWindowSeconds} of history has accumulated; {@link #poseHistory} cannot
     * answer that on its own because its trim discards everything older than the window.
     */
    private double pathFollowingSinceTimestamp = Double.NaN;
    private int consecutiveVisionRejections;

    private boolean trackingDegraded;
    private boolean stalled;
    private boolean visionUnhealthy;
    private RecoveryReason recoveryReason = RecoveryReason.NONE;

    public AutonomousHealthMonitor(
            DoubleSupplier lateralErrorMetersSupplier,
            DoubleSupplier longitudinalErrorMetersSupplier,
            Supplier<Pose2d> measuredPoseSupplier,
            DoubleSupplier timestampSecondsSupplier,
            DoubleSupplier secondsSinceLastFreshTargetPoseSupplier,
            DoubleSupplier rejectedJumpMetersSupplier) {
        this.lateralErrorMetersSupplier = lateralErrorMetersSupplier;
        this.longitudinalErrorMetersSupplier = longitudinalErrorMetersSupplier;
        this.measuredPoseSupplier = measuredPoseSupplier;
        this.timestampSecondsSupplier = timestampSecondsSupplier;
        this.secondsSinceLastFreshTargetPoseSupplier = secondsSinceLastFreshTargetPoseSupplier;
        this.rejectedJumpMetersSupplier = rejectedJumpMetersSupplier;
    }

    /** Pure computation, no logging -- safe to call without AdvantageKit's Logger started. */
    public void update() {
        updateTrackingDegraded();
        updateStalled();
        updateVisionUnhealthy();
        updateRecoveryReason();
    }

    /** {@link #update()} plus logging. Call once per robotPeriodic() cycle. */
    public void periodic() {
        update();
        log();
    }

    private void updateTrackingDegraded() {
        double lateral = lateralErrorMetersSupplier.getAsDouble();
        double longitudinal = longitudinalErrorMetersSupplier.getAsDouble();
        boolean lateralBad = !Double.isNaN(lateral) && Math.abs(lateral) > kTrackingDegradedLateralMeters;
        boolean longitudinalBad =
                !Double.isNaN(longitudinal) && Math.abs(longitudinal) > kTrackingDegradedLongitudinalMeters;
        trackingDegraded = lateralBad || longitudinalBad;
    }

    private void updateStalled() {
        boolean pathFollowingActive =
                secondsSinceLastFreshTargetPoseSupplier.getAsDouble() <= kStalePathSetpointGraceSeconds;
        if (!pathFollowingActive) {
            stalled = false;
            poseHistory.clear();
            pathFollowingSinceTimestamp = Double.NaN;
            return;
        }

        double t = timestampSecondsSupplier.getAsDouble();
        Pose2d pose = measuredPoseSupplier.get();
        if (poseHistory.isEmpty()) {
            pathFollowingSinceTimestamp = t;
        }
        poseHistory.addLast(new double[] {t, pose.getX(), pose.getY()});
        while (poseHistory.size() > 1 && poseHistory.peekFirst()[0] < t - kStallWindowSeconds) {
            poseHistory.pollFirst();
        }

        // Mirrors WpilogStallAnalyzer's "if (windowStart < 0) continue;": refuse to judge until a
        // FULL kStallWindowSeconds of history exists. kStallTranslationThresholdMeters is defined
        // as the travel required over a whole window, so grading a partial window against it
        // silently applies a much stricter rate -- 0.05 m over the 0.2 s available just after
        // path-following starts is 0.25 m/s, five times the intended 0.05 m/s bar. That is what
        // produced the spurious startup pulse this class used to log: measured Stalled=true from
        // t = 0.204 s to t = 0.364 s in all three RB Neutral runs
        // (logs/akit_26-08-20_01-46-03.wpilog, akit_26-08-19_18-07-00.wpilog,
        // akit_26-08-19_16-36-12.wpilog), clearing when accumulated travel crossed the threshold
        // rather than when the window filled.
        //
        // The trailing cost is that a stall inside the first second of a path is not reported. That
        // is accepted for the same reason the analyzer accepts it: during initial acceleration a
        // stalled robot and a healthy one are not yet distinguishable, and the analyzer already
        // gates the regression suite on exactly this rule.
        // Note this is measured from when path-following BEGAN, not from the deque's oldest retained
        // sample: the trim above discards everything older than the window, so the deque's own span
        // can never reveal whether a full window has elapsed.
        if (t - pathFollowingSinceTimestamp < kStallWindowSeconds) {
            stalled = false;
            return;
        }
        stalled = accumulatedPathLengthMeters() < kStallTranslationThresholdMeters;
    }

    /**
     * Translation actually travelled across the samples currently inside the stall window, summed
     * consecutive pair by consecutive pair.
     *
     * <p>Deliberately NOT {@code hypot(newest - oldest)}. Endpoint displacement asks "is the robot
     * somewhere else than it was a second ago", which is a different question from "is the robot
     * moving" and answers it wrongly whenever a route reverses direction inside one window. Measured
     * on RB Neutral, which decelerates to rest at Neutral Position 3 and immediately drives back the
     * other way: at t = 2.980 s the endpoint rule saw 0.0486 m (under the 0.05 m threshold, so it
     * reported a stall) while the robot had in fact covered 0.744 m of path -- 14x the threshold.
     * That false positive reproduced in 3 of 3 runs (logs/akit_26-08-20_01-46-03.wpilog,
     * akit_26-08-19_18-07-00.wpilog, akit_26-08-19_16-36-12.wpilog: {@code Auto/Health/Stalled}
     * true at t = 2.983-3.043 s).
     *
     * <p>Kept mirrored with {@code WpilogStallAnalyzer}'s copy, which grades the same rule post-hoc
     * from a wpilog; see that class for why the duplication exists.
     */
    private double accumulatedPathLengthMeters() {
        double total = 0.0;
        double[] previous = null;
        for (double[] sample : poseHistory) {
            if (previous != null) {
                double step = Math.hypot(sample[1] - previous[1], sample[2] - previous[2]);
                if (step > kStallSampleNoiseFloorMeters) {
                    total += step;
                }
            }
            previous = sample;
        }
        return total;
    }

    private void updateVisionUnhealthy() {
        double rejected = rejectedJumpMetersSupplier.getAsDouble();
        if (rejected > 0.0) {
            consecutiveVisionRejections++;
        } else {
            consecutiveVisionRejections = 0;
        }
        visionUnhealthy = consecutiveVisionRejections >= kVisionUnhealthyConsecutiveRejections;
    }

    private void updateRecoveryReason() {
        if (trackingDegraded) {
            recoveryReason = RecoveryReason.TRACKING_DEGRADED;
        } else if (stalled) {
            recoveryReason = RecoveryReason.STALLED;
        } else if (visionUnhealthy) {
            recoveryReason = RecoveryReason.VISION_UNHEALTHY;
        } else {
            recoveryReason = RecoveryReason.NONE;
        }
    }

    private void log() {
        Logger.recordOutput("Auto/Health/TrackingDegraded", trackingDegraded);
        Logger.recordOutput("Auto/Health/Stalled", stalled);
        Logger.recordOutput("Auto/Health/VisionUnhealthy", visionUnhealthy);
        Logger.recordOutput("Auto/Health/RecoveryReason", recoveryReason.name());
    }

    public boolean isTrackingDegraded() {
        return trackingDegraded;
    }

    public boolean isStalled() {
        return stalled;
    }

    public boolean isVisionUnhealthy() {
        return visionUnhealthy;
    }

    public RecoveryReason getRecoveryReason() {
        return recoveryReason;
    }
}
