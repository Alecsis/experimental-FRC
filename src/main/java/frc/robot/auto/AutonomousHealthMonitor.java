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
            return;
        }

        double t = timestampSecondsSupplier.getAsDouble();
        Pose2d pose = measuredPoseSupplier.get();
        poseHistory.addLast(new double[] {t, pose.getX(), pose.getY()});
        while (poseHistory.size() > 1 && poseHistory.peekFirst()[0] < t - kStallWindowSeconds) {
            poseHistory.pollFirst();
        }

        if (poseHistory.size() == 1) {
            // Mirrors WpilogStallAnalyzer's "windowStart == i" check: only the just-added current
            // sample exists, so there is no earlier reference point yet (true only during the
            // startup transient, the first kStallWindowSeconds after path-following begins).
            stalled = false;
            return;
        }
        double[] oldest = poseHistory.peekFirst();
        double moved = Math.hypot(pose.getX() - oldest[1], pose.getY() - oldest[2]);
        stalled = moved < kStallTranslationThresholdMeters;
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
