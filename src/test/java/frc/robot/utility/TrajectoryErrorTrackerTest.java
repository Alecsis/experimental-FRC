package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

/**
 * Pure unit tests -- no HAL/robot boot needed, matching TrajectoryErrorTracker's own
 * supplier-injected design. Covers only the new live "how long since a path setpoint last
 * arrived" signal (getSecondsSinceLastFreshTargetPose()) added for the Autonomous Observability
 * Layer (docs/Autonomous_Recovery_Readiness_Assessment.md's Phase 0.5) -- the pre-existing
 * error-computation behavior is unchanged and already covered by the auto-regression suite's
 * wpilog-based checks.
 */
class TrajectoryErrorTrackerTest {
    private double clockSeconds;
    private final TrajectoryErrorTracker tracker =
            new TrajectoryErrorTracker(() -> new Pose2d(), () -> clockSeconds);

    @Test
    void secondsSinceLastFreshTargetPoseIsInfiniteBeforeAnyTargetPoseArrives() {
        assertEquals(Double.POSITIVE_INFINITY, tracker.getSecondsSinceLastFreshTargetPose());
    }

    @Test
    void secondsSinceLastFreshTargetPoseIsZeroRightAfterAFreshDelivery() {
        clockSeconds = 5.0;
        tracker.onTargetPose(new Pose2d(1, 0, new Rotation2d()));
        tracker.periodic();

        assertEquals(0.0, tracker.getSecondsSinceLastFreshTargetPose(), 1e-9);
    }

    @Test
    void secondsSinceLastFreshTargetPoseGrowsAsTimePassesWithNoNewDelivery() {
        clockSeconds = 5.0;
        tracker.onTargetPose(new Pose2d(1, 0, new Rotation2d()));
        tracker.periodic();

        clockSeconds = 5.3;
        tracker.periodic(); // no onTargetPose() call this cycle -- freshness pulse has passed

        assertEquals(0.3, tracker.getSecondsSinceLastFreshTargetPose(), 1e-9);
    }

    @Test
    void secondsSinceLastFreshTargetPoseTracksTheMostRecentDelivery() {
        clockSeconds = 1.0;
        tracker.onTargetPose(new Pose2d(1, 0, new Rotation2d()));
        tracker.periodic();

        clockSeconds = 1.5;
        tracker.onTargetPose(new Pose2d(2, 0, new Rotation2d()));
        tracker.periodic();

        clockSeconds = 1.6;
        assertTrue(tracker.getSecondsSinceLastFreshTargetPose() < 0.2,
                "should measure from the second (most recent) delivery, not the first");
    }
}
