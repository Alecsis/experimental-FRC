package frc.robot.utility;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

/**
 * Compares PathPlanner's per-loop setpoint pose against the drivetrain's measured pose.
 * Owned by RobotContainer (constructed with the drivetrain's pose supplier), not a singleton --
 * exactly one place needs a handle to this. See docs/claudex/design/trajectory-error-instrumentation.md.
 *
 * <p>{@link #onActivePath} and {@link #onTargetPose} are ingest-only -- called from
 * CommandSwerveDrivetrain's existing PathPlannerLogging callbacks to store the latest data.
 * They must stay side-effect-free (no computation, no logging); periodic() is the only place
 * that reads what they stored.
 */
public class TrajectoryErrorTracker {
    // Below this, a setpoint-to-setpoint delta is numerically indistinguishable from noise (e.g.
    // a rotate-in-place segment, or the first/last sample of a path) and its angle is meaningless.
    private static final double kTangentEpsilonMeters = 1e-4;

    private final Supplier<Pose2d> measuredPoseSupplier;
    private final DoubleSupplier timestampSecondsSupplier;

    private List<Pose2d> latestActivePath = List.of();
    private Pose2d latestTargetPose = null;

    // Set only when a fresh target pose actually arrives (see logAndClearFreshness()); NaN means
    // "no path setpoint has ever been delivered". Backs getSecondsSinceLastFreshTargetPose(), the
    // live equivalent of AutoRegressionTolerances.kStalePathSetpointGraceSeconds's post-hoc grace
    // window -- lets a live consumer (AutonomousHealthMonitor) tell "still path-following" from
    // "path-following ended, now in a stationary named-command phase" without re-deriving the
    // same aliasing problem WpilogStallAnalyzer's own comment already documents.
    private double lastFreshTargetPoseTimestampSeconds = Double.NaN;

    // Path tangent is derived from consecutive per-loop setpoints (this loop vs. last), not from
    // latestActivePath -- there is no "current progress index" into that list, so it isn't a
    // synchronized sequence. previousSetpointTranslation/lastValidPathTangent carry that state
    // across periodic() calls. See docs/claudex/design/trajectory-error-instrumentation.md #2.2.
    private Translation2d previousSetpointTranslation = null;
    private Rotation2d lastValidPathTangent = null;

    private double longitudinalErrorMeters = Double.NaN;
    private double lateralErrorMeters = Double.NaN;
    private double headingErrorRadians = Double.NaN;

    // True only for the periodic() call following an onTargetPose() delivery -- represents "a new
    // setpoint arrived this loop", not "the robot is successfully following the trajectory".
    private boolean hasFreshTargetPose;

    public TrajectoryErrorTracker(Supplier<Pose2d> measuredPoseSupplier, DoubleSupplier timestampSecondsSupplier) {
        this.measuredPoseSupplier = measuredPoseSupplier;
        this.timestampSecondsSupplier = timestampSecondsSupplier;
    }

    /** Stores PathPlanner's active path. Ingest only -- no computation, no logging. */
    public void onActivePath(List<Pose2d> activePath) {
        this.latestActivePath = activePath;
    }

    /** Stores PathPlanner's per-loop target pose. Ingest only -- no computation, no logging. */
    public void onTargetPose(Pose2d targetPose) {
        this.latestTargetPose = targetPose;
        this.hasFreshTargetPose = true;
    }

    /**
     * Recomputes lateral/longitudinal/heading error against the latest stored setpoint, then logs
     * them plus setpoint freshness. If no setpoint has ever arrived, or no path tangent has ever
     * been established, the corresponding outputs are NaN rather than 0.0, so a gap is visible
     * rather than a plausible lie -- and still logged, not skipped.
     */
    public void periodic() {
        Pose2d setpointPose = latestTargetPose;
        if (setpointPose == null) {
            longitudinalErrorMeters = Double.NaN;
            lateralErrorMeters = Double.NaN;
            headingErrorRadians = Double.NaN;
            logAndClearFreshness();
            return;
        }

        Pose2d measuredPose = measuredPoseSupplier.get();

        headingErrorRadians = setpointPose.getRotation().minus(measuredPose.getRotation()).getRadians();

        Translation2d setpointTranslation = setpointPose.getTranslation();
        if (previousSetpointTranslation != null) {
            Translation2d delta = setpointTranslation.minus(previousSetpointTranslation);
            if (delta.getNorm() >= kTangentEpsilonMeters) {
                lastValidPathTangent = delta.getAngle();
            }
        }
        previousSetpointTranslation = setpointTranslation;

        if (lastValidPathTangent == null) {
            longitudinalErrorMeters = Double.NaN;
            lateralErrorMeters = Double.NaN;
            logAndClearFreshness();
            return;
        }

        Translation2d fieldError = setpointTranslation.minus(measuredPose.getTranslation());
        Translation2d pathFrameError = fieldError.rotateBy(lastValidPathTangent.unaryMinus());
        longitudinalErrorMeters = pathFrameError.getX();
        lateralErrorMeters = -pathFrameError.getY();

        logAndClearFreshness();
    }

    private void logAndClearFreshness() {
        Logger.recordOutput("Trajectory/ErrorLateralMeters", lateralErrorMeters);
        Logger.recordOutput("Trajectory/ErrorLongitudinalMeters", longitudinalErrorMeters);
        Logger.recordOutput("Trajectory/ErrorHeadingRadians", headingErrorRadians);
        Logger.recordOutput("Trajectory/SetpointFresh", hasFreshTargetPose);
        if (hasFreshTargetPose) {
            lastFreshTargetPoseTimestampSeconds = timestampSecondsSupplier.getAsDouble();
        }
        hasFreshTargetPose = false;
    }

    public double getLongitudinalErrorMeters() {
        return longitudinalErrorMeters;
    }

    public double getLateralErrorMeters() {
        return lateralErrorMeters;
    }

    public double getHeadingErrorRadians() {
        return headingErrorRadians;
    }

    /**
     * Seconds elapsed since a path setpoint was last delivered via {@link #onTargetPose}, i.e.
     * "how long since path-following was last active". {@link Double#POSITIVE_INFINITY} if no
     * setpoint has ever arrived. A live gate for stall/degradation checks that should not fire
     * during an intentional stationary phase (e.g. every auto's final aim-and-shoot segment).
     */
    public double getSecondsSinceLastFreshTargetPose() {
        if (Double.isNaN(lastFreshTargetPoseTimestampSeconds)) {
            return Double.POSITIVE_INFINITY;
        }
        return timestampSecondsSupplier.getAsDouble() - lastFreshTargetPoseTimestampSeconds;
    }
}
