package frc.robot.utility;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

/**
 * Drop-in {@link PPHolonomicDriveController} that also logs the feedforward/feedback split of its
 * own output, for tracking-error root-cause analysis (distinguishing "the trajectory itself asks
 * for too much" from "the correction term is asking for too much"). PID gains are unchanged --
 * this only observes the vendor math, it does not alter it.
 *
 * <p>Exact, not estimated: {@code PPHolonomicDriveController.calculateRobotRelativeSpeeds()}
 * (PathplannerLib 2026.1.2, verified from the vendor's own source) sums feedforward and feedback
 * with no clamping in between (
 * {@code fromFieldRelativeSpeeds(xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, ...)}),
 * so {@code feedback = total - feedforward} recovers the feedback term exactly without needing
 * access to the vendor class's private PID controllers.
 */
public class LoggingHolonomicDriveController extends PPHolonomicDriveController {

    public LoggingHolonomicDriveController(PIDConstants translationConstants, PIDConstants rotationConstants) {
        super(translationConstants, rotationConstants);
    }

    @Override
    public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose, PathPlannerTrajectoryState targetState) {
        ChassisSpeeds total = super.calculateRobotRelativeSpeeds(currentPose, targetState);

        ChassisSpeeds feedforwardRobotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
                targetState.fieldSpeeds.vxMetersPerSecond,
                targetState.fieldSpeeds.vyMetersPerSecond,
                targetState.fieldSpeeds.omegaRadiansPerSecond,
                currentPose.getRotation());
        ChassisSpeeds feedback = total.minus(feedforwardRobotRelative);

        Logger.recordOutput("Trajectory/CommandedTotalSpeeds", total);
        Logger.recordOutput("Trajectory/CommandedFeedforwardSpeeds", feedforwardRobotRelative);
        Logger.recordOutput("Trajectory/CommandedFeedbackSpeeds", feedback);

        return total;
    }
}
