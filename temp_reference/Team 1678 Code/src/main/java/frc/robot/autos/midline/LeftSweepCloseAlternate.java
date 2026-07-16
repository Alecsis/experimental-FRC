package frc.robot.autos.midline;

import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.autos.AutoHelpers;
import frc.robot.autos.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;

/**
 * Sweep and Shoot Auto Routine that stays on the Left Side of the field
 */
public class LeftSweepCloseAlternate extends AutoModeBase {

	AutoTrajectory leftStartToLine = trajectory("startToScoreClose");
	AutoTrajectory scoreToSecondSweep = trajectory("scoreToSecondSweepAlternate");
	AutoTrajectory secondScoreToNeutral = trajectory("secondScoreToNeutral");

	static SwerveRequest.FieldCentric velocityRequest = new SwerveRequest.FieldCentric()
			.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
			.withDriveRequestType(DriveRequestType.Velocity);

	public LeftSweepCloseAlternate(AutoFactory factory) {
		super(factory, "Left Sweep Close Alternate");
		logTrajectories(leftStartToLine, scoreToSecondSweep);

		prepRoutine(
				cmdWithCollisonAvoidance(leftStartToLine, velocityRequest),
				Commands.defer(
						() -> driveOverBump(
								velocityRequest, Rotation2d.fromDegrees(RobotConstants.isRedAlliance ? -45.0 : 135.0)),
						Set.of(Drive.mInstance)),
				AutoHelpers.fireCommand(Units.Seconds.of(3.0), Seconds.of(0.75)),
				cmdWithCollisonAvoidance(scoreToSecondSweep, velocityRequest),
				Commands.defer(
						() -> driveOverBump(
								velocityRequest,
								Rotation2d.fromDegrees(RobotConstants.isRedAlliance ? -38.37 : 141.15)),
						Set.of(Drive.mInstance)),
				AutoHelpers.fireCommand(Units.Seconds.of(3.0), Seconds.of(0.75)),
				secondScoreToNeutral.cmd());
	}

	@Override
	public Pose2d getInitialPose() {
		return leftStartToLine.getInitialPose().get();
	}
}
