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
 * Sweep and Shoot Auto Routine mirrored to the opposite side of {@link LeftSweep}.
 */
public class RightSweep extends AutoModeBase {

	AutoTrajectory startToLine = mirroredTrajectory("startToScore");
	AutoTrajectory scoreToSecondSweep = mirroredTrajectory("scoreToSecondSweep");
	AutoTrajectory secondScoreToNeutral = mirroredTrajectory("secondScoreToNeutral");

	static SwerveRequest.FieldCentric velocityRequest = new SwerveRequest.FieldCentric()
			.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
			.withDriveRequestType(DriveRequestType.Velocity);

	public RightSweep(AutoFactory factory) {
		super(factory, "Right Sweep");
		logTrajectories(startToLine, scoreToSecondSweep);

		prepRoutine(
				cmdWithCollisonAvoidance(startToLine, velocityRequest),
				Commands.defer(
						() -> driveOverBump(
								velocityRequest, Rotation2d.fromDegrees(RobotConstants.isRedAlliance ? 45.0 : -135.0)),
						Set.of(Drive.mInstance)),
				AutoHelpers.fireCommand(Units.Seconds.of(3.0), Seconds.of(0.75)),
				cmdWithCollisonAvoidance(scoreToSecondSweep, velocityRequest),
				Commands.defer(
						() -> driveOverBump(
								velocityRequest,
								Rotation2d.fromDegrees(RobotConstants.isRedAlliance ? 38.37 : -141.15)),
						Set.of(Drive.mInstance)),
				AutoHelpers.fireCommand(Units.Seconds.of(3.0), Seconds.of(0.75)),
				secondScoreToNeutral.cmd());
	}

	@Override
	public Pose2d getInitialPose() {
		return startToLine.getInitialPose().get();
	}
}
