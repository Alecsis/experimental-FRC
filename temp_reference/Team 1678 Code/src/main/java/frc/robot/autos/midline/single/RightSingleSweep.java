package frc.robot.autos.midline.single;

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
import frc.lib.util.FieldLayout.ClimbLocation;
import frc.robot.RobotConstants;
import frc.robot.autos.AutoHelpers;
import frc.robot.autos.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.Set;

public class RightSingleSweep extends AutoModeBase {

	AutoTrajectory tearDrop = mirroredTrajectory("leftTearDrop");

	static SwerveRequest.FieldCentric velocityRequest = new SwerveRequest.FieldCentric()
			.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
			.withDriveRequestType(DriveRequestType.Velocity);

	public RightSingleSweep(AutoFactory factory) {
		super(factory, "Right Single Sweep");

		logTrajectories(tearDrop);

		prepRoutine(
				cmdWithCollisonAvoidance(tearDrop, velocityRequest),
				Commands.defer(
						() -> driveOverBump(
								velocityRequest, Rotation2d.fromDegrees(RobotConstants.isRedAlliance ? 38.37 : -130.0)),
						Set.of(Drive.mInstance)),
				AutoHelpers.fireCommand(Units.Seconds.of(6.0), Seconds.of(1.75)),
				Superstructure.mInstance.climb(ClimbLocation.A));
	}

	@Override
	public Pose2d getInitialPose() {
		return tearDrop.getInitialPose().get();
	}
}
