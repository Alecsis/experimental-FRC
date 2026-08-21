package frc.robot.shooting.driveplanner;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.logging.LogUtil;
import frc.robot.RobotConstants;
import frc.robot.shooting.Shooting;
import frc.robot.shooting.Shooting.ShootingState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.function.UnaryOperator;

public class DynamicShootingPlanner {

	public UnaryOperator<SwerveRequest.FieldCentric> getAimRequestUpdater(
			ProfiledPIDController lockController,
			Translation2d target,
			InterpolatingDoubleTreeMap regressionMap,
			Time lookAheadTimeOffset,
			double constrainedSpeed) {
		return (request) -> {
			DriveConstants.teleopRequestUpdater.apply(request);
			Translation2d stickVelocity = new Translation2d(request.VelocityX, request.VelocityY);

			Pose2d drivePose = Drive.mInstance.getPose();

			Time lookAheadTime = Seconds.of(
							regressionMap.get(drivePose.getTranslation().getDistance(target)))
					.plus(lookAheadTimeOffset);

			// Angle desiredAngle =
			// MathHelpers.angleModulus(target.minus(Shooting.m_lookAheadTracker.getLookAhead().getTranslation())
			// 		.getAngle().plus(Rotation2d.k180deg)
			// 		.getMeasure());
			Angle desiredAngle = Shooting.m_lookAheadTracker.getTargetAngle();
			LogUtil.log("Desired Angle Target", target);

			SmartDashboard.putNumber("Desired Angle", desiredAngle.in(Radians));

			return DriveConstants.getHeadingLockRequestUpdaterWithProfile(
							lockController, stickVelocity, desiredAngle, constrainedSpeed)
					.apply(request);
		};
	}

	public UnaryOperator<SwerveRequest.FieldCentric> getTargetStateAimRequestUpdater() {
		return (request) -> {
			ShootingState state = Shooting.getState();
			Translation2d target = state.getLocationTranslation(RobotConstants.isRedAlliance);
			ProfiledPIDController lockController = state.getLockController();
			InterpolatingDoubleTreeMap driveDistanceToFuelAirTimeMap = state.getDistanceToFuelAirTimeMap();
			Time lookAheadTimeOffset = state.getLookAheadTimeOffset();
			double constrainedTranslationTravelSpeedCoeffecint = state.getConstrainedTranslationTravelSpeedCoeffecint();

			return getAimRequestUpdater(
							lockController,
							target,
							driveDistanceToFuelAirTimeMap,
							lookAheadTimeOffset,
							constrainedTranslationTravelSpeedCoeffecint)
					.apply(request);
		};
	}
}
