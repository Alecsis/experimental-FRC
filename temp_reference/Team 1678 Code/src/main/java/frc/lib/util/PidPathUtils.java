package frc.lib.util;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.lib.util.controller.SynchronousPIDF;

public class PidPathUtils {

	private static final double ROTATION_FEED_FORWARD = 0.01;

	public PidPathUtils() {}

	public static ChassisSpeeds calculateSpeeds(
			Pose2d currentPose,
			Pose2d targetPose,
			PIDController xController,
			PIDController yController,
			PIDController thetaController) {
		double rotationSpeed = thetaController.calculate(
				currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

		if (!MathUtil.isNear(
				targetPose.getRotation().getDegrees(), currentPose.getRotation().getDegrees(), 1.0, -180, 180)) {
			rotationSpeed += Math.copySign(Units.rotationsToRadians(ROTATION_FEED_FORWARD), rotationSpeed);
		}

		return new ChassisSpeeds(
				xController.calculate(currentPose.getX(), targetPose.getX()),
				yController.calculate(currentPose.getY(), targetPose.getY()),
				rotationSpeed);
	}

	public static ChassisSpeeds calculateCombined(
			Pose2d currentPose, Pose2d targetPose, PIDController translationController, PIDController thetaController) {
		/*
		 * We flip the sign here to produce the right output (negative error = pos output)
		 * Could alternatively do (target - current).unaryMinus() but this is just cleaner
		 */

		Translation2d translationError = currentPose.getTranslation().minus(targetPose.getTranslation());

		Translation2d uError = translationError.div(translationError.getNorm());
		Translation2d velocity = uError.times(translationController.calculate(translationError.getNorm()));

		return new ChassisSpeeds(
				MetersPerSecond.of(velocity.getX()),
				MetersPerSecond.of(velocity.getY()),
				RadiansPerSecond.of(thetaController.calculate(
						MathUtil.angleModulus(currentPose.getRotation().getRadians()),
						MathUtil.angleModulus(targetPose.getRotation().getRadians()))));
	}

	public static ChassisSpeeds calculateCombinedSync(
			Pose2d input, Pose2d output, SynchronousPIDF translationController, SynchronousPIDF thetaController) {
		/*
		 * We flip the sign here to produce the right output (negative error = pos output)
		 * Could alternatively do (target - current).unaryMinus() but this is just cleaner
		 */

		Translation2d translationError = input.getTranslation().minus(output.getTranslation());

		Translation2d uError = translationError.div(translationError.getNorm());
		Translation2d velocity = uError.times(translationController.calculate(translationError.getNorm()));

		thetaController.setSetpoint(MathUtil.angleModulus(output.getRotation().getRadians()));

		return new ChassisSpeeds(
				velocity.getX(),
				velocity.getY(),
				thetaController.calculate(
						MathUtil.angleModulus(input.getRotation().getRadians())));
	}

	public static ChassisSpeeds calculateSync(
			Pose2d drivePose,
			Pose2d desired,
			SynchronousPIDF xController,
			SynchronousPIDF yController,
			SynchronousPIDF headingController) {

		return new ChassisSpeeds(
				xController.setSetpointAndCalculate(desired.getX(), drivePose.getX()),
				yController.setSetpointAndCalculate(desired.getY(), drivePose.getY()),
				headingController.setSetpointAndCalculate(
						desired.getRotation().getRadians(),
						desired.getRotation().getRadians()));
	}
}
