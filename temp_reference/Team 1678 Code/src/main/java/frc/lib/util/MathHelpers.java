package frc.lib.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.axis3d.TranslationAxis3d;

public class MathHelpers {
	public static final Pose2d kPose2dZero = new Pose2d();
	public static final Angle PI = Radians.of(Math.PI);
	public static final Angle PI_DIV_2 = PI.div(2.0);

	public static final Pose2d pose2dFromRotation(Rotation2d rotation) {
		return new Pose2d(kTranslation2dZero, rotation);
	}

	public static final Pose2d pose2dFromTranslation(Translation2d translation) {
		return new Pose2d(translation, kRotation2dZero);
	}

	public static final Rotation2d kRotation2dZero = new Rotation2d();
	public static final Rotation2d kRotation2dPi = Rotation2d.fromDegrees(180.0);

	public static final Translation2d kTranslation2dZero = new Translation2d();

	public static final Transform2d kTransform2dZero = new Transform2d();

	public static final Transform2d transform2dFromRotation(Rotation2d rotation) {
		return new Transform2d(kTranslation2dZero, rotation);
	}

	public static final Transform2d transform2dFromTranslation(Translation2d translation) {
		return new Transform2d(translation, kRotation2dZero);
	}

	public static final Pose2d poseFromTransform(Transform2d transform) {
		return new Pose2d(transform.getX(), transform.getY(), transform.getRotation());
	}

	public static final Pose3d poseFromTransform(Transform3d transform) {
		return new Pose3d(transform.getX(), transform.getY(), transform.getZ(), transform.getRotation());
	}

	public static final Transform2d transformFromPose(Pose2d pose) {
		return new Transform2d(pose.getX(), pose.getY(), pose.getRotation());
	}

	public static final Transform3d transformFromPose(Pose3d pose) {
		return new Transform3d(pose.getTranslation(), pose.getRotation());
	}

	public Pose2d getLookAhead(ChassisSpeeds speeds, Pose2d currentPose, Time lookAheadTime) {
		return currentPose.transformBy(new Transform2d(
						speeds.vxMetersPerSecond,
						speeds.vyMetersPerSecond,
						Rotation2d.fromRadians(speeds.omegaRadiansPerSecond))
				.times(lookAheadTime.in(Seconds)));
	}

	public static int clamp(int input, int high, int low) {
		return Math.max(Math.min(input, high), low);
	}

	public static double clamp(double input, double high, double low) {
		return Math.max(Math.min(input, high), low);
	}

	public static final Angle abs(Angle angle) {
		return Radians.of(Math.abs(angle.in(Radians)));
	}

	public static final Rotation2d abs(Rotation2d rotation) {
		return new Rotation2d(abs(rotation.getMeasure()));
	}

	public static final AngularVelocity abs(AngularVelocity velocity) {
		return Rotations.per(Second).of(Math.abs(velocity.in(Rotations.per(Second))));
	}

	public static final double sin(Angle theta) {
		return Math.sin(theta.in(Radians));
	}

	public static final double cos(Angle theta) {
		return Math.cos(theta.in(Radians));
	}

	public static final double tan(Angle theta) {
		return Math.tan(theta.in(Radians));
	}

	public static final Distance hypot(Translation2d translation) {
		return Meters.of(Math.hypot(
				translation.getMeasureX().in(Meters), translation.getMeasureY().in(Meters)));
	}

	public static final LinearVelocity hypot(ChassisSpeeds speed) {
		return MetersPerSecond.of(Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond));
	}

	public static final LinearVelocity hypot(LinearVelocity x, LinearVelocity y) {
		return MetersPerSecond.of(Math.hypot(x.in(MetersPerSecond), y.in(MetersPerSecond)));
	}

	public static final Angle angleModulus(Angle angle) {
		return Radians.of(MathUtil.inputModulus(angle.in(Radians), -Math.PI, Math.PI));
	}

	public static final Rotation2d angleModulus(Rotation2d rotation) {
		return new Rotation2d(angleModulus(rotation.getMeasure()));
	}

	public static final Pose3d unaryMinus(Pose3d pose) {
		return pose.times(-1.0);
	}

	public static Translation3d addToTranslation3d(Translation3d translation, TranslationAxis3d axis, Distance offset) {
		return switch (axis) {
			case X -> translation.plus(new Translation3d(offset.in(Meters), 0.0, 0.0));
			case Y -> translation.plus(new Translation3d(0.0, offset.in(Meters), 0.0));
			case Z -> translation.plus(new Translation3d(0.0, 0.0, offset.in(Meters)));
		};
	}

	public static Transform3d addToTranslation3d(Transform3d transform, TranslationAxis3d axis, Distance offset) {
		return new Transform3d(addToTranslation3d(transform.getTranslation(), axis, offset), transform.getRotation());
	}

	public static Transform3d transform3d(Pose2d pose) {
		return new Transform3d(pose.getX(), pose.getY(), 0.0, new Rotation3d(pose.getRotation()));
	}

	public static Translation2d get2dCordsFromCircleAngle(Distance radius, Rotation2d rotation) {
		return new Translation2d(radius.times(rotation.getCos()), radius.times(rotation.getSin()));
	}

	public static Translation2d get2dCordsFromCircleAngle(Distance radius, Angle angle) {
		return get2dCordsFromCircleAngle(radius, new Rotation2d(angle));
	}

	public static double calculateTriangleAngleFromSides(double opposingFace, double face1, double face2) {
		double a = face1;
		double b = face2;
		double c = opposingFace;
		return Math.acos(((a * a) + (b * b) - (c * c)) / (2.0 * a * b));
	}

	public static Dimensionless clampToZero(Dimensionless value) {
		return value.gte(Percent.zero()) ? value : Percent.zero();
	}
}
