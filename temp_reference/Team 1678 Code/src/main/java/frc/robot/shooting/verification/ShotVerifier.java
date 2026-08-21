package frc.robot.shooting.verification;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout;
import frc.lib.util.Util;
import frc.robot.RobotConstants;
import frc.robot.shooting.Shooting;
import frc.robot.shooting.Shooting.ShootingState;
import frc.robot.subsystems.drive.Drive;
import java.awt.geom.Line2D;

public class ShotVerifier {

	public static boolean isAligned = false;

	public static boolean m_prevAligned = false;

	public static void reset() {
		m_prevAligned = false;
	}

	public static boolean verifyShot(ShootingState shootingState) {
		boolean tilted = isTilted();
		boolean aligned = isAligned(shootingState);

		SmartDashboard.putBoolean("Not Is Tilted", !tilted);
		SmartDashboard.putBoolean("Shot Aligned for State", aligned);

		return !tilted && aligned;
	}

	public static boolean verifyShotAlignment(ShootingState shootingState) {
		return isAligned(shootingState);
	}

	public static boolean verifyAutoShotAlignment(ShootingState shootingState, Time elapsedTime) {
		boolean aligned = isAligned(shootingState);
		boolean timedOut = elapsedTime.gte(ShotVerifierConstants.kAutoAlignmentMaxWaitTime);
		boolean useTimeout = DriverStation.isAutonomousEnabled();

		SmartDashboard.putNumber("Auto Shot Alignment Elapsed Seconds", elapsedTime.in(Seconds));
		SmartDashboard.putBoolean("Auto Shot Alignment Timeout", timedOut);

		return aligned || (useTimeout && timedOut);
	}

	private static boolean isAligned(ShootingState shootingState) {

		SmartDashboard.putBoolean("Prev Shot Align", m_prevAligned);
		Angle offsetAngle = Shooting.m_lookAheadTracker.getTargetAngle();
		Angle driveAngle = Drive.mInstance.getPose().getRotation().getMeasure();

		double angleTolerance = Shooting.getState() == ShootingState.HUB
				? ShotVerifierConstants.kAcceptableHubAngleError.in(Degrees)
				: ShotVerifierConstants.kAcceptableFerryAngleError.in(Degrees);
		if (offsetAngle.minus(driveAngle).abs(Degrees) <= angleTolerance) {
			m_prevAligned = false;
			isAligned = true;
			return true;
		} else {
			// Rotation2d driveLookAheadRotation =
			// Drive.mInstance.getLookAheadHeading(Seconds.of(ShotVerifierConstants.LOOKAHEAD_TIME_TUNNABLE.getAsDouble()));
			Rotation2d driveLookAheadRotation = Drive.mInstance.getPose().getRotation();

			LogUtil.log(
					"SHOOTING LOOKAHEAD",
					new Pose2d(Drive.mInstance.getPose().getTranslation(), driveLookAheadRotation));

			Angle error = offsetAngle.minus(driveLookAheadRotation.getMeasure());
			isAligned = error.abs(Degrees) <= ShotVerifierConstants.kAcceptableHubAngleError.in(Degrees);

			if (!m_prevAligned && isAligned) {
				m_prevAligned = true;
			}
			return m_prevAligned;
		}
	}

	public static boolean isTilted() {

		var pigeon = Drive.mInstance.getGeneratedDrive().getPigeon2();
		SmartDashboard.putBoolean("Shot Verifier Gyro Connected", pigeon.isConnected());

		if (!pigeon.isConnected()) {
			SmartDashboard.putBoolean("isTilted", false);
			return false;
		}

		Angle pitch = pigeon.getPitch().getValue();
		Angle roll = pigeon.getRoll().getValue();

		double pitchDegrees = pitch.in(Units.Degrees);
		double rollDegrees = roll.in(Units.Degrees);

		if (!Double.isFinite(pitchDegrees) || !Double.isFinite(rollDegrees)) {
			SmartDashboard.putBoolean("isTilted", false);
			return false;
		}

		boolean tilted = !(Util.epsilonEquals(pitch, Units.Degrees.zero(), ShotVerifierConstants.kAcceptableTiltError)
				&& Util.epsilonEquals(roll, Units.Degrees.zero(), ShotVerifierConstants.kAcceptableTiltError));

		SmartDashboard.putBoolean("isTilted", tilted);
		return tilted;
	}

	public static boolean isInRange(ShootingState shootingState) {
		Distance distance = Meters.of(getLookAheadTranslation(shootingState)
				.getDistance(shootingState.getLocationTranslation(RobotConstants.isRedAlliance)));
		SmartDashboard.putNumber("Distance", distance.in(Meters));
		SmartDashboard.putNumber("Max distance", getMaxDistance(shootingState).in(Meters));

		return distance.lte(getMaxDistance(shootingState));
	}

	public static Distance getMaxDistance(ShootingState shootingState) {
		return Meters.of(getBallVelocity(shootingState)
				.times(Math.cos(getHoodAngle(shootingState).in(Radians)))
				.in(MetersPerSecond));
	}

	private static Angle getHoodAngle(ShootingState shootingState) {
		return shootingState.getShootingRegressions().getHoodAngle(getDistanceWithLookAheadOffset(shootingState));
	}

	private static LinearVelocity getBallVelocity(ShootingState shootingState) {
		double distInMeters = getDistanceWithLookAheadOffset(shootingState).in(Meters);
		return Units.MetersPerSecond.of(
				distInMeters / shootingState.getDistanceToFuelAirTimeMap().get(distInMeters));
	}

	private static Distance getDistanceWithLookAheadOffset(ShootingState shootingState) {
		return Units.Meters.of(shootingState
				.getLocationTranslation(RobotConstants.isRedAlliance)
				.getDistance(getLookAheadTranslation(shootingState)));
	}

	private static Translation2d getLookAheadTranslation(ShootingState shootingState) {
		Time lookAheadTime = Units.Seconds.of(shootingState
						.getDistanceToFuelAirTimeMap()
						.get(Drive.mInstance
								.getPose()
								.getTranslation()
								.getDistance(shootingState.getLocationTranslation(RobotConstants.isRedAlliance))))
				.plus(shootingState.getLookAheadTimeOffset());

		return Drive.mInstance.getLookAheadPoseWithoutHeading(lookAheadTime).getTranslation();
	}

	private static boolean isShotBlocked(ShootingState shootingState) {
		if (FieldLayout.isPoseWithinAllianceZone(RobotConstants.isRedAlliance, Drive.mInstance.getPose())) return false;

		Translation2d targetTranslation = shootingState.getLocationTranslation(RobotConstants.isRedAlliance);
		Translation2d robotTranslation2d = Drive.mInstance.getPose().getTranslation();

		if (doIntersect(
				targetTranslation,
				robotTranslation2d,
				ShotVerifierConstants.blueFrontLeftHubCorner,
				ShotVerifierConstants.blueFrontRightHubCorner)) return true;

		if (doIntersect(
				targetTranslation,
				robotTranslation2d,
				ShotVerifierConstants.blueFrontRightHubCorner,
				ShotVerifierConstants.blueBackRightHubCorner)) return true;

		if (doIntersect(
				targetTranslation,
				robotTranslation2d,
				ShotVerifierConstants.blueBackRightHubCorner,
				ShotVerifierConstants.blueBackLeftHubCorner)) return true;

		if (doIntersect(
				targetTranslation,
				robotTranslation2d,
				ShotVerifierConstants.blueBackLeftHubCorner,
				ShotVerifierConstants.blueFrontLeftHubCorner)) return true;

		Translation2d flippedBlueFrontLeftHubCorner =
				FieldLayout.flipAboutMidline(ShotVerifierConstants.blueFrontLeftHubCorner);
		Translation2d flippedBlueFrontRightHubCorner =
				FieldLayout.flipAboutMidline(ShotVerifierConstants.blueFrontRightHubCorner);
		Translation2d flippedBlueBackLeftHubCorner =
				FieldLayout.flipAboutMidline(ShotVerifierConstants.blueBackLeftHubCorner);
		Translation2d flippedBlueBackRightHubCorner =
				FieldLayout.flipAboutMidline(ShotVerifierConstants.blueBackRightHubCorner);

		if (doIntersect(
				targetTranslation, robotTranslation2d, flippedBlueFrontLeftHubCorner, flippedBlueFrontRightHubCorner))
			return true;

		if (doIntersect(
				targetTranslation, robotTranslation2d, flippedBlueFrontRightHubCorner, flippedBlueBackRightHubCorner))
			return true;

		if (doIntersect(
				targetTranslation, robotTranslation2d, flippedBlueBackRightHubCorner, flippedBlueBackLeftHubCorner))
			return true;

		if (doIntersect(
				targetTranslation, robotTranslation2d, flippedBlueBackLeftHubCorner, flippedBlueFrontLeftHubCorner))
			return true;

		return false;
	}

	private static boolean doIntersect(
			Translation2d targetTranslation,
			Translation2d robotPose,
			Translation2d testPoseOne,
			Translation2d testPoseTwo) {
		return Line2D.linesIntersect(
				targetTranslation.getMeasureX().baseUnitMagnitude(),
				targetTranslation.getMeasureY().baseUnitMagnitude(),
				robotPose.getMeasureX().baseUnitMagnitude(),
				robotPose.getMeasureY().baseUnitMagnitude(),
				testPoseOne.getMeasureX().baseUnitMagnitude(),
				testPoseOne.getMeasureY().baseUnitMagnitude(),
				testPoseTwo.getMeasureX().baseUnitMagnitude(),
				testPoseTwo.getMeasureY().baseUnitMagnitude());
	}
}
