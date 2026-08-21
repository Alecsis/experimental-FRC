package frc.robot.shooting;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.logging.LogUtil;
import frc.lib.math.ChassisAccelerations;
import frc.lib.math.LinearLineCalculator;
import frc.lib.util.FieldLayout;
import frc.lib.util.FieldLayout.FieldArea;
import frc.robot.RobotConstants;
import frc.robot.shooting.driveplanner.DynamicShootingPlanner;
import frc.robot.shooting.driveplanner.DynamicShootingPlannerConstants.FerryNeutralDynamicShootingConstants;
import frc.robot.shooting.driveplanner.DynamicShootingPlannerConstants.HubDynamicShootingConstants;
import frc.robot.shooting.regressions.DistanceToAirTimeRegressions;
import frc.robot.shooting.regressions.ShooterVelocityRegressions.FerryRegressionConstants;
import frc.robot.shooting.regressions.ShooterVelocityRegressions.HubRegressionConstants;
import frc.robot.shooting.regressions.ShootingRegressions;
import frc.robot.shooting.setpoints.ShootingSetpoints;
import frc.robot.shooting.shootingplanner.MotionCompensatedShootingInputs;
import frc.robot.shooting.shootingplanner.MotionCompensatedShootingPlanner;
import frc.robot.shooting.targeting.Targeting;
import frc.robot.shooting.trends.TrendLines;
import frc.robot.shooting.verification.ShotVerifier;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.feederrollers.FeederRollers;

public class Shooting {
	public static ShootingState state = ShootingState.HUB;

	public static final Shooting mInstance = new Shooting();

	public static final DynamicShootingPlanner dynamicShooterPlanner = new DynamicShootingPlanner();
	public static final ShootingSetpoints shootingSetpoints = new ShootingSetpoints();
	public static final Targeting targeting = new Targeting();

	public static boolean m_rejectingLookAhead = false;

	public static final MotionCompensatedShootingPlanner m_lookAheadTracker =
			new MotionCompensatedShootingPlanner(DriveConstants.SHOOTER_OFFSET);

	public Shooting() {}

	public void periodic() {
		if (DriverStation.isEnabled()) {
			targeting.periodic();
			LogUtil.log(
					"Target Shooting State Translation",
					new Pose2d(state.getLocationTranslation(RobotConstants.isRedAlliance), new Rotation2d()));
			SmartDashboard.putString("Target Shooting State", state.name());
		}
		SmartDashboard.putNumber(
				"Shooting/Distance from target Meters",
				state.getLocationTranslation(RobotConstants.isRedAlliance)
						.getDistance(Drive.mInstance.getPose().getTranslation()));
		ChassisAccelerations linearAcceleration = Drive.mInstance.getFilteredAccelerations();
		ChassisSpeeds driveSpeeds = Drive.mInstance.getFilteredChasisSpeeds();
		Pose2d drivePose = Drive.mInstance.getPose();
		Time lookAheadTime = ShootingSetpoints.getLookAheadTime(state);
		m_lookAheadTracker.update(new MotionCompensatedShootingInputs(
				state, driveSpeeds, drivePose, linearAcceleration, lookAheadTime.in(Seconds)));
		SmartDashboard.putBoolean("Shot Verifier", shotVerified());
	}

	public enum ShootingState {
		HUB(
				FieldLayout.getAllianceHubTranslation(false).get(),
				HubDynamicShootingConstants.LOCK_CONTROLLER,
				HubRegressionConstants.REGRESSIONS,
				DistanceToAirTimeRegressions.distanceToTimeHub,
				Seconds.of(0.5).unaryMinus(),
				1.0 / 1.0,
				FieldLayout.kHubAltitude,
				TrendLines.HUB_TREND_LINE),
		LEFT_NEUTRAL_FERRY(
				FieldLayout.getAllianceFerryTranslation(false, true, FieldArea.NEUTRAL)
						.get(),
				FerryNeutralDynamicShootingConstants.LOCK_CONTROLLER,
				FerryRegressionConstants.NEUTRAL_REGRESSIONS,
				DistanceToAirTimeRegressions.distanceToTimeNeutralFerry,
				Seconds.of(0.5).unaryMinus(),
				1.0 / 1.0,
				FieldLayout.kGroundAltitude,
				TrendLines.FERRY_TREND_LINE),
		RIGHT_NEUTRAL_FERRY(
				FieldLayout.getAllianceFerryTranslation(false, false, FieldArea.NEUTRAL)
						.get(),
				FerryNeutralDynamicShootingConstants.LOCK_CONTROLLER,
				FerryRegressionConstants.NEUTRAL_REGRESSIONS,
				DistanceToAirTimeRegressions.distanceToTimeNeutralFerry,
				Seconds.of(0.5).unaryMinus(),
				1.0 / 1.0,
				FieldLayout.kGroundAltitude,
				TrendLines.FERRY_TREND_LINE),
		LEFT_ALLIANCE_FERRY(
				FieldLayout.getAllianceFerryTranslation(false, true, FieldArea.BLUE_DEEP)
						.get(),
				FerryNeutralDynamicShootingConstants.LOCK_CONTROLLER,
				FerryRegressionConstants.ALLIANCE_REGRESSIONS,
				DistanceToAirTimeRegressions.distanceToTimeAllianceFerry,
				Seconds.of(0.5).unaryMinus(),
				1.0 / 1.0,
				FieldLayout.kGroundAltitude,
				TrendLines.FERRY_TREND_LINE),
		RIGHT_ALLIANCE_FERRY(
				FieldLayout.getAllianceFerryTranslation(false, false, FieldArea.BLUE_DEEP)
						.get(),
				FerryNeutralDynamicShootingConstants.LOCK_CONTROLLER,
				FerryRegressionConstants.ALLIANCE_REGRESSIONS,
				DistanceToAirTimeRegressions.distanceToTimeAllianceFerry,
				Seconds.of(0.5).unaryMinus(),
				1.0 / 1.0,
				FieldLayout.kGroundAltitude,
				TrendLines.FERRY_TREND_LINE),
		LEFT_FAR_ALLIANCE_FERRY(
				FieldLayout.getAllianceFerryTranslation(false, true, FieldArea.BLUE_SHALLOW)
						.get(),
				FerryNeutralDynamicShootingConstants.LOCK_CONTROLLER,
				FerryRegressionConstants.FAR_ALLIANCE_REGRESSIONS,
				DistanceToAirTimeRegressions.distanceToTimeNeutralFerry,
				Seconds.of(0.5).unaryMinus(),
				1.0 / 1.0,
				FieldLayout.kGroundAltitude,
				TrendLines.FERRY_TREND_LINE),
		RIGHT_FAR_ALLIANCE_FERRY(
				FieldLayout.getAllianceFerryTranslation(false, false, FieldArea.BLUE_SHALLOW)
						.get(),
				FerryNeutralDynamicShootingConstants.LOCK_CONTROLLER,
				FerryRegressionConstants.FAR_ALLIANCE_REGRESSIONS,
				DistanceToAirTimeRegressions.distanceToTimeNeutralFerry,
				Seconds.of(0.5).unaryMinus(),
				1.0 / 1.0,
				FieldLayout.kGroundAltitude,
				TrendLines.FERRY_TREND_LINE);

		private final Translation2d blueTargetTranslation;
		private final ProfiledPIDController lockController;
		private final InterpolatingDoubleTreeMap distanceToFuelAirTimeMap;
		private final ShootingRegressions shootingRegressions;
		private final Time lookAheadTimeOffset;
		private final double constrainedTranslationTravelSpeedCoeffecint;
		private final Distance deltaY;
		private final LinearLineCalculator tofTrendLine;

		private ShootingState(
				Translation2d blueTargetTranslation,
				ProfiledPIDController lockController,
				ShootingRegressions shootingRegressions,
				InterpolatingDoubleTreeMap distanceToFuelAirTimeMap,
				Time lookAheadTimeOffset,
				double constrainedTranslationTravelSpeedCoeffecint,
				Distance deltaY,
				LinearLineCalculator tofTrendLine) {
			this.blueTargetTranslation = blueTargetTranslation;
			this.lockController = lockController;
			this.shootingRegressions = shootingRegressions;
			this.distanceToFuelAirTimeMap = distanceToFuelAirTimeMap;
			this.lookAheadTimeOffset = lookAheadTimeOffset;
			this.constrainedTranslationTravelSpeedCoeffecint = constrainedTranslationTravelSpeedCoeffecint;
			this.deltaY = deltaY;
			this.tofTrendLine = tofTrendLine;
		}

		public Translation2d getLocationTranslation(boolean isRedAlliance) {
			return FieldLayout.handleAllianceFlip(blueTargetTranslation, isRedAlliance);
		}

		public ShootingRegressions getShootingRegressions() {
			return shootingRegressions;
		}

		public InterpolatingDoubleTreeMap getDistanceToFuelAirTimeMap() {
			return distanceToFuelAirTimeMap;
		}

		public double getConstrainedTranslationTravelSpeedCoeffecint() {
			return constrainedTranslationTravelSpeedCoeffecint;
		}

		public ProfiledPIDController getLockController() {
			return lockController;
		}

		public Time getLookAheadTimeOffset() {
			return lookAheadTimeOffset;
		}

		public Distance getDeltaY() {
			return deltaY;
		}

		public Time tof(Distance distance) {
			return Seconds.of(tofTrendLine.calculate(distance.in(Meters)));
		}

		public boolean isFiring() {
			return FeederRollers.mInstance.getSetpoint() == FeederRollers.FEED_VOLTAGE;
		}
	}

	public static void updateState(ShootingState nextState) {
		state = nextState;
	}

	public static ShootingState getState() {
		return state;
	}

	public Command headingLockToStateTargetCommand() {
		return Drive.mInstance.followSwerveRequestCommand(
				DriveConstants.HEADING_LOCK_REQUEST, dynamicShooterPlanner.getTargetStateAimRequestUpdater());
	}

	public Setpoint getLatestHoodTargetSetpoint() {
		return shootingSetpoints.getHoodSetpoint();
	}

	public Setpoint getLatestShooterSetpoint() {
		return shootingSetpoints.getShooterSetpoint();
	}

	public Setpoint getLatestTunnelSetpoint() {
		return shootingSetpoints.getTunnelSetpoint();
	}

	public boolean shotVerified() {
		boolean shotVerified = ShotVerifier.verifyShot(state);
		SmartDashboard.putNumber("Shot Verified Last Updated Timestamp", Timer.getFPGATimestamp());
		SmartDashboard.putBoolean("Shot Verified", shotVerified);
		return shotVerified;
	}

	public boolean shotAligned() {
		return ShotVerifier.verifyShotAlignment(state);
	}

	public Command followShootOnTheMoveRequest() {
		return new WrapperCommand(Drive.mInstance.followSwerveRequestCommand(
				DriveConstants.aimingRequest, dynamicShooterPlanner.getTargetStateAimRequestUpdater())) {

			boolean firstLoop = false;

			@Override
			public void initialize() {
				getState()
						.lockController
						.reset(
								Drive.mInstance.getPose().getRotation().getRadians(),
								Drive.mInstance.getFilteredChasisSpeeds().omegaRadiansPerSecond);
				DriveConstants.driveInitRotation = Drive.mInstance.getPose().getRotation();
				ShotVerifier.reset();
				SmartDashboard.putBoolean("Align", false);
				firstLoop = true;
				super.execute();
			}

			@Override
			public void execute() {
				if (firstLoop) {
					SmartDashboard.putBoolean("Align", true);
					firstLoop = false;
				}
				super.execute();
			}

			@Override
			public void end(boolean i) {
				SmartDashboard.putBoolean("Align", false);
				firstLoop = false;
			}
		};
	}

	public boolean autoShotAlignedOrTimedOut(Time elapsedTime) {
		return ShotVerifier.verifyAutoShotAlignment(state, elapsedTime);
	}

	public DynamicShootingPlanner getDynamicShooterPlanner() {
		return dynamicShooterPlanner;
	}
}
