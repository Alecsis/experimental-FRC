package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.logging.LogUtil;
import frc.lib.util.MathHelpers;
import frc.lib.util.Util;
import frc.lib.util.controller.SynchronousPIDF;
import frc.lib.util.tunablenumbers.TunableNumber;
import frc.robot.RobotConstants;
import frc.robot.controlboard.ControlBoardConstants;
import java.util.function.UnaryOperator;

public class DriveConstants {
	public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
	public static final LinearVelocity kMaxSpeedFAST = kMaxSpeed.times(2.0);

	public static final SynchronousPIDF mAutoAlignHeadingController = getAutoAlignHeadingController();
	public static final PIDController mAutoAlignTranslationController = getAutoAlignTranslationController();

	public static final LinearAcceleration kMaxAcceleration = Units.MetersPerSecondPerSecond.of(12.0);
	public static final LinearAcceleration kMaxSpeedAccelerationFast = MetersPerSecondPerSecond.of(16.0);
	public static final AngularVelocity kMaxAngularRate =
			Units.RadiansPerSecond.of(RobotConstants.isEpsilon ? 11.35 : 11.015797);
	public static final AngularVelocity kMaxAngularRateFAST = kMaxAngularRate.times(2.0);
	public static final AngularAcceleration kMaxAngularAcceleration =
			RadiansPerSecondPerSecond.of(RobotConstants.isGamma ? 40.0 : 50.0);

	public static final PIDController SWERVE_HEADING_CONTROLELR = new PIDController(5.0, 0.0, 0.0);

	public static final Time LOOK_AHEAD_TIME = Milliseconds.of(200.0);
	public static final Distance CHOREO_TRANSLATION_EPSILON = Units.Inches.of(0.5);

	public static final PIDController mTranslationController = new PIDController(3.7, 0, 0);
	public static final PIDController mRotationController = new PIDController(5.0, 0, 0.2);

	public static final SynchronousPIDF AUTO_SYNC_TRANSLATION_CONTROLLER = new SynchronousPIDF(3.9, 0, 0, 0.0);
	public static final SynchronousPIDF AUTO_SYNC_HEADING_CONTROLLER = new SynchronousPIDF(5.0, 0, 0.1, 0.00);

	public static final TunableNumber m_minFeedFowardActivation = new TunableNumber("FF Activation Degrees", 0.0);
	public static final TunableNumber m_ffLow = new TunableNumber("Feed Foward Low", 0.0);

	public static final LinearVelocity CHOREO_MAX_TRANSLATION_FEEDBACK = kMaxSpeed;
	public static final AngularVelocity CHOREO_MAX_HEADING_FEEDBACK = kMaxAngularRate;
	public static final SynchronousPIDF CHOREO_TRANSLATION_CONTROLLER = new SynchronousPIDF(6.0, 0.0, 0.0);
	public static final SynchronousPIDF CHOREO_THETA_CONTROLLER = new SynchronousPIDF(4.0, 0.0, 0.0);

	public static final TunableNumber FEED_FOWARD_CONSTANT =
			new TunableNumber("Feed Foward", RobotConstants.isEpsilon ? 1.0 : 0.8);

	public static final TunableNumber FEED_SCALE_FOWARD_PERCENT =
			new TunableNumber("FF/Scale Percent", RobotConstants.isEpsilon ? 100.0 : 100.0);

	public static final Translation2d PIGEON_OFFSET = new Translation2d(Inches.of(-0.857), Inches.of(8.151));

	public static final Transform2d SHOOTER_OFFSET =
			new Transform2d(Inches.of(11.28).unaryMinus(), BaseUnits.DistanceUnit.zero(), Rotation2d.kPi);

	public static final AngularVelocity YAW_UNSTABLE_VELOCITY = DegreesPerSecond.of(300.0);

	public static final Time PHASE_DELAY = Milliseconds.of(20.0);

	static {
		SWERVE_HEADING_CONTROLELR.enableContinuousInput(-Math.PI, Math.PI);
		SWERVE_HEADING_CONTROLELR.setIZone(Degrees.of(5.0).in(Radians));
	}

	static {
		AUTO_SYNC_TRANSLATION_CONTROLLER.setContinuous(false);
		AUTO_SYNC_TRANSLATION_CONTROLLER.setMaxAbsoluteOutput(Double.MAX_VALUE); // let the utils constrain

		AUTO_SYNC_HEADING_CONTROLLER.setContinuous(true);
		AUTO_SYNC_HEADING_CONTROLLER.setInputRange(-Math.PI, Math.PI);
		AUTO_SYNC_HEADING_CONTROLLER.setTolerance(Degrees.of(3.0).in(Radians));
		AUTO_SYNC_HEADING_CONTROLLER.setMaxAbsoluteOutput(Double.MAX_VALUE); // let the utils constrain

		CHOREO_TRANSLATION_CONTROLLER.setMaxAbsoluteOutput(CHOREO_MAX_TRANSLATION_FEEDBACK.in(MetersPerSecond));
		CHOREO_THETA_CONTROLLER.setMaxAbsoluteOutput(CHOREO_MAX_HEADING_FEEDBACK.in(RadiansPerSecond));

		CHOREO_THETA_CONTROLLER.setInputRange(-Math.PI, Math.PI);
		CHOREO_THETA_CONTROLLER.setContinuous();
	}

	public static final SynchronousPIDF HEADING_LOCK_CONTROLLER = new SynchronousPIDF(3.7, 0.0, 0.0);

	public static final SynchronousPIDF HUB_HEADING_LOCK_CONTROLLER = new SynchronousPIDF(9.0, 0.0, 0.5, 0.0);

	public static final DriveStableConfig HUB_STABLE_CONFIG =
			new DriveStableConfig(MetersPerSecond.of(0.5), DegreesPerSecond.of(3.0), Milliseconds.of(0.0));

	static {
		mRotationController.enableContinuousInput(-Math.PI, Math.PI);
		mRotationController.setTolerance(0.01);

		HEADING_LOCK_CONTROLLER.setContinuous(true);
		HEADING_LOCK_CONTROLLER.setInputRange(-Math.PI, Math.PI);
		HEADING_LOCK_CONTROLLER.setMaxAbsoluteOutput(kMaxAngularRate.in(RadiansPerSecond));
		HUB_HEADING_LOCK_CONTROLLER.setContinuous(true);
		HUB_HEADING_LOCK_CONTROLLER.setInputRange(-Math.PI, Math.PI);
		HUB_HEADING_LOCK_CONTROLLER.setMaxAbsoluteOutput(kMaxAngularRate.in(RadiansPerSecond));
		HUB_HEADING_LOCK_CONTROLLER.setTolerance(Degrees.of(3.0).in(Radians));
	}

	public static final SwerveRequest.FieldCentric EMPTY_SWERVE_REQUEST = new SwerveRequest.FieldCentric();

	public static final SwerveRequest.FieldCentric PIDToPoseRequest = new SwerveRequest.FieldCentric()
			.withDeadband(DriveConstants.kMaxSpeed
					.times(ControlBoardConstants.stickDeadband)
					.div(10.0))
			.withRotationalDeadband(DriveConstants.kMaxAngularRate
					.times(ControlBoardConstants.stickDeadband)
					.div(10.0))
			.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
			.withDesaturateWheelSpeeds(true)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	public static final SwerveRequest.FieldCentric HEADING_LOCK_REQUEST =
			new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	public static final SwerveRequest.FieldCentric teleopRequest =
			new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	public static final SwerveRequest.FieldCentric ANGULAR_SPEEDS_TEST_REQUEST =
			new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

	public static final UnaryOperator<SwerveRequest.FieldCentric> teleopRequestUpdater =
			(SwerveRequest.FieldCentric request) -> {
				double xDesiredRaw = -ControlBoardConstants.mDriverController.getLeftY();
				double yDesiredRaw = -ControlBoardConstants.mDriverController.getLeftX();
				double rotDesiredRaw = -ControlBoardConstants.mDriverController.getRightX();
				double xFancy = getDeadbandedStick(xDesiredRaw);
				double yFancy = getDeadbandedStick(yDesiredRaw);
				double rotFancy = getDeadbandedStick(rotDesiredRaw);

				// SmartDashboard.putNumber("Sticks/x/raw", xDesiredRaw);
				// SmartDashboard.putNumber("Sticks/x/fancy", xFancy);
				// SmartDashboard.putNumber("Sticks/y/raw", yDesiredRaw);
				// SmartDashboard.putNumber("Sticks/y/fancy", yFancy);
				// SmartDashboard.putNumber("Sticks/rot/raw", rotDesiredRaw);
				// SmartDashboard.putNumber("Sticks/rot/fancy", rotFancy);

				AngularVelocity targetOmega = DriveConstants.kMaxAngularRate.times(rotFancy);

				LinearVelocity targetX = DriveConstants.kMaxSpeed.times(xFancy);
				LinearVelocity targetY = DriveConstants.kMaxSpeed.times(yFancy);
				LinearVelocity targetMagnitude = MathHelpers.hypot(targetX, targetY);

				SmartDashboard.putNumber("Sticks/Target/Omega Radians Per Second", targetOmega.in(RadiansPerSecond));
				SmartDashboard.putNumber("Sticks/Target/X Meters Per Second", targetX.in(MetersPerSecond));
				SmartDashboard.putNumber("Sticks/Target/Y Meters Per Second", targetY.in(MetersPerSecond));
				SmartDashboard.putNumber("Sticks/Target/Translation Magnitude", targetMagnitude.in(MetersPerSecond));

				SmartDashboard.putNumber("Sticks/hypot/raw", Math.hypot(xDesiredRaw, yDesiredRaw));
				// SmartDashboard.putNumber("Sticks/hypot/fancy", Math.hypot(xFancy, yFancy));

				return request.withVelocityX(targetX).withVelocityY(targetY).withRotationalRate(targetOmega);
			};

	public static final SwerveRequest.FieldCentric aimingRequest =
			new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

	public static final SwerveRequest.FieldCentric hitTower =
			new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

	public static Rotation2d driveInitRotation = new Rotation2d();

	public static UnaryOperator<SwerveRequest.FieldCentric> getHeadingLockRequestUpdaterWithProfile(
			ProfiledPIDController lockController,
			Translation2d stickTranslation,
			Angle targetAngle,
			double constrainedSpeedCoef) {
		return (request) -> {
			double calculatedOutput = lockController.calculate(
					MathUtil.angleModulus(
							Drive.mInstance.getPose().getRotation().getRadians()),
					MathUtil.angleModulus(targetAngle.in(Radians)));

			SmartDashboard.putNumber("Heading Setpoint Position", lockController.getSetpoint().position);
			SmartDashboard.putNumber("Heading Setpoint Velocity", lockController.getSetpoint().velocity);

			TrapezoidProfile.State setpoint = lockController.getSetpoint();
			TrapezoidProfile.State goal = lockController.getGoal();

			double ff;

			double progress = Math.abs(MathUtil.angleModulus(setpoint.position - driveInitRotation.getRadians())
					/ MathUtil.angleModulus(goal.position - driveInitRotation.getRadians()));

			double scalar = Util.epsilonEquals(FEED_SCALE_FOWARD_PERCENT.getAsDouble(), 0.0)
					? 1.0
					: Math.max(((-1.0 / (FEED_SCALE_FOWARD_PERCENT.getAsDouble() / 100.0)) * progress + 1.0), 0.0);

			ff = FEED_FOWARD_CONSTANT.getAsDouble() * setpoint.velocity * scalar;

			double output = calculatedOutput + ff;
			SmartDashboard.putNumber("Feed Foward Radians Per Second", ff);
			SmartDashboard.putNumber("Raw Output Radians Per Second", output);
			SmartDashboard.putNumber("Feed Foward Scalar Radians Per Second", scalar);
			SmartDashboard.putNumber("Feed Foward progess", progress);

			// SmartDashboard.putNumber("FF", ff);

			SmartDashboard.putNumber("Calc output Radians Per Second", output);
			SmartDashboard.putNumber("Calculated Setpoint Radians", targetAngle.in(Radians));

			return request.withVelocityX(stickTranslation.getX() * constrainedSpeedCoef)
					.withVelocityY(stickTranslation.getY() * constrainedSpeedCoef)
					.withRotationalRate(output)
					.withDriveRequestType(DriveRequestType.Velocity)
					.withSteerRequestType(SteerRequestType.MotionMagicExpo)
					.withRotationalDeadband(kMaxAngularRate.times(0.01));
		};
	}

	public static double getDeadbandedStick(double rawValue) {
		if (Math.abs(rawValue) < ControlBoardConstants.stickDeadband) {
			return 0.0;
		} else {
			double unsignedValue = (Math.abs(rawValue) - ControlBoardConstants.stickDeadband)
					/ (1.0 - ControlBoardConstants.stickDeadband);
			return (rawValue > 0 ? unsignedValue : -unsignedValue);
		}
	}

	public static final UnaryOperator<SwerveRequest.FieldCentric> getHeadingLockUpdater(Angle lock) {
		return (request) -> {
			double xDesiredRaw = -ControlBoardConstants.mDriverController.getLeftY();
			double yDesiredRaw = -ControlBoardConstants.mDriverController.getLeftX();
			double xFancy = getDeadbandedStick(xDesiredRaw);
			double yFancy = getDeadbandedStick(yDesiredRaw);

			Pose2d drivePose = Drive.mInstance.getPose();
			SmartDashboard.putNumber("Desired Lock Rads", lock.in(Radians));

			HEADING_LOCK_CONTROLLER.setSetpoint(MathUtil.angleModulus(lock.in(Radians)));

			double angularVelocityRadsPerSec = HEADING_LOCK_CONTROLLER.calculate(
					MathUtil.angleModulus(drivePose.getRotation().getRadians()));

			return request.withVelocityX(kMaxSpeed.times(xFancy))
					.withVelocityY(kMaxSpeed.times(yFancy))
					.withRotationalRate(angularVelocityRadsPerSec);
		};
	}

	public static final Angle maxPitchStableThreshold = Units.Degree.of(5.0); // TODO: Get actual values
	public static final AngularVelocity maxYawStableThreshold =
			Units.DegreesPerSecond.of(100.0); // TODO: Get actual values

	public static final SwerveRequest.FieldCentric autoPIDRequest = new SwerveRequest.FieldCentric()
					.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
					.withDesaturateWheelSpeeds(true)
					.withDriveRequestType(DriveRequestType.Velocity)
					.withSteerRequestType(SteerRequestType.Position)
			// .withDeadband(DriveConstants.kMaxSpeed
			// 		.times(ControlBoardConstants.stickDeadband)
			// 		.div(10.0))
			// .withRotationalDeadband(DriveConstants.kMaxAngularRate
			// 		.times(ControlBoardConstants.stickDeadband)
			// 		.div(10.0))
			// .withDeadband(MetersPerSecond.of(0.07))
			// .withRotationalDeadband(RadiansPerSecond.of(0.05))
			;

	public static final SwerveRequest.FieldCentric choreoRequest = new SwerveRequest.FieldCentric()
			.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
			.withDesaturateWheelSpeeds(true)
			.withDriveRequestType(DriveRequestType.Velocity)
			.withSteerRequestType(SteerRequestType.Position);

	public static final UnaryOperator<SwerveRequest.FieldCentric> getPIDToPoseRequestUpdater(Pose2d targetPose) {
		return getPIDToPoseRequestUpdater(targetPose, mAutoAlignTranslationController, mAutoAlignHeadingController);
	}

	public static final UnaryOperator<SwerveRequest.FieldCentric> getChoreoPathRequestUpdater(SwerveSample sample) {
		return getChoreoPathRequestUpdater(sample, CHOREO_TRANSLATION_CONTROLLER, CHOREO_THETA_CONTROLLER);
	}

	public static final UnaryOperator<SwerveRequest.FieldCentric> getChoreoPathRequestUpdater(
			SwerveSample sample, SynchronousPIDF translationController, SynchronousPIDF headingController) {
		return (SwerveRequest.FieldCentric request) -> {
			Pose2d currentPose = Drive.mInstance.getPose();
			Pose2d desiredPose = sample.getPose();
			Translation2d translationError = currentPose.getTranslation().minus(desiredPose.getTranslation());
			double translationErrorNorm = translationError.getNorm();
			// Scale up unit vector by PID output
			Translation2d pidTranslationOutput = Translation2d.kZero;
			double translationPidMagnitudeMetersPerSecond = 0.0;
			if (translationErrorNorm > CHOREO_TRANSLATION_EPSILON.in(Units.Meters)) {
				translationPidMagnitudeMetersPerSecond = translationController.calculate(translationErrorNorm);
				pidTranslationOutput =
						translationError.div(translationErrorNorm).times(translationPidMagnitudeMetersPerSecond);
			}

			double commandedVxMetersPerSecond = sample.vx + pidTranslationOutput.getX();
			double commandedVyMetersPerSecond = sample.vy + pidTranslationOutput.getY();
			double currentHeadingRad =
					MathUtil.angleModulus(currentPose.getRotation().getRadians());
			double desiredHeadingRad =
					MathUtil.angleModulus(desiredPose.getRotation().getRadians());
			double headingErrorRad = MathUtil.angleModulus(desiredHeadingRad - currentHeadingRad);
			headingController.setSetpoint(desiredHeadingRad);
			double rawHeadingPidOutputRadPerSecond = headingController.calculate(currentHeadingRad);
			double headingPidOutputRadPerSecond = MathUtil.clamp(
					rawHeadingPidOutputRadPerSecond,
					-CHOREO_MAX_HEADING_FEEDBACK.in(RadiansPerSecond),
					CHOREO_MAX_HEADING_FEEDBACK.in(RadiansPerSecond));
			double commandedOmegaRadPerSecond = headingPidOutputRadPerSecond + sample.omega;

			LogUtil.log("Choreo/Controller/DesiredPose", desiredPose);

			logChoreoControllerTelemetry(
					sample,
					currentPose,
					desiredPose,
					translationError,
					translationErrorNorm,
					translationPidMagnitudeMetersPerSecond,
					pidTranslationOutput,
					headingErrorRad,
					headingPidOutputRadPerSecond,
					commandedVxMetersPerSecond,
					commandedVyMetersPerSecond,
					commandedOmegaRadPerSecond,
					false,
					false,
					translationController,
					headingController);

			request.withVelocityX(commandedVxMetersPerSecond)
					.withVelocityY(commandedVyMetersPerSecond)
					.withRotationalRate(commandedOmegaRadPerSecond);

			return request;
		};
	}

	private static void logChoreoControllerTelemetry(
			SwerveSample sample,
			Pose2d currentPose,
			Pose2d desiredPose,
			Translation2d translationError,
			double translationErrorNorm,
			double translationPidMagnitudeMetersPerSecond,
			Translation2d pidTranslationOutput,
			double headingErrorRad,
			double headingPidOutputRadPerSecond,
			double commandedVxMetersPerSecond,
			double commandedVyMetersPerSecond,
			double commandedOmegaRadPerSecond,
			boolean linearOutputSaturated,
			boolean omegaOutputSaturated,
			SynchronousPIDF translationController,
			SynchronousPIDF headingController) {
		LogUtil.log("Choreo/Controller/CurrentPose", currentPose);
		LogUtil.log("Choreo/Controller/DesiredPose", desiredPose);

		SmartDashboard.putNumber("Choreo/Controller/Sample Time Seconds", sample.t);

		SmartDashboard.putNumber("Choreo/Controller/Translation Error X Meters", translationError.getX());
		SmartDashboard.putNumber("Choreo/Controller/Translation Error Y Meters", translationError.getY());
		SmartDashboard.putNumber("Choreo/Controller/Translation Error Norm Meters", translationErrorNorm);
		SmartDashboard.putNumber("Choreo/Controller/Heading Error Radians", headingErrorRad);

		SmartDashboard.putNumber("Choreo/Controller/Feedforward Vx Meters Per Second", sample.vx);
		SmartDashboard.putNumber("Choreo/Controller/Feedforward Vy Meters Per Second", sample.vy);
		SmartDashboard.putNumber("Choreo/Controller/Feedforward Omega Radians Per Second", sample.omega);

		SmartDashboard.putNumber(
				"Choreo/Controller/Translation PID Output Magnitude Meters Per Second",
				-translationPidMagnitudeMetersPerSecond);
		SmartDashboard.putNumber(
				"Choreo/Controller/Translation PID Output X Meters Per Second", pidTranslationOutput.getX());
		SmartDashboard.putNumber(
				"Choreo/Controller/Translation PID Output Y Meters Per Second", pidTranslationOutput.getY());
		SmartDashboard.putNumber(
				"Choreo/Controller/Heading PID Output Radians Per Second", headingPidOutputRadPerSecond);

		SmartDashboard.putNumber("Choreo/Controller/Commanded Vx Meters Per Second", commandedVxMetersPerSecond);
		SmartDashboard.putNumber("Choreo/Controller/Commanded Vy Meters Per Second", commandedVyMetersPerSecond);
		SmartDashboard.putNumber("Choreo/Controller/Commanded Omega Radians Per Second", commandedOmegaRadPerSecond);
		SmartDashboard.putNumber(
				"Choreo/Controller/Commanded Speed Magnitude Meters Per Second",
				Math.hypot(commandedVxMetersPerSecond, commandedVyMetersPerSecond));

		var measuredFieldSpeeds = Drive.mInstance.getFieldRelativeSpeeds();
		SmartDashboard.putNumber(
				"Choreo/Controller/Measured Vx Meters Per Second", measuredFieldSpeeds.vxMetersPerSecond);
		SmartDashboard.putNumber(
				"Choreo/Controller/Measured Vy Meters Per Second", measuredFieldSpeeds.vyMetersPerSecond);
		SmartDashboard.putNumber(
				"Choreo/Controller/Measured Omega Radians Per Second", measuredFieldSpeeds.omegaRadiansPerSecond);

		SmartDashboard.putBoolean("Choreo/Controller/Linear Output Saturated", linearOutputSaturated);
		SmartDashboard.putBoolean("Choreo/Controller/Omega Output Saturated", omegaOutputSaturated);

		SmartDashboard.putNumber("Choreo/Controller/Translation kP", translationController.getP());
		SmartDashboard.putNumber("Choreo/Controller/Translation kI", translationController.getI());
		SmartDashboard.putNumber("Choreo/Controller/Translation kD", translationController.getD());
		SmartDashboard.putNumber("Choreo/Controller/Theta kP", headingController.getP());
		SmartDashboard.putNumber("Choreo/Controller/Theta kI", headingController.getI());
		SmartDashboard.putNumber("Choreo/Controller/Theta kD", headingController.getD());
	}

	public static final UnaryOperator<SwerveRequest.FieldCentric> getPIDToPoseRequestUpdater(
			Pose2d targetPose, PIDController translationController, SynchronousPIDF headingController) {
		return (SwerveRequest.FieldCentric request) -> {
			Pose2d currentPose = Drive.mInstance.getPose();
			Translation2d deltaTranslation = currentPose.getTranslation().minus(targetPose.getTranslation());
			double translationErrorNorm = deltaTranslation.getNorm();
			Translation2d unscaledOutput = deltaTranslation.div(translationErrorNorm);
			Translation2d scaledOutput = unscaledOutput.times(translationController.calculate(translationErrorNorm));

			// double speedMetersPerSecond = Math.max(0.0, -translationController.calculate(distanceToTargetMeters));
			// Rotation2d velocityDirection = toTargetTranslation.getAngle();
			// double commandedVxMetersPerSecond = speedMetersPerSecond * velocityDirection.getCos();
			// double commandedVyMetersPerSecond = speedMetersPerSecond * velocityDirection.getSin();

			double commandedVxMetersPerSecond = scaledOutput.getX();
			double commandedVyMetersPerSecond = scaledOutput.getY();

			headingController.setSetpoint(Units.Radians.of(
							MathUtil.angleModulus(targetPose.getRotation().getRadians()))
					.in(Units.Rotations));
			double commandedRotationsPerSecond = headingController.calculate(Units.Radians.of(
							MathUtil.angleModulus(currentPose.getRotation().getRadians()))
					.in(Units.Rotations));

			LogUtil.log("PID To Pose Updater/Target Pose", targetPose);
			SmartDashboard.putNumber("ClimbAutoAlign/TargetX", targetPose.getX());
			SmartDashboard.putNumber("ClimbAutoAlign/TargetY", targetPose.getY());
			SmartDashboard.putNumber("ClimbAutoAlign/CurrentX", currentPose.getX());
			SmartDashboard.putNumber("ClimbAutoAlign/CurrentY", currentPose.getY());
			SmartDashboard.putNumber("ClimbAutoAlign/CmdVx", commandedVxMetersPerSecond);
			SmartDashboard.putNumber("ClimbAutoAlign/CmdVy", commandedVyMetersPerSecond);
			SmartDashboard.putNumber("ClimbAutoAlign/CmdOmegaRps", commandedRotationsPerSecond);

			request.withVelocityX(commandedVxMetersPerSecond)
					.withVelocityY(commandedVyMetersPerSecond)
					.withRotationalRate(Units.RotationsPerSecond.of(commandedRotationsPerSecond));

			return request;
		};
	}

	private static SynchronousPIDF getAutoAlignHeadingController() {
		SynchronousPIDF controller = new SynchronousPIDF(
				3.2, // Proportional
				0.0, // Integral
				0.0 // Derivative
				);
		controller.setInputRange(-0.5, 0.5);
		if (Drive.mInstance == null) {
			controller.setMaxAbsoluteOutput(
					Units.RadiansPerSecond.of(2.75 * Math.PI).in(Units.RotationsPerSecond));
		} else {
			controller.setMaxAbsoluteOutput(kMaxAngularRate.in(Units.RotationsPerSecond));
		}
		controller.setContinuous();
		return controller;
	}

	public static PIDController getAutoAlignTranslationController() {
		PIDController controller = new PIDController(3.5, 0.1, 0.2);
		controller.setIZone(0.1);
		return controller;
	}

	public static CurrentLimitsConfigs getTeleConfig() {
		return new CurrentLimitsConfigs()
				.withStatorCurrentLimit(Amps.of(60.0))
				.withStatorCurrentLimitEnable(true)
				.withSupplyCurrentLimit(Amps.of(30.0))
				.withSupplyCurrentLowerLimit(Amps.of(30.0))
				.withSupplyCurrentLimitEnable(true)
				.withSupplyCurrentLowerTime(Seconds.of(1.0));
	}
}
