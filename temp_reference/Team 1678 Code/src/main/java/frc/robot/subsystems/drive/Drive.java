package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LogUtil;
import frc.lib.math.ChassisAccelerations;
import frc.lib.math.KinematicHelpers;
import frc.lib.util.tracker.DerivativeTracker;
import frc.lib.util.tracker.LinearFilterDerivativeTracker;
import frc.robot.Telemetry;
import java.util.function.UnaryOperator;

public class Drive extends SubsystemBase {
	public static final Drive mInstance = new Drive();

	private SwerveDriveState lastReadState;
	private SwerveRequest driveRequest = DriveConstants.teleopRequest;
	private final GeneratedDrivetrain drivetrain = TunerConstants.createDrivetrain();
	private final Telemetry telemetry = new Telemetry(DriveConstants.kMaxSpeed.baseUnitMagnitude());
	private final LinearFilterDerivativeTracker m_translationAccelerationXFiltered =
			new LinearFilterDerivativeTracker(LinearFilter.movingAverage(5), 0.00);
	private final LinearFilterDerivativeTracker m_translationAccelerationYFiltered =
			new LinearFilterDerivativeTracker(LinearFilter.movingAverage(5), 0.00);

	private final DerivativeTracker m_translationAccelerationX = new DerivativeTracker(0.0);
	private final DerivativeTracker m_translationAccelerationY = new DerivativeTracker(0.0);

	private Translation2d m_translationAcceleration = new Translation2d();

	private final DerivativeTracker m_angularAccelerationTrackerFiltered =
			new LinearFilterDerivativeTracker(LinearFilter.movingAverage(5), 0.0);
	private final DerivativeTracker m_angularAccelerationTracker = new DerivativeTracker(0.0);

	private double m_filteredTranslationVelocityX = 0;
	private double m_filteredTranslationVelocityY = 0;
	private double m_filteredAngularVelocity = 0;

	private final LinearFilter m_translationXVelocityFilter = LinearFilter.movingAverage(5);
	private final LinearFilter m_translationYVelocityFilter = LinearFilter.movingAverage(5);
	private final LinearFilter m_angularVelocityFilter = LinearFilter.movingAverage(5);

	private final Debouncer yawDebouncer = new Debouncer(0.1);
	private final Debouncer pitchDebouncer = new Debouncer(0.1);

	private StructPublisher<Pose3d> mechanismPublisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Drivetrain", Pose3d.struct)
			.publish();

	private final Field2d elasticPose = new Field2d();

	private Drive() {
		lastReadState = drivetrain.getState();
		drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
			return driveRequest;
		}));

		drivetrain.getOdometryThread().setThreadPriority(31);
	}

	public GeneratedDrivetrain getGeneratedDrive() {
		return drivetrain;
	}

	public Translation2d getTranslationAccelerationFromPigeon(Translation2d pigeonOffset) {
		double omegaRadsPerSecond = Drive.mInstance.getState().Speeds.omegaRadiansPerSecond;
		Pigeon2 pigeon = drivetrain.getPigeon2();
		return new Translation2d(
				pigeon.getAccelerationX().getValue().in(MetersPerSecondPerSecond)
						- (getAngularAcceleration().in(RadiansPerSecondPerSecond) * pigeonOffset.getX())
						- (omegaRadsPerSecond * omegaRadsPerSecond * pigeonOffset.getX()),
				pigeon.getAccelerationY().getValue().in(MetersPerSecondPerSecond)
						- (getAngularAcceleration().in(RadiansPerSecondPerSecond) * pigeonOffset.getY())
						- (omegaRadsPerSecond * omegaRadsPerSecond * pigeonOffset.getY()));
	}

	public ChassisAccelerations getFilteredAccelerations() {
		return new ChassisAccelerations(
				m_translationAccelerationXFiltered.getTrackedDerivative(),
				m_translationAccelerationYFiltered.getTrackedDerivative(),
				m_angularAccelerationTrackerFiltered.getTrackedDerivative());
	}

	public ChassisAccelerations getAccelerations() {
		return new ChassisAccelerations(
				m_translationAccelerationX.getTrackedDerivative(),
				m_translationAccelerationY.getTrackedDerivative(),
				m_angularAccelerationTracker.getTrackedDerivative());
	}

	public ChassisSpeeds getFilteredChasisSpeeds() {
		return new ChassisSpeeds(
				m_filteredTranslationVelocityX, m_filteredTranslationVelocityY, m_filteredAngularVelocity);
	}

	public Translation2d getTranslationAccelerationFromPigeon() {
		return getTranslationAccelerationFromPigeon(DriveConstants.PIGEON_OFFSET);
	}

	public Translation2d getTranslationAcceleration() {
		return m_translationAcceleration;
	}

	public Translation2d getFilteredTranslationAcceleration() {
		return new Translation2d(
				m_translationAccelerationXFiltered.getTrackedDerivative(),
				m_translationAccelerationYFiltered.getTrackedDerivative());
	}

	public AngularAcceleration getAngularAcceleration() {
		return RadiansPerSecondPerSecond.of(m_angularAccelerationTrackerFiltered.getTrackedDerivative());
	}

	@Override
	public void periodic() {
		lastReadState = drivetrain.getState();
		updateAcceleration();
		outputTelemetry();
		updateFilteredVelocity();
	}

	public void updateFilteredVelocity() {
		ChassisSpeeds speeds = Drive.mInstance.getState().Speeds;
		m_filteredTranslationVelocityX = m_translationXVelocityFilter.calculate(speeds.vxMetersPerSecond);
		m_filteredTranslationVelocityY = m_translationYVelocityFilter.calculate(speeds.vyMetersPerSecond);
		m_filteredAngularVelocity = m_angularVelocityFilter.calculate(speeds.omegaRadiansPerSecond);
	}

	public void updateAcceleration() {
		ChassisSpeeds speeds = getState().Speeds;
		m_translationAccelerationXFiltered.update(speeds.vxMetersPerSecond, 0.02);
		m_translationAccelerationYFiltered.update(speeds.vyMetersPerSecond, 0.02);
		m_angularAccelerationTrackerFiltered.update(speeds.omegaRadiansPerSecond, 0.02);

		m_translationAccelerationX.update(speeds.vxMetersPerSecond, 0.02);
		m_translationAccelerationY.update(speeds.vyMetersPerSecond, 0.02);
		m_angularAccelerationTracker.update(speeds.omegaRadiansPerSecond, 0.02);
	}

	public void outputTelemetry() {
		mechanismPublisher.set(new Pose3d(getPose()));
		telemetry.telemeterize(lastReadState);
		SmartDashboard.putData("Drive", this);
		elasticPose.setRobotPose(getPose());
		LogUtil.log("Drive/Compensated Pose", getPhaseDelayedPose());
		SmartDashboard.putData("Elastic Field 2D", elasticPose);
	}

	public Pose2d getPhaseDelayedPose() {
		ChassisSpeeds speeds = getState().Speeds;
		double dt = DriveConstants.PHASE_DELAY.in(Seconds);
		return KinematicHelpers.secondOrderApproximate(getPose(), speeds, getFilteredAccelerations(), dt);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addStringProperty(
				"Swerve Request Type", () -> driveRequest.getClass().getSimpleName(), null);
		builder.addDoubleProperty(
				"Pitch Velocity Degrees Per Second",
				() -> drivetrain
						.getPigeon2()
						.getAngularVelocityYDevice()
						.getValue()
						.in(Units.DegreesPerSecond),
				null);
		builder.addDoubleProperty(
				"Pitch Degrees",
				() -> drivetrain.getPigeon2().getPitch().getValue().in(Units.Degrees),
				null);
		builder.addDoubleProperty(
				"Angular Acceleration Radians Per Second Per Second",
				() -> getAngularAcceleration().in(RadiansPerSecondPerSecond),
				null);

		builder.addDoubleProperty(
				"Roll Velocity Degrees Per Second",
				() -> drivetrain
						.getPigeon2()
						.getAngularVelocityXDevice()
						.getValue()
						.in(Units.DegreesPerSecond),
				null);
		builder.addDoubleProperty(
				"Roll Degrees",
				() -> drivetrain.getPigeon2().getRoll().getValue().in(Units.Degrees),
				null);

		builder.addDoubleProperty(
				"Tilt", () -> getFieldAlignedPigeonAngle().getMeasureY().in(Degrees), null);

		builder.addBooleanProperty(
				"Pigeon Connected", () -> drivetrain.getPigeon2().isConnected(), null);

		builder.addDoubleProperty(
				"Field Velocity/X Meters Per Second", () -> getFieldRelativeSpeeds().vxMetersPerSecond, null);
		builder.addDoubleProperty(
				"Field Velocity/Y Meters per Second", () -> getFieldRelativeSpeeds().vyMetersPerSecond, null);
		builder.addDoubleProperty(
				"Field Velocity/O Radians per Second", () -> getFieldRelativeSpeeds().omegaRadiansPerSecond, null);
		builder.addDoubleProperty(
				"Field Velocity/Direction Meters per Second",
				() -> Math.hypot(
						getFieldRelativeSpeeds().vxMetersPerSecond, getFieldRelativeSpeeds().vyMetersPerSecond),
				null);

		builder.addDoubleProperty(
				"Filtered Field Velocity/X Meters Per Second", () -> m_filteredTranslationVelocityX, null);
		builder.addDoubleProperty(
				"Filtered Field Velocity/Y Meters per Second", () -> m_filteredTranslationVelocityY, null);
		builder.addDoubleProperty(
				"Filtered Field Velocity/O Radians per Second", () -> m_filteredAngularVelocity, null);
		builder.addDoubleProperty(
				"Filtered Field Velocity/Direction Meters per Second",
				() -> Math.hypot(m_filteredTranslationVelocityX, m_filteredTranslationVelocityY),
				null);

		builder.addDoubleProperty(
				"Filtered Acceleration Translation/X Meters Per Second Per Second",
				() -> m_translationAccelerationXFiltered.getTrackedDerivative(),
				null);
		builder.addDoubleProperty(
				"Filtered Acceleration Translation/Y Meters Per Second Per Second",
				() -> m_translationAccelerationYFiltered.getTrackedDerivative(),
				null);

		builder.addDoubleProperty(
				"Acceleration Translation/X Meters Per Second Per Second",
				() -> m_translationAccelerationX.getTrackedDerivative(),
				null);
		builder.addDoubleProperty(
				"Acceleration Translation/Y Meters Per Second Per Second",
				() -> m_translationAccelerationY.getTrackedDerivative(),
				null);

		builder.addBooleanProperty("Pitch Stable", () -> pitchStable(), null);
		addModuleToBuilder(builder, 0);
		addModuleToBuilder(builder, 1);
		addModuleToBuilder(builder, 2);
		addModuleToBuilder(builder, 3);
	}

	private void addModuleToBuilder(SendableBuilder builder, int module) {
		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Drive/Volts",
				() -> drivetrain
						.getModules()[module]
						.getDriveMotor()
						.getMotorVoltage()
						.getValue()
						.in(Units.Volts),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Rotation/Volts",
				() -> drivetrain
						.getModules()[module]
						.getSteerMotor()
						.getMotorVoltage()
						.getValue()
						.in(Units.Volts),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Drive/Stator Current",
				() -> drivetrain
						.getModules()[module]
						.getDriveMotor()
						.getStatorCurrent()
						.getValue()
						.in(Units.Amps),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Drive/Temperature Celsius",
				() -> drivetrain
						.getModules()[module]
						.getDriveMotor()
						.getDeviceTemp()
						.getValue()
						.in(Units.Celsius),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Rotation/Stator Current",
				() -> drivetrain
						.getModules()[module]
						.getSteerMotor()
						.getStatorCurrent()
						.getValue()
						.in(Units.Amps),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Drive/Supply Current",
				() -> drivetrain
						.getModules()[module]
						.getDriveMotor()
						.getSupplyCurrent()
						.getValue()
						.in(Units.Amps),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Rotation/Supply Current",
				() -> drivetrain
						.getModules()[module]
						.getSteerMotor()
						.getSupplyCurrent()
						.getValue()
						.in(Units.Amps),
				null);

		builder.addDoubleProperty(
				"ModuleStates/" + module + "/Rotation/Temperature Celsius",
				() -> drivetrain
						.getModules()[module]
						.getSteerMotor()
						.getDeviceTemp()
						.getValue()
						.in(Units.Celsius),
				null);
		ChassisSpeeds speeds = getState().Speeds;
		builder.addDoubleProperty(
				"Translation Velocity", () -> Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), null);
	}

	public Pose2d getLookAheadPoseWithoutHeading(Time lookAheadTime) {
		ChassisSpeeds speeds = getState().Speeds;
		return getPose()
				.exp(new Twist2d(
						speeds.vxMetersPerSecond * lookAheadTime.in(Seconds),
						speeds.vyMetersPerSecond * lookAheadTime.in(Seconds),
						0.0));
	}

	public Pose2d getLookAheadPose(Time lookAheadTime) {
		ChassisSpeeds speeds = getState().Speeds;
		return getPose()
				.exp(new Twist2d(
						speeds.vxMetersPerSecond * lookAheadTime.in(Seconds),
						speeds.vyMetersPerSecond * lookAheadTime.in(Seconds),
						speeds.omegaRadiansPerSecond * lookAheadTime.in(Seconds)));
	}

	public Rotation2d getLookAheadHeading(Time lookAheadTime) {
		ChassisSpeeds speeds = getState().Speeds;
		return getPose()
				.exp(new Twist2d(0.0, 0.0, speeds.omegaRadiansPerSecond * lookAheadTime.in(Seconds)))
				.getRotation();
	}

	public SwerveDriveState getState() {
		return drivetrain.getState();
	}

	public ChassisSpeeds getFieldRelativeSpeeds() {
		return ChassisSpeeds.fromRobotRelativeSpeeds(
				getState().Speeds, getPose().getRotation());
	}

	public Pose2d getPose() {
		return lastReadState.Pose;
	}

	public void setSwerveRequest(SwerveRequest request) {
		driveRequest = request;
		drivetrain.setControl(request);
	}

	public Command followSwerveRequestCommand(
			SwerveRequest.FieldCentric request, UnaryOperator<SwerveRequest.FieldCentric> updater) {
		return run(() -> setSwerveRequest(updater.apply(request)))
				.handleInterrupt(() -> setSwerveRequest(new SwerveRequest.FieldCentric()));
	}

	public void followChoreoTrajectory(SwerveSample sample) {
		setSwerveRequest(DriveConstants.getChoreoPathRequestUpdater(sample).apply(DriveConstants.choreoRequest));
	}

	public void addVisionUpdate(Pose2d pose, Time timestamp) {
		getGeneratedDrive().addVisionMeasurement(pose, timestamp.in(Units.Seconds));
	}

	public void addVisionUpdate(Pose2d pose, Time timestamp, Matrix<N3, N1> stdDevs) {
		getGeneratedDrive().addVisionMeasurement(pose, timestamp.in(Units.Seconds), stdDevs);
	}

	public void resetPose(Pose2d pose) {
		getGeneratedDrive().resetPose(pose);
	}

	public Command resetPoseCmd(Pose2d pose) {
		return Commands.runOnce(() -> resetPose(pose));
	}

	public boolean yawStable() {
		boolean yawIsStable =
				drivetrain.getPigeon2().getAngularVelocityZDevice().getValue().abs(Units.DegreesPerSecond)
						< DriveConstants.maxYawStableThreshold.in(Units.DegreesPerSecond);

		return yawDebouncer.calculate(yawIsStable);
	}

	public boolean pitchStable() {
		boolean pitchIsStable = drivetrain.getPigeon2().getPitch().getValue().abs(Units.Degrees)
				< DriveConstants.maxPitchStableThreshold.in(Units.Degrees);

		return pitchDebouncer.calculate(pitchIsStable);
	}

	public boolean driveStable() {
		SmartDashboard.putBoolean("Yaw Stable", yawStable());
		SmartDashboard.putBoolean("Pitch Stable", pitchStable());
		return yawStable() && pitchStable();
	}

	public Translation2d getDriveSpeedsAsTranslation() {
		ChassisSpeeds speed = Drive.mInstance.getState().Speeds;
		return new Translation2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
	}

	public Rotation3d getFieldAlignedPigeonAngle() {
		Rotation3d robotRotation =
				Drive.mInstance.getGeneratedDrive().getPigeon2().getRotation3d();
		SmartDashboard.putString("Pigeon Rotation3d", robotRotation.toString());
		Rotation3d robotYaw = new Rotation3d(
				Degrees.zero(),
				Degrees.zero(),
				Drive.mInstance.getPose().getRotation().getMeasure());

		return robotYaw.unaryMinus().plus(robotRotation);
	}

	public void configDrivetainCurrent(CurrentLimitsConfigs config) {
		drivetrain.getModule(0).getDriveMotor().getConfigurator().apply(config);
		drivetrain.getModule(1).getDriveMotor().getConfigurator().apply(config);
		drivetrain.getModule(2).getDriveMotor().getConfigurator().apply(config);
		drivetrain.getModule(3).getDriveMotor().getConfigurator().apply(config);
	}
}
