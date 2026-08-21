package frc.robot.shooting.shootingplanner;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.lib.logging.LogUtil;
import frc.lib.math.ChassisAccelerations;
import frc.lib.util.MathHelpers;
import frc.lib.util.tunablenumbers.TunableNumber;
import frc.robot.RobotConstants;
import frc.robot.shooting.verification.ShotVerifierConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class MotionCompensatedShootingPlanner implements Sendable {

	private final Transform2d m_centerOfRotationOffset;
	private Pose2d m_currentLookAhead = new Pose2d();
	private Translation2d m_currentLookAheadOffset = new Translation2d();

	private boolean m_rejectLookAhead = false;

	private MotionCompensatedShootingInputs m_lastInputs = MotionCompensatedShootingInputs.zero();

	public final TunableNumber m_tF = new TunableNumber("Time of Flight", 0.0);
	public final TunableNumber m_tE = new TunableNumber("Time of Pass", 0.0);

	public static final Time FUEL_HOPPER_TO_EXIT_TIME = Seconds.of(0.0);

	public MotionCompensatedShootingPlanner(Transform2d centerOfRotationOffset) {
		m_centerOfRotationOffset = centerOfRotationOffset;
		SendableRegistry.add(this, "Drive Tracker");
	}

	public Translation2d getOffsetTranslation(MotionCompensatedShootingInputs inputs) {
		double cos = inputs.drivePose().getRotation().getCos();
		double sin = inputs.drivePose().getRotation().getSin();
		Translation2d driveTranslation = Drive.mInstance.getPose().getTranslation();
		return new Translation2d(
				m_centerOfRotationOffset.getX() * cos - m_centerOfRotationOffset.getY() * sin,
				m_centerOfRotationOffset.getX() * sin + m_centerOfRotationOffset.getY() * cos);
	}

	public Translation2d getShooterOffsetTranslation(MotionCompensatedShootingInputs inputs) {
		double offsetFromCenter =
				m_centerOfRotationOffset.getTranslation().times(0.0).getNorm();
		ChassisSpeeds speeds = inputs.driveSpeeds();
		Pose2d drivePose = inputs.drivePose();
		double dt = inputs.dt();
		double delay = FUEL_HOPPER_TO_EXIT_TIME.in(Seconds);

		Rotation2d theta = drivePose
				.getRotation()
				.plus(
						Rotation2d.fromRadians(
								(speeds.omegaRadiansPerSecond + inputs.driveAccelerations().m_alpha * delay))
						// Rotation2d.kZero
						);

		Rotation2d phi = theta.plus(Rotation2d.k180deg)
				.plus(Rotation2d.fromRadians(Math.copySign(Math.PI / 2.0, speeds.omegaRadiansPerSecond)));

		return new Translation2d(
				offsetFromCenter * speeds.omegaRadiansPerSecond * phi.getCos(),
				offsetFromCenter * speeds.omegaRadiansPerSecond * phi.getSin());
	}

	public boolean getYawUnstable() {
		return Math.abs(Drive.mInstance.getFieldRelativeSpeeds().omegaRadiansPerSecond)
				>= DriveConstants.YAW_UNSTABLE_VELOCITY.in(RadiansPerSecond);
	}

	public boolean getErrorTolerable() {
		return getDriveError().abs(Degrees) <= ShotVerifierConstants.TOLERRABLE_ANGLE_RANGE_TUNNABLE.getAsDouble();
	}

	public void updateRejectLookAhead() {
		boolean yawUnstable = getYawUnstable();
		boolean errorIsNotTolerable = !getErrorTolerable();
		Pose2d drivePose = Drive.mInstance.getPose();
		boolean staticShot =
				Math.hypot(m_lastInputs.driveSpeeds().vxMetersPerSecond, m_lastInputs.driveSpeeds().vyMetersPerSecond)
						< 0.09;
		m_rejectLookAhead = (yawUnstable && errorIsNotTolerable) || staticShot;
	}

	public static ChassisSpeeds transformVelocity(
			ChassisSpeeds velocity, Translation2d transform, Rotation2d currentRotation) {
		return new ChassisSpeeds(
				velocity.vxMetersPerSecond
						- velocity.omegaRadiansPerSecond
								* (transform.getX() * currentRotation.getSin()
										+ transform.getY() * currentRotation.getCos()),
				velocity.vyMetersPerSecond
						+ velocity.omegaRadiansPerSecond
								* (transform.getX() * currentRotation.getCos()
										- transform.getY() * currentRotation.getSin()),
				velocity.omegaRadiansPerSecond);
	}

	public void updateLookAhead(MotionCompensatedShootingInputs inputs) {
		Pose2d centerPose = inputs.drivePose();
		double offsetFromCenter = m_centerOfRotationOffset.getTranslation().getX();
		ChassisSpeeds speeds = inputs.driveSpeeds();
		ChassisAccelerations acceleration = inputs.driveAccelerations();
		double timeOfFlight = inputs.dt();
		double launchDelay = FUEL_HOPPER_TO_EXIT_TIME.in(Seconds);

		Translation2d shooterOffset = getShooterOffsetTranslation(inputs);

		Pose2d compensatedPose = centerPose.exp(new Twist2d(
				speeds.vxMetersPerSecond * launchDelay,
				speeds.vyMetersPerSecond * launchDelay,
				speeds.omegaRadiansPerSecond * launchDelay));

		Rotation2d theta = compensatedPose.getRotation();

		Rotation2d phi = theta.plus(Rotation2d.fromRadians(Math.copySign(Math.PI / 2.0, speeds.omegaRadiansPerSecond)));

		ChassisSpeeds transformed = new ChassisSpeeds(
				speeds.vxMetersPerSecond + launchDelay * offsetFromCenter * phi.getCos(),
				speeds.vyMetersPerSecond + launchDelay * offsetFromCenter * phi.getSin(),
				0.0);

		m_currentLookAhead = compensatedPose.exp(new Twist2d(
				((transformed.vxMetersPerSecond) * timeOfFlight),
				((transformed.vyMetersPerSecond) * timeOfFlight),
				0.0));
	}

	public void update(MotionCompensatedShootingInputs inputs) {
		m_lastInputs = inputs;
		updateLookAhead(inputs);
		updateRejectLookAhead();
		outputTelemtry();
	}

	public Translation2d getTarget() {
		return m_lastInputs.state().getLocationTranslation(RobotConstants.isRedAlliance);
	}

	public Translation2d getLookAheadOffset() {
		return m_currentLookAheadOffset;
	}

	public Pose2d getLookAhead() {
		return m_currentLookAhead;
	}

	public Angle getTargetAngle(Translation2d baseTranslation) {
		Translation2d target = getTarget();
		return target.minus(baseTranslation)
				.getAngle()
				.plus(m_centerOfRotationOffset.getRotation())
				.getMeasure();
	}

	public Angle getTargetAngle() {
		Translation2d baseTranslation = getBaseTranslation();
		return getTargetAngle(baseTranslation);
	}

	public Angle getDriveError() {
		return MathHelpers.angleModulus(getTargetAngle(getBaseTranslation())
				.minus(Drive.mInstance.getPose().getRotation().getMeasure()));
	}

	public Translation2d getBaseTranslation() {
		return m_rejectLookAhead ? Drive.mInstance.getPose().getTranslation() : m_currentLookAhead.getTranslation();
	}

	public boolean getRejectLookAhead() {
		return m_rejectLookAhead;
	}

	public void outputTelemtry() {
		LogUtil.log("Drive Tracker/LookAhead", m_currentLookAhead);
		LogUtil.log(
				"Drive Tracker/Output",
				new Pose2d(getBaseTranslation(), Drive.mInstance.getPose().getRotation()));
		m_lastInputs.telemetrize();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addBooleanProperty("Reject Look Ahead/Status", () -> m_rejectLookAhead, null);
		builder.addDoubleProperty("Align/Target Radians", () -> getTargetAngle().in(Radians), null);
		builder.addDoubleProperty(
				"Align/Drive Error Radians", () -> getDriveError().abs(Radians), null);
		builder.addBooleanProperty("Reject Look Ahead/Yaw Stable", () -> !getYawUnstable(), null);
		builder.addBooleanProperty("Reject Look Ahead/Error Tolerablle", () -> getErrorTolerable(), null);
	}
}
