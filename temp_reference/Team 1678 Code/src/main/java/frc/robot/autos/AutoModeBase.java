package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout;
import frc.lib.util.Stopwatch;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.commands.PIDToPoseCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.Cameras;
import java.util.Arrays;
import java.util.List;

public class AutoModeBase {
	private AutoRoutine routine;
	private static Stopwatch stopwatch = new Stopwatch();
	Debouncer hitDebouncer =
			new Debouncer(AutoConstants.MIDFIELD_CONTACT_HIT_DEBOUNCE.in(Seconds), DebounceType.kRising);
	private static boolean simulateCrash = false;

	private static final Time MAX_POSE_AGE_FOR_BUMP_CORRECTION = Seconds.of(Robot.kDefaultPeriod * 2.0);

	private final String name;

	public AutoModeBase(AutoFactory factory, String name) {
		routine = factory.newRoutine(name);
		this.name = name;
	}

	public AutoTrajectory mirroredTrajectory(AutoTrajectory trajectory) {
		return mirroredTrajectoryFromRaw(trajectory.<SwerveSample>getRawTrajectory());
	}

	public AutoTrajectory mirroredTrajectory(String name) {
		return mirroredTrajectoryFromRaw(trajectory(name).<SwerveSample>getRawTrajectory());
	}

	private AutoTrajectory mirroredTrajectoryFromRaw(Trajectory<SwerveSample> trajectory) {
		return routine.trajectory(mirrorChoreoTrajectoryAcrossMidline(trajectory));
	}

	public static Trajectory<SwerveSample> mirrorChoreoTrajectoryAcrossMidline(Trajectory<SwerveSample> trajectory) {
		return mirrorSwerveTrajectoryAcrossY(trajectory);
	}

	private static Trajectory<SwerveSample> mirrorSwerveTrajectoryAcrossY(Trajectory<SwerveSample> trajectory) {
		final double fieldWidthMeters = FieldLayout.kFieldWidth.in(Units.Meters);
		List<SwerveSample> mirroredSamples = trajectory.samples().stream()
				.map(sample -> mirrorSwerveSampleAcrossY(sample, fieldWidthMeters))
				.toList();
		return new Trajectory<>(trajectory.name(), mirroredSamples, trajectory.splits(), trajectory.events());
	}

	private static SwerveSample mirrorSwerveSampleAcrossY(SwerveSample sample, double fieldWidthMeters) {
		double[] fx = sample.moduleForcesX();
		double[] fy = sample.moduleForcesY();
		double[] mirroredFx = new double[] {fx[1], fx[0], fx[3], fx[2]};
		double[] mirroredFy = new double[] {-fy[1], -fy[0], -fy[3], -fy[2]};

		return new SwerveSample(
				sample.t,
				sample.x,
				fieldWidthMeters - sample.y,
				-sample.heading,
				sample.vx,
				-sample.vy,
				-sample.omega,
				sample.ax,
				-sample.ay,
				-sample.alpha,
				mirroredFx,
				mirroredFy);
	}

	/**
	 * @return Trajectory from choreo
	 */
	public AutoTrajectory trajectory(String name) {
		return routine.trajectory(name);
	}

	public static void setCrashDetectionOverride(boolean val) {
		simulateCrash = val;
	}

	public AutoTrajectory trajectory(String name, int index) {
		return routine.trajectory(name, index);
	}

	public <SampleType extends TrajectorySample<SampleType>> AutoTrajectory trajectory(
			Trajectory<SampleType> trajectory) {
		return routine.trajectory(trajectory);
	}

	public Command cmdWithCollisonAvoidance(AutoTrajectory trajectory, SwerveRequest.FieldCentric request) {
		Timer trajTimer = new Timer();

		return Commands.sequence(
						Commands.race(
								Commands.waitUntil(() -> collisionDetected(trajectory, trajTimer)), trajectory.cmd()),
						Commands.either(
								recoverFromCollision(request, getTrajectoryFinalPose(trajectory)),
								Commands.runOnce(
										() -> SmartDashboard.putBoolean("Collisions/Attempting recovery", false)),
								() -> collisionDetected(trajectory, trajTimer)))
				.beforeStarting(trajTimer::restart) // timer reset in initialize, not in defer
				.finallyDo(() -> setCrashDetectionOverride(false))
				.withName("Auto/Choreo With Collision Avoidance");
	}

	private Command recoverFromCollision(SwerveRequest.FieldCentric request, Pose2d finalPose) {
		return Commands.sequence(
				Commands.runOnce(() -> SmartDashboard.putBoolean("Collisions/Attempting recovery", true)),
				Commands.runOnce(
						() -> Drive.mInstance.setSwerveRequest(request.withVelocityX(collisionclearvelocity()))),
				Commands.waitSeconds(0.4),
				new PIDToPoseCommand(
						finalPose,
						AutoConstants.MIDFIELD_RECOVERY_TRANSLATION_TOLERANCE,
						AutoConstants.MIDFIELD_RECOVERY_ROTATION_TOLERANCE));
	}

	public Pose2d getInitialPose() {
		return new Pose2d();
	}

	private Pose2d getTrajectoryFinalPose(AutoTrajectory trajectory) {
		return trajectory.getFinalPose().orElseGet(() -> trajectory
				.<SwerveSample>getRawTrajectory()
				.getFinalPose(RobotConstants.isRedAlliance)
				.orElse(Drive.mInstance.getPose()));
	}

	private boolean belowExpectedSpeed(AutoTrajectory trajectory, Timer trajectoryTimer) {
		ChassisSpeeds currentFieldSpeeds = Drive.mInstance.getFieldRelativeSpeeds();
		double currentSpeedMetersPerSecond =
				Math.hypot(currentFieldSpeeds.vxMetersPerSecond, currentFieldSpeeds.vyMetersPerSecond);
		double expectedSpeedMetersPerSecond = trajectory
				.<SwerveSample>getRawTrajectory()
				.sampleAt(trajectoryTimer.get(), RobotConstants.isRedAlliance)
				.map(sample -> Math.hypot(sample.vx, sample.vy))
				.orElse(0.0);
		double speedRatio =
				expectedSpeedMetersPerSecond != 0.0 ? currentSpeedMetersPerSecond / expectedSpeedMetersPerSecond : 1.0;
		return speedRatio <= AutoConstants.MIDFIELD_CONTACT_MAX_ACTUAL_SPEED_FRACTION_OF_EXPECTED;
	}

	private boolean collisionDetected(AutoTrajectory trajectory, Timer trajectoryTimer) {
		Pose2d currentPose = Drive.mInstance.getPose();

		boolean belowSpeed = belowExpectedSpeed(trajectory, trajectoryTimer);
		boolean insideNeutralZone = AutoHelpers.isInsideNeutralZone(currentPose);
		boolean farEnoughAwayFromFinalPose = currentPose
						.getTranslation()
						.getDistance(getTrajectoryFinalPose(trajectory).getTranslation())
				>= AutoConstants.MIDFIELD_CONTACT_MIN_DISTANCE_TO_FINAL_POSE.in(Units.Meters);

		boolean hitDetected = belowSpeed && farEnoughAwayFromFinalPose && insideNeutralZone;

		SmartDashboard.putBoolean("Collisions/Below Speed", belowSpeed);
		SmartDashboard.putBoolean("Collisions/Far enough from final pose", farEnoughAwayFromFinalPose);
		SmartDashboard.putBoolean("Collisions/In Neutral Zone", insideNeutralZone);
		SmartDashboard.putBoolean("Collisions/ Collision Detected", hitDetected);
		return hitDebouncer.calculate(hitDetected) || simulateCrash;
	}

	public Command driveOverBump(SwerveRequest.FieldCentric request, Rotation2d angle) {
		return Commands.deadline(
				Commands.waitUntil(() -> hasHubFrontTags()),
				Commands.sequence(
								driveUpBump(request, angle),
								driveDownBump(request, angle),
								Commands.parallel(
										Commands.runOnce(
												() -> Cameras.mInstance.setSTDDeviations(AutoConstants.BUMP_STD_DEVS)),
										prepForShot(request, angle)),
								Commands.waitUntil(() -> hasRecentVisionPoseUpdate()))
						.withTimeout(3.0));
	}

	private Command driveUpBump(SwerveRequest.FieldCentric request, Rotation2d angle) {
		return Commands.deadline(
				waitUntilDownwardPitch(),
				Drive.mInstance
						.followSwerveRequestCommand(request, r -> {
							double pidOutput = DriveConstants.CHOREO_THETA_CONTROLLER.calculate(
									Drive.mInstance.getPose().getRotation().getRadians());
							r.withRotationalRate(pidOutput).withVelocityX(overBumpForwardVelocityMetersPerSecond());
							return r;
						})
						.beforeStarting(() -> {
							DriveConstants.CHOREO_THETA_CONTROLLER.reset();
							DriveConstants.CHOREO_THETA_CONTROLLER.setSetpoint(angle.getRadians());
						}));
	}

	private Command driveDownBump(SwerveRequest.FieldCentric request, Rotation2d angle) {
		return Commands.deadline(
				waitUntilFlat(),
				Drive.mInstance
						.followSwerveRequestCommand(request, r -> {
							double pidOutput = DriveConstants.CHOREO_THETA_CONTROLLER.calculate(
									Drive.mInstance.getPose().getRotation().getRadians());
							r.withRotationalRate(pidOutput).withVelocityX(overBumpForwardVelocityMetersPerSecond());
							return r;
						})
						.beforeStarting(() -> {
							DriveConstants.CHOREO_THETA_CONTROLLER.reset();
							DriveConstants.CHOREO_THETA_CONTROLLER.setSetpoint(angle.getRadians());
						}));
	}

	private Command prepForShot(SwerveRequest.FieldCentric request, Rotation2d hubAngle) {
		return Commands.deadline(
				Commands.waitSeconds(0.2),
				Drive.mInstance
						.followSwerveRequestCommand(request, r -> {
							double pidOutput = DriveConstants.CHOREO_THETA_CONTROLLER.calculate(
									Drive.mInstance.getPose().getRotation().getRadians());
							r.withRotationalRate(pidOutput).withVelocityX(overBumpForwardVelocityMetersPerSecond());
							return r;
						})
						.beforeStarting(() -> {
							DriveConstants.CHOREO_THETA_CONTROLLER.reset();
							DriveConstants.CHOREO_THETA_CONTROLLER.setSetpoint(hubAngle.getRadians());
						}));
	}

	private Command waitUntilDownwardPitch() {
		return Commands.waitUntil(() -> {
			boolean downwardPitch = Drive.mInstance
					.getFieldAlignedPigeonAngle()
					.getMeasureY()
					.times((RobotConstants.isRedAlliance) ? 1.0 : -1.0)
					.lte(Units.Degrees.of(-10.0));
			SmartDashboard.putBoolean("Downward Pitch", downwardPitch);
			return downwardPitch;
		});
	}

	private Command waitUntilFlat() {
		return Commands.waitUntil(() -> {
			boolean flat =
					Drive.mInstance.getFieldAlignedPigeonAngle().getMeasureY().gte(Units.Degrees.of(-5.0))
							&& Drive.mInstance
									.getFieldAlignedPigeonAngle()
									.getMeasureY()
									.lte(Units.Degrees.of(5.0));
			SmartDashboard.putBoolean("Flat", flat);
			return flat;
		});
	}

	private static boolean hasRecentVisionPoseUpdate() {
		Time poseAge = Seconds.of(Timer.getFPGATimestamp()).minus(Cameras.mInstance.getLastUpdatedPoseTime());
		return poseAge.lte(MAX_POSE_AGE_FOR_BUMP_CORRECTION);
	}

	private static boolean hasHubFrontTags() {
		Integer[] ids = Cameras.mInstance.getTargetIDs();

		for (int i = 0; i < ids.length; i++) {
			if (ids[i].equals(9) || ids[i].equals(10) || ids[i].equals(25) || ids[i].equals(26)) {
				return true;
			}
		}
		return false;
	}

	private static double overBumpForwardVelocityMetersPerSecond() {
		return overBumpForwardDirectionSign() * AutoConstants.OVER_BUMP_FORWARD_SPEED.in(MetersPerSecond);
	}

	private static double collisionclearvelocity() {
		return overBumpForwardDirectionSign() * AutoConstants.COLLISION_CLEAR_SPEED.in(MetersPerSecond);
	}

	private static double overBumpForwardDirectionSign() {
		return RobotConstants.isRedAlliance ? 1.0 : -1.0;
	}

	public void prepRoutine(Command... steps) {
		routine.active().onTrue(Commands.sequence(steps).withName("Auto Routine Sequential Command Group"));
	}

	public void logTrajectories(AutoTrajectory... trajectories) {
		List<AutoTrajectory> list = Arrays.asList(trajectories);
		for (int i = 1; i <= list.size(); ++i) {
			if (RobotConstants.isRedAlliance) {
				LogUtil.recordTrajectory(
						"Autos/Choreo Path " + i,
						list.get(i - 1).getRawTrajectory().flipped());
			} else {
				LogUtil.recordTrajectory(
						"Autos/Choreo Path " + i, list.get(i - 1).getRawTrajectory());
			}
		}
	}

	public AutoRoutine getRoutine() {
		return routine;
	}

	public Command asCommand() {
		return routine.cmd();
	}
}
