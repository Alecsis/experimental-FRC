package frc.lib.sim.field.rebuilt;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout;
import frc.lib.util.MathHelpers;
import frc.robot.RobotConstants;
import frc.robot.shooting.Shooting;
import frc.robot.shooting.Shooting.ShootingState;
import frc.robot.shooting.regressions.DistanceToAirTimeRegressions;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

/**
 * Class that visualizes shot trajectories.
 * Can compute a parabolic trajectory for a given launch speed
 * and angle and also compute a regression-based launch using precomputed
 * distance-to-air-time mappings.
 */
public class ShotVisualizer {
	public static final ShotVisualizer mInstance = new ShotVisualizer(
			() -> {
				return new Pose3d(Drive.mInstance.getPose()).plus(FieldSimConstants.kShooterOffset);
			},
			() -> {
				return Drive.mInstance.getState().Speeds;
			});

	private Translation3d[] trajectory = new Translation3d[50];
	private Supplier<Pose3d> poseSupplier;
	private Supplier<ChassisSpeeds> fieldSpeedsSupplier;

	public ShotVisualizer(Supplier<Pose3d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
		this.poseSupplier = poseSupplier;
		this.fieldSpeedsSupplier = fieldSpeedsSupplier;
	}

	private Translation3d launchVel(LinearVelocity vel, Angle angle) {
		Pose3d robot = poseSupplier.get();
		ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

		double horizontalVel = Math.cos(angle.in(Radians)) * vel.in(MetersPerSecond);
		double verticalVel = Math.sin(angle.in(Radians)) * vel.in(MetersPerSecond);
		double xVel =
				horizontalVel * Math.cos(robot.getRotation().toRotation2d().getRadians());
		double yVel =
				horizontalVel * Math.sin(robot.getRotation().toRotation2d().getRadians());

		xVel += fieldSpeeds.vxMetersPerSecond;
		yVel += fieldSpeeds.vyMetersPerSecond;

		return new Translation3d(xVel, yVel, verticalVel);
	}

	/**
	 * Spawn a single simulated fuel using the provided launch parameters.
	 *
	 * @param vel   launch speed magnitude
	 * @param angle launch angle
	 */
	public void launchFuel(LinearVelocity vel, Angle angle) {
		Pose3d robot = poseSupplier.get();
		Translation3d initialPosition = robot.getTranslation();
		FieldSim.mInstance.generateFuel(initialPosition, launchVel(vel, angle));
	}

	/**
	 * Compute and log a parabolic trajectory for visualization.
	 *
	 * @param vel   launch speed magnitude
	 * @param angle launch angle
	 */
	public void updateFuel(LinearVelocity vel, Angle angle) {
		Translation3d trajVel = launchVel(vel, angle);
		for (int i = 0; i < trajectory.length; i++) {
			// do parabolic arc (x = vxt, y = vyt, z = vzt - 0.5gt^2)
			double t = i * FieldSimConstants.kFuelArcInterval.in(Seconds);
			double x = trajVel.getX() * t + poseSupplier.get().getTranslation().getX();
			double y = trajVel.getY() * t + poseSupplier.get().getTranslation().getY();
			double z = trajVel.getZ() * t
					- 0.5 * FieldSimConstants.kGravity.in(MetersPerSecondPerSecond) * t * t
					+ poseSupplier.get().getTranslation().getZ();

			trajectory[i] = new Translation3d(x, y, z);
		}

		LogUtil.log("Fuel Simulation/Trajectory", trajectory);
	}

	public void update() {
		updateWithRegression();
	}

	/**
	 * Compute and spawn a shot based on distance to airtime regressions and the
	 * current robot pose/state.
	 */
	public void updateWithRegression() {
		Pose3d shooterPose = poseSupplier.get();

		ShootingState state = Shooting.getState();
		Translation2d target = state.getLocationTranslation(RobotConstants.isRedAlliance);

		Time lookAheadTime = Seconds.of(state.getDistanceToFuelAirTimeMap()
				.get(Drive.mInstance.getPose().getTranslation().getDistance(target)));

		var lookAheadPose =
				Drive.mInstance.getLookAheadPoseWithoutHeading(lookAheadTime.minus(state.getLookAheadTimeOffset()));

		Distance distanceToHub = Meters.of(lookAheadPose.getTranslation().getDistance(target));

		Time dt = DistanceToAirTimeRegressions.getHubShotTime(distanceToHub);

		double dx = distanceToHub.in(Meters);
		double dtSec = dt.in(Seconds);

		double hubZ = FieldLayout.kHubAltitude.in(Units.Meters);
		double dz = hubZ - shooterPose.getTranslation().getZ();

		double vxReq = dx / dtSec;
		double vyReq = (dz + 0.5 * FieldSimConstants.kGravity.in(MetersPerSecondPerSecond) * dtSec * dtSec) / dtSec;

		double vMag = Math.hypot(vxReq, vyReq);
		double angleRad = Math.atan2(vyReq, vxReq);

		updateFuel(MetersPerSecond.of(vMag), Radians.of(angleRad));
	}

	/**
	 * Solve for a combined velocity magnitude given horizontal distance and
	 * an angle.
	 * @param dx    horizontal distance
	 * @param dt    time of flight
	 * @param theta launch angle
	 * @return combined velocity magnitude
	 */
	public LinearVelocity solveForYVelo(Distance dx, Time dt, Angle theta) {
		LinearVelocity vX = dx.div(dt);
		LinearVelocity vY = vX.times(MathHelpers.tan(theta));
		return Units.MetersPerSecond.of(Math.hypot(vX.in(MetersPerSecond), vY.in(MetersPerSecond)));
	}

	public Distance getDistanceToHub(Pose2d drivePose) {
		return MathHelpers.hypot(FieldLayout.handleAllianceFlip(FieldLayout.kBlueHub, RobotConstants.isRedAlliance)
				.minus(drivePose.getTranslation()));
	}

	/**
	 * Compute the required launch speed and angle for the current shooter/drive
	 * state using the distance-to-air-time regressions.
	 *
	 * @return Pair<v, angle> where v is the LinearVelocity magnitude and angle is the launch Angle
	 */
	private Pair<LinearVelocity, Angle> computeRegressionVelocityAnglePair() {
		Pose3d shooterPose = poseSupplier.get();

		var state = Shooting.getState();
		var target = state.getLocationTranslation(RobotConstants.isRedAlliance);

		Time lookAheadTime = Seconds.of(state.getDistanceToFuelAirTimeMap()
				.get(Drive.mInstance.getPose().getTranslation().getDistance(target)));

		var lookAheadPose =
				Drive.mInstance.getLookAheadPoseWithoutHeading(lookAheadTime.minus(state.getLookAheadTimeOffset()));

		Distance distanceToHub = Meters.of(lookAheadPose.getTranslation().getDistance(target));

		Time dt = DistanceToAirTimeRegressions.getHubShotTime(distanceToHub);

		double dx = distanceToHub.in(Meters);
		double dtSec = dt.in(Units.Seconds);

		double hubZ = FieldLayout.kHubAltitude.in(Units.Meters); // meters
		double dz = hubZ - shooterPose.getTranslation().getZ();

		double vxReq = dx / dtSec;
		double vyReq = (dz + 0.5 * FieldSimConstants.kGravity.in(MetersPerSecondPerSecond) * dtSec * dtSec) / dtSec;

		double vMag = Math.hypot(vxReq, vyReq);
		double angleRad = Math.atan2(vyReq, vxReq);

		return Pair.of(Units.MetersPerSecond.of(vMag), Radians.of(angleRad));
	}

	/**
	 * Create a command that launches a single fuel using provided suppliers.
	 * The command fires once, waits briefly, then repeats while scheduled.
	 */
	public Command continuousLaunchFuelCommand(Supplier<LinearVelocity> velSupplier, Supplier<Angle> angleSupplier) {
		return Commands.runOnce(() -> launchFuel(velSupplier.get(), angleSupplier.get()))
				.andThen(Commands.waitTime(FieldSimConstants.kShootingWait))
				.repeatedly();
	}

	/**
	 * Create a continuous command that recomputes the regression-derived
	 * launch parameters each invocation.
	 */
	public Command continuousFuelStreamWithRegressionCommand() {
		return continuousLaunchFuelCommand(
				() -> computeRegressionVelocityAnglePair().getFirst(),
				() -> computeRegressionVelocityAnglePair().getSecond());
	}
}
