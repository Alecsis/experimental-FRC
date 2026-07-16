package frc.robot.shooting.shootingplanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.logging.LogUtil;
import frc.lib.math.ChassisAccelerations;
import frc.robot.shooting.Shooting.ShootingState;

/**
 * @param state                   Current shooting state (hub, ferry)
 * @param driveSpeeds             Current robot chassis velocities (vx, vy, omega)
 * @param drivePose               Current pose of the robot on the field
 * @param driveAcceleration       Current robot chassis accelerations (ax, ay, alpha)
 * @param dt                      Duration of the integration step in seconds
 */
public record MotionCompensatedShootingInputs(
		ShootingState state,
		ChassisSpeeds driveSpeeds,
		Pose2d drivePose,
		ChassisAccelerations driveAccelerations,
		double dt) {

	public void telemetrize() {
		SmartDashboard.putString("Drive Tracker/Input/State", state.name());
		LogUtil.log("Drive Tracker/Input/Speeds", driveSpeeds);
		LogUtil.log("Drive Tracker/Input/Pose", drivePose);
		SmartDashboard.putNumber(
				"Drive Tracker/Input/Acceleration/X Meters per Second Per Second", driveAccelerations.m_ax);
		SmartDashboard.putNumber(
				"Drive Tracker/Input/Acceleration/Y Meters per Second Per Second", driveAccelerations.m_ay);
		SmartDashboard.putNumber(
				"Drive Tracker/Input/Acceleration/O Radians per Second per Second", driveAccelerations.m_alpha);
		SmartDashboard.putNumber("Drive Tracker/Input/Delta Time Seconds", dt);
	}

	public static MotionCompensatedShootingInputs zero() {
		return new MotionCompensatedShootingInputs(
				ShootingState.HUB, new ChassisSpeeds(), Pose2d.kZero, new ChassisAccelerations(), 0.0);
	}
}
