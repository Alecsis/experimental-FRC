package frc.robot.shooting.setpoints;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.RobotConstants;
import frc.robot.shooting.Shooting;
import frc.robot.shooting.Shooting.ShootingState;
import frc.robot.subsystems.drive.Drive;

public class ShootingSetpoints {

	private Setpoint shooterSetpoint = Setpoint.withCoastSetpoint();
	private Setpoint hoodSetpoint = Setpoint.withNeutralSetpoint();

	public ShootingSetpoints() {}

	public static Time getLookAheadTime(ShootingState state) {
		Translation2d target = state.getLocationTranslation(RobotConstants.isRedAlliance);
		Translation2d drive = Drive.mInstance.getPose().getTranslation();
		double distanceMeters = drive.getDistance(target);
		return state.tof(Meters.of(distanceMeters));
	}

	public Distance getTargetLookAheadDistance(ShootingState state) {
		return Meters.of(state.getLocationTranslation(RobotConstants.isRedAlliance)
				.getDistance(Shooting.m_lookAheadTracker.getBaseTranslation()));
	}

	public Setpoint getShooterSetpoint() {
		ShootingState state = Shooting.getState();
		Distance lookAheadDistance = getTargetLookAheadDistance(state);
		return Setpoint.withVelocitySetpoint(state.getShootingRegressions().getShooterVelocity(lookAheadDistance));
	}

	public Setpoint getHoodSetpoint() {
		ShootingState state = Shooting.getState();
		Distance lookAheadDistance = getTargetLookAheadDistance(state);
		return Setpoint.withMotionMagicSetpoint(state.getShootingRegressions().getHoodAngle(lookAheadDistance));
	}

	public Setpoint getTunnelSetpoint() {
		ShootingState state = Shooting.getState();
		Distance lookAheadDistance = getTargetLookAheadDistance(state);
		return Setpoint.withVelocitySetpoint(state.getShootingRegressions().getTunnelVelocity(lookAheadDistance));
	}
}
