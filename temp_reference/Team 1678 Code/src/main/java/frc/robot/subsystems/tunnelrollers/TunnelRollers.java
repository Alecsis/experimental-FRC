package frc.robot.subsystems.tunnelrollers;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import frc.lib.bases.FlywheelMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class TunnelRollers extends FlywheelMotorSubsystem<MotorIOTalonFX> {

	public static final TunnelRollers mInstance = new TunnelRollers();

	public static final Setpoint UP = Setpoint.withVelocitySetpoint(TunnelRollerConstants.upSpeed);
	public static final Setpoint DOWN = Setpoint.withVelocitySetpoint(TunnelRollerConstants.downSpeed);
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint SLOW = Setpoint.withVelocitySetpoint(TunnelRollerConstants.slowSpeed);
	public static final Setpoint PRELOAD_SPEED = Setpoint.withVelocitySetpoint(TunnelRollerConstants.preloadSpeed);

	private TunnelRollers() {
		super(TunnelRollerConstants.getMotorIO(), "Tunnel Rollers", RPM.of(130.0), Seconds.of(0.04), true);
	}
}
