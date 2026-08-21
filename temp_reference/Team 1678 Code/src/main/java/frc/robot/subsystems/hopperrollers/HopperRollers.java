package frc.robot.subsystems.hopperrollers;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class HopperRollers extends MotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint INTAKE = Setpoint.withVoltageSetpoint(HopperRollersConstants.kIntakeVoltage);
	public static final Setpoint SLOW_INTAKE = Setpoint.withVoltageSetpoint(HopperRollersConstants.kSlowIntakeVoltage);
	public static final Setpoint OUTTAKE = Setpoint.withVoltageSetpoint(HopperRollersConstants.kOuttakeVoltage);
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint PRELOAD_SPEED = Setpoint.withVoltageSetpoint(HopperRollersConstants.kPreloadSpeed);

	public static final HopperRollers mInstance = new HopperRollers();

	private HopperRollers() {
		super(HopperRollersConstants.getMotorIO(), "Hopper Rollers");
	}
}
