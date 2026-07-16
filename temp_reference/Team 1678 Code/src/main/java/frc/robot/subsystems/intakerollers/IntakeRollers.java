package frc.robot.subsystems.intakerollers;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class IntakeRollers extends MotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint INTAKE = Setpoint.withVoltageSetpoint(IntakeRollersConstants.kIntakeVoltage);
	public static final Setpoint OUTTAKE = Setpoint.withVoltageSetpoint(IntakeRollersConstants.kOuttakeVoltage);

	public static final IntakeRollers mInstance = new IntakeRollers();

	public IntakeRollers() {
		super(IntakeRollersConstants.getMotorIO(), "Intake Rollers");
		// applySetpoint(IDLE);
	}
}
