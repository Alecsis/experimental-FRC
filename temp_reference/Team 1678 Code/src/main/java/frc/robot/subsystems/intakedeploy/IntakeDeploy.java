package frc.robot.subsystems.intakedeploy;

import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class IntakeDeploy extends ServoMotorSubsystem<MotorIOTalonFX> {

	public static final Setpoint STOWED_SETPOINT = Setpoint.withPositionSetpoint(IntakeDeployConstants.kStowedAngle);
	public static final Setpoint DEPLOYED_SETPOINT =
			Setpoint.withPositionSetpoint(IntakeDeployConstants.kDeployedAngle);
	public static final Setpoint PARTIAL_IN = Setpoint.withPositionSetpoint(IntakeDeployConstants.kPartialIn);
	public static final Setpoint MORE_PARTIAL_IN = Setpoint.withPositionSetpoint(IntakeDeployConstants.kMorePartialIn);
	public static final Setpoint LESS_PARTIAL_IN = Setpoint.withPositionSetpoint(IntakeDeployConstants.kLessPartialIn);

	public static final IntakeDeploy mInstance = new IntakeDeploy();

	private IntakeDeploy() {
		super(
				IntakeDeployConstants.getMotorIO(), "Intake Deploy", IntakeDeployConstants.kEpsilonThreshold
				// IntakeDeployConstants.getServoHomingConfig()
				);
		setCurrentPosition(IntakeDeployConstants.kStowedAngle);
		// applySetpoint(STOWED_SETPOINT);
	}
}
