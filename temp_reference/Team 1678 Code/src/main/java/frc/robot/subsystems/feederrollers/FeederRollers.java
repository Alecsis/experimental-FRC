package frc.robot.subsystems.feederrollers;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class FeederRollers extends MotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint FEED_VOLTAGE = Setpoint.withVoltageSetpoint(FeederRollersConstants.kFeedVoltage);
	public static final Setpoint SLOW_FEED_VOLTAGE =
			Setpoint.withVoltageSetpoint(FeederRollersConstants.kSlowFeedVoltage);
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint REVERSE = Setpoint.withVelocitySetpoint(FeederRollersConstants.kReverse);
	public static final Setpoint SLOW_REVERSE = Setpoint.withVoltageSetpoint(FeederRollersConstants.kReverseVoltage);

	public static final FeederRollers mInstance = new FeederRollers();

	private FeederRollers() {
		super(FeederRollersConstants.getMotorIO(), "Feeder Rollers");
	}
}
