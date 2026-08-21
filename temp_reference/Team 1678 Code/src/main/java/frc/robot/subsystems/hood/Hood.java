package frc.robot.subsystems.hood;

import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class Hood extends ServoMotorSubsystem<MotorIOTalonFX> {

	public static final Hood mInstance = new Hood();

	public static final Setpoint STOW = Setpoint.withMotionMagicSetpoint(HoodConstants.kMinAngle);
	public static final Setpoint MAX_ANGLE = Setpoint.withMotionMagicSetpoint(HoodConstants.kMaxAngle);
	public static final Setpoint TEST = Setpoint.withMotionMagicSetpoint(HoodConstants.kTestAngle);
	public static final Setpoint TEST2 = Setpoint.withMotionMagicSetpoint(HoodConstants.kTest2Angle);

	private Hood() {
		super(HoodConstants.getMotorIO(), "Hood", HoodConstants.kEpsilonThreshold, HoodConstants.getServoConfig());
		setCurrentPosition(HoodConstants.kMinAngle);
	}
}
