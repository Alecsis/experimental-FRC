package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import frc.lib.bases.FlywheelMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class Shooter extends FlywheelMotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint TESTSHOT = Setpoint.withVelocitySetpoint(ShooterConstants.kTestShot, 1);
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint FERRY = Setpoint.withVelocitySetpoint(ShooterConstants.kFerry, 1);
	public static final Setpoint SLOW = Setpoint.withVelocitySetpoint(ShooterConstants.kSlow);

	public static final Shooter mInstance = new Shooter();

	public Shooter() {
		super(
				ShooterConstants.getMotorIO(),
				"Shooter",
				RPM.of(80.0),
				ShooterConstants.VELOCITY_THRESHOLD_DEBOUNCE_TIME,
				true);

		// applySetpoint(SLOW);
	}
}
