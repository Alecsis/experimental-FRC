package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Units;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class Climber extends ServoMotorSubsystem<MotorIOTalonFX> {

	public static final Setpoint MAX = Setpoint.withMotionMagicSetpointAndCurrentLimit(
			ClimberConstants.converter.toAngle(ClimberConstants.kMaxPosition),
			ClimberConstants.LOW_CURRENT,
			ClimberConstants.LOW_CURRENT);
	public static final Setpoint STOW = Setpoint.withMotionMagicSetpointAndCurrentLimit(
			ClimberConstants.converter.toAngle(ClimberConstants.kStowPosition), Amps.of(90.0), Amps.of(90.0));
	public static final Setpoint CLIMB = Setpoint.withMotionMagicSetpointAndCurrentLimit(
			ClimberConstants.converter.toAngle(ClimberConstants.kClimbPosition), Amps.of(90.0), Amps.of(90.0));
	public static final Setpoint TRENCH = Setpoint.withMotionMagicSetpointAndCurrentLimit(
			ClimberConstants.converter.toAngle(ClimberConstants.kTrenchPosition), Amps.of(90.0), Amps.of(90.0));
	public static final Setpoint DOWN = Setpoint.withVoltageSetpoint(Volts.of(-12.0));
	public static final Setpoint JOG_UP = Setpoint.withVoltageSetpoint(Units.Volts.of(3.0));
	public static final Setpoint CLEAR_INTAKE =
			Setpoint.withMotionMagicSetpoint(ClimberConstants.converter.toAngle(ClimberConstants.kClearPosition));
	public static final Setpoint JOG_DOWN = Setpoint.withVoltageSetpoint(Units.Volts.of(-3.0));

	public static final Climber mInstance = new Climber();

	private Climber() {
		super(
				ClimberConstants.getMotorIO(),
				"Climber",
				ClimberConstants.converter.toAngle(ClimberConstants.kEpsilonThreshold),
				true,
				ClimberConstants.getServoConfig());
		setCurrentPosition(ClimberConstants.converter.toAngle(ClimberConstants.kStowPosition));
	}
}
