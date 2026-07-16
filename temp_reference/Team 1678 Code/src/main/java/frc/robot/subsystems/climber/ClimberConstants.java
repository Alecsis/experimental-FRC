package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import frc.lib.bases.ServoMotorSubsystem.ServoHomingConfig;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.LinearSim;
import frc.lib.sim.LinearSim.LinearSimConstants;
import frc.lib.util.Util.DistanceAngleConverter;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class ClimberConstants {
	public static final double kGearing = (44.0 / 8.0) * (84.0 / 14.0) * (1.0 / 1.0) * (1.0 / 1.0);

	public static final Distance kRadius = Units.Inches.of(1.757).div(2.0);

	public static final DistanceAngleConverter converter = new DistanceAngleConverter(kRadius);

	public static final Distance kStowPosition = Units.Inches.of(0.0);
	public static final Distance kClearPosition = kStowPosition.plus(Units.Inches.of(2.8));
	public static final Distance kMaxPosition = Units.Inches.of(RobotConstants.isEpsilon ? 8.5 : 8.25);
	public static final Distance kClimbPosition = Units.Inches.of(RobotConstants.isEpsilon ? 3.0 : 2.0);
	public static final Distance kTrenchPosition = Units.Inches.of(RobotConstants.isEpsilon ? 4.5 : 3.5);
	public static final Distance kJogDelta = Units.Inches.of(RobotConstants.isEpsilon ? 0.25 : 0.25);

	public static final Distance kEpsilonThreshold = Units.Inches.of(0.2);

	public static final Current LOW_CURRENT = Amps.of(30.0);

	public static final TalonFXConfiguration getFXConfig() {
		return new TalonFXConfiguration()
				.withSlot0(new Slot0Configs()
						.withKP(150.0)
						.withKI(0.0)
						.withKD(0.0)
						.withKG(0.25)
						.withKS(0.75)
						.withGravityType(GravityTypeValue.Elevator_Static))
				.withCurrentLimits(new CurrentLimitsConfigs()
						.withStatorCurrentLimitEnable(Robot.isReal())
						.withStatorCurrentLimit(40.0)
						.withSupplyCurrentLimitEnable(Robot.isReal())
						.withSupplyCurrentLimit(25.0)
						.withSupplyCurrentLowerLimit(25.0)
						.withSupplyCurrentLowerTime(0.1))
				.withVoltage(new VoltageConfigs().withPeakForwardVoltage(12.0).withPeakReverseVoltage(-12.0))
				.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
						.withForwardSoftLimitEnable(false)
						.withForwardSoftLimitThreshold(
								converter.toAngle(kMaxPosition).in(Units.Rotations))
						.withReverseSoftLimitEnable(false)
						.withReverseSoftLimitThreshold(
								converter.toAngle(kStowPosition).in(Units.Rotations)))
				.withMotorOutput(new MotorOutputConfigs()
						.withInverted(InvertedValue.CounterClockwise_Positive)
						.withNeutralMode(NeutralModeValue.Brake))
				.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(kGearing));
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.unit = converter.getDistanceUnitAsAngleUnit(Units.Inches);
		config.time = Units.Second;
		config.mainBus = Ports.CLIMBER.bus;
		config.mainID = Ports.CLIMBER.id;
		return config;
	}

	public static LinearSimConstants getSimConstants() {
		LinearSimConstants constants = new LinearSimConstants();
		constants.gearing = kGearing;
		constants.minHeight = kStowPosition;
		constants.maxHeight = kMaxPosition;
		constants.simGravity = false;
		constants.startingHeight = kStowPosition;
		constants.motor = DCMotor.getKrakenX60Foc(1);
		constants.carriageMass = Units.Pounds.of(8.0);
		constants.converter = converter;
		return constants;
	}

	public static ServoHomingConfig getServoConfig() {
		ServoHomingConfig servoConfig = new ServoHomingConfig();
		servoConfig.kHomingTimeout = Units.Seconds.of(0.5);
		servoConfig.kHomingVoltage = Units.Volts.of(-0.75); // includes direction
		servoConfig.kSetHomedVelocity = Units.DegreesPerSecond.of(10.0);
		servoConfig.kHomePosition = converter.toAngle(kStowPosition);
		return servoConfig;
	}

	public static MotorIOTalonFX getMotorIO() {
		if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		} else {
			return new MotorIOTalonFXSim(getIOConfig(), new LinearSim(getSimConstants()));
		}
	}
}
