package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.bases.ServoMotorSubsystem.ServoHomingConfig;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.PivotSim.PivotSimConstants;
import frc.lib.sim.PivotSimHelper;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class HoodConstants {

	public static final double kGearing = RobotConstants.isEpsilon
			? (48.0 / 8.0) * (18.0 / 15.0) * (175.0 / 10.0)
			: (48.0 / 10.0) * (18.0 / 15.0) * (177.0 / 10.0);
	public static final Angle kEpsilonThreshold = Units.Degrees.of(0.5);

	public static final Angle kMaxAngle = Units.Degrees.of(52.5);
	public static final Angle kMinAngle = Units.Degrees.of(17.5);
	public static final Angle kTestAngle = Units.Degrees.of(50.0);
	public static final Angle kTest2Angle = Units.Degrees.of(35.0);
	public static final Voltage kSimStaticFrictionVoltage = Units.Volts.of(0.0);
	public static final Voltage kSimKineticFrictionVoltage = Units.Volts.of(0.0);
	public static final AngularVelocity kSimStictionVelocity = Units.DegreesPerSecond.of(2.0);
	public static final double kSimViscousFrictionVoltsPerRadPerSec = 0.38;

	public static final TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = RobotConstants.isEpsilon ? 440.0 : 500.0; // Position
		FXConfig.Slot0.kD = 0.0; // Derivative
		FXConfig.Slot0.kS = RobotConstants.isEpsilon ? 0.45 : 0.48; // Feedforwards

		FXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		FXConfig.CurrentLimits.SupplyCurrentLimit = 25.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 25.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;

		FXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
		FXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kMaxAngle.in(Units.Rotations);

		FXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
		FXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kMinAngle.in(Units.Rotations);

		FXConfig.Feedback.SensorToMechanismRatio = kGearing;

		FXConfig.MotionMagic.MotionMagicCruiseVelocity = 40.0;

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		FXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getMotorIOTalonFXConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.mainID = Ports.HOOD.id;
		config.mainBus = Ports.HOOD.bus;
		config.unit = Units.Degrees;
		config.time = Units.Second;
		return config;
	}

	public static PivotSimConstants getSimConstants() {
		PivotSimConstants constants = new PivotSimConstants();
		constants.motor = DCMotor.getKrakenX44(1);
		constants.armLength = Units.Meters.of(0.259);
		constants.gearing = kGearing;
		constants.momentOfInertia = Units.KilogramSquareMeters.of(0.0614835912);
		constants.simGravity = false;
		constants.mechanismStartPos = kMinAngle;
		constants.mechanismMaxHardStop = kMaxAngle;
		constants.mechanismMinHardStop = kMinAngle;
		return constants;
	}

	public static MotorIOTalonFX getMotorIO() {
		if (Robot.isReal()) {
			return new MotorIOTalonFX(getMotorIOTalonFXConfig());
		} else {
			return new MotorIOTalonFXSim(
					getMotorIOTalonFXConfig(),
					new PivotSimHelper(
							getSimConstants(),
							kSimStaticFrictionVoltage,
							kSimKineticFrictionVoltage,
							kSimStictionVelocity,
							kSimViscousFrictionVoltsPerRadPerSec));
		}
	}

	public static ServoHomingConfig getServoConfig() {
		ServoHomingConfig servoConfig = new ServoHomingConfig();
		servoConfig.kHomingTimeout = Units.Seconds.of(0.5);
		servoConfig.kHomingVoltage = Units.Volts.of(-1.5); // includes direction
		servoConfig.kSetHomedVelocity = Units.DegreesPerSecond.of(5.0);
		servoConfig.kHomePosition = kMinAngle;
		return servoConfig;
	}
}
