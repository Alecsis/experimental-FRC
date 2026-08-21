package frc.robot.subsystems.feederrollers;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;

public class FeederRollersConstants {
	public static final double kGearing = (30.0 / 12.0);
	public static final Voltage kFeedVoltage = Volts.of(12.0); // change later
	public static final Voltage kSlowFeedVoltage = Volts.of(3.0); // change later
	public static final Voltage kIdleVoltage = Volts.of(0.0); // change later
	public static final Voltage kReverseVoltage = Volts.of(-4.0); // change later

	public static final AngularVelocity kfeed = RPM.of(1700.0); // change later
	public static final AngularVelocity kIdle = RPM.of(0.0); // change later
	public static final AngularVelocity kReverse = RPM.of(-1000); // change later
	public static final AngularVelocity kSlowReverse = RPM.of(-650.0);

	public static final TalonFXConfiguration feederRollersConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.Feedback.SensorToMechanismRatio = kGearing;
		config.Slot1.kP = 0.9;
		config.Slot1.kI = 0.0;
		config.Slot1.kD = 0.0;
		config.Slot1.kV = ((12.0) / (5800.0 / 60.0) * kGearing * 0.98);
		config.Slot1.kS = 0.26;

		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = 50.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 25.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 25.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 1.0;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.MotionMagic.MotionMagicAcceleration = 200.0;
		config.MotionMagic.MotionMagicJerk = 400.0;

		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		return config;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainID = Ports.FEEDER_ROLLERS.id;
		config.mainBus = Ports.FEEDER_ROLLERS.bus;
		config.mainConfig = feederRollersConfig();
		config.time = Units.Minutes;
		config.unit = Units.Rotations;

		return config;
	}

	public static RollerSimConstants getSimConstants() {
		RollerSimConstants constants = new RollerSimConstants();
		constants.gearing = kGearing;
		constants.motor = DCMotor.getKrakenX60(1);
		constants.momentOfInertia = 0.0002569376;
		return constants;
	}

	public static final MotorIOTalonFX getMotorIO() {
		if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		} else {
			return new MotorIOTalonFXSim(getIOConfig(), new RollerSim(getSimConstants()));
		}
	}
}
