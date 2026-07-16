package frc.robot.subsystems.hopperrollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class HopperRollersConstants {
	public static final double kGearing = RobotConstants.isEpsilon
			? (18.0 / 18.0) * (1.0 / 1.0) * (1.0 / 1.0) * (1.0 / 1.0)
			: (36.0 / 12.0) * (12.0 / 24.0) * (1.0 / 1.0) * (1.0 / 1.0) * (1.0 / 1.0);

	public static final Voltage kIntakeVoltage = Volts.of(12.0); // change later
	public static final Voltage kSlowIntakeVoltage = Volts.of(3.0); // change later
	public static final Voltage kOuttakeVoltage = Volts.of(-12.0); // change later
	public static final Voltage kIdleVoltage = Volts.of(0.0); // change later
	public static final Voltage kPreloadSpeed = Volts.of(3.0); // change later

	public static final Current kCurrentThreshold = Amps.of(50); // change later

	public static final TalonFXConfiguration hopperRollersConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.StatorCurrentLimit = 90.0; // AMPS, change later

		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.SupplyCurrentLimit = 45.0; // AMPS, change later
		config.CurrentLimits.SupplyCurrentLowerLimit = 45.0; // AMPS, change later
		config.CurrentLimits.SupplyCurrentLowerTime = 1.0; // change later

		config.Feedback.SensorToMechanismRatio = kGearing;

		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		return config;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainID = Ports.HOPPER_ROLLERS.id;
		config.mainBus = Ports.HOPPER_ROLLERS.bus;
		config.mainConfig = hopperRollersConfig();
		config.time = Units.Seconds;
		config.unit = Units.Degrees;

		return config;
	}

	public static RollerSimConstants getSimConstants() {
		RollerSimConstants constants = new RollerSimConstants();
		constants.gearing = kGearing;
		constants.motor = DCMotor.getKrakenX60(1);
		constants.momentOfInertia = 0.0000175584;
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
