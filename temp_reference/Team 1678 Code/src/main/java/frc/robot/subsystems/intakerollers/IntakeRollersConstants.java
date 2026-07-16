package frc.robot.subsystems.intakerollers;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;

public class IntakeRollersConstants {
	private static final double kGearing = (30.0 / 16.0) * (1.0 / 1.0) * (1.0 / 1.0) * (1.0 / 1.0);

	public static final Voltage kIdleVoltage = Units.Volts.of(0.0);
	public static final Voltage kIntakeVoltage = Units.Volts.of(12.0);
	public static final Voltage kOuttakeVoltage = Units.Volts.of(-12.0);

	public static TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.StatorCurrentLimit = 80.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.SupplyCurrentLimit = 50.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 50.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.Feedback.SensorToMechanismRatio = kGearing;

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		return config;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.unit = Units.Rotations;
		config.time = Units.Minutes;
		config.mainID = Ports.INTAKE_ROLLERS_MAIN.id;
		config.mainBus = Ports.INTAKE_ROLLERS_MAIN.bus;
		config.mainConfig = getFXConfig();
		config.followerConfig = getFXConfig();
		config.followerAlignment = new MotorAlignmentValue[] {MotorAlignmentValue.Opposed};
		config.followerIDs = new int[] {Ports.INTAKE_ROLLERS_FOLLOWER.id};
		config.followerBuses = new CANBus[] {Ports.INTAKE_ROLLERS_FOLLOWER.bus};
		return config;
	}

	public static MotorIOTalonFX getMotorIO() {
		if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		} else {
			return new MotorIOTalonFXSim(getIOConfig(), new RollerSim(getSimConstants()));
		}
	}

	public static RollerSimConstants getSimConstants() {
		RollerSimConstants simConstants = new RollerSimConstants();

		simConstants.motor = DCMotor.getKrakenX60(1);
		simConstants.gearing = kGearing;
		simConstants.momentOfInertia = 0.0001618297;

		return simConstants;
	}
}
