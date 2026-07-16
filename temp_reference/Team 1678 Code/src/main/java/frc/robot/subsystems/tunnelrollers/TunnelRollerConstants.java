package frc.robot.subsystems.tunnelrollers;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
import frc.robot.RobotConstants;

public class TunnelRollerConstants {

	public static final Voltage kUpVoltage = Volts.of(-12.0); // To be determined
	public static final Voltage kDownVoltage = Volts.of(12.0); // To be determined

	public static final AngularVelocity upSpeed = RPM.of(5000); // To be determined
	public static final AngularVelocity downSpeed = RPM.of(3000); // To be determined
	public static final AngularVelocity slowSpeed = RPM.of(1250); // To be determined
	public static final AngularVelocity preloadSpeed = RPM.of(250); // To be determined

	public static final double kGearing = (21.0 / 12.0)
			* (19.0 / 21.0)
			* (19.0 / 21.0)
			* (19.0 / 21.0)
			* (19.0 / 21.0)
			* (19.0 / 21.0); // change later

	public static final TalonFXConfiguration TunnelRollerConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.Feedback.SensorToMechanismRatio = kGearing;
		config.Slot1.kP = RobotConstants.isEpsilon ? 1.75 : 7.5;
		config.Slot1.kI = 0.0;
		config.Slot1.kD = 0.0;
		config.Slot1.kV = RobotConstants.isEpsilon
				? ((12.0) / (6000.0 / 60.0) * kGearing) * 1.02
				: ((12.0) / (6000.0 / 60.0) * kGearing * 0.985);
		config.Slot1.kS = RobotConstants.isEpsilon ? 0.36 : 0.26;

		config.CurrentLimits.SupplyCurrentLimit = 10.0; // TODO: Change Later
		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();

		config.CurrentLimits.StatorCurrentLimit = 60.0; // TODO: Change Later
		config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();

		config.Voltage.PeakForwardVoltage = 12.0; // TODO: Change Later
		config.Voltage.PeakReverseVoltage = 0.0; // TODO: Change Later
		config.TorqueCurrent.PeakForwardTorqueCurrent = 1000.0; // TODO: Change Later
		config.TorqueCurrent.PeakReverseTorqueCurrent = -1000.0; // TODO: Change Later
		config.TorqueCurrent.TorqueNeutralDeadband = 0.5; // TODO: Change Later

		config.Feedback.VelocityFilterTimeConstant = 0.25;

		config.MotionMagic.MotionMagicAcceleration = 800.0;
		config.MotionMagic.MotionMagicJerk = 1200.0;

		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		return config;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainID = Ports.TUNNEL_ROLLERS_MAIN.id;
		config.mainBus = Ports.TUNNEL_ROLLERS_MAIN.bus;
		config.time = Units.Minutes;
		config.unit = Units.Rotations;
		config.mainConfig = TunnelRollerConfig();
		config.followerConfig = TunnelRollerConfig();
		config.followerAlignment = new MotorAlignmentValue[] {MotorAlignmentValue.Opposed};
		config.followerIDs = new int[] {Ports.TUNNEL_ROLLERS_FOLLOWER.id};
		config.followerBuses = new CANBus[] {Ports.TUNNEL_ROLLERS_FOLLOWER.bus};

		return config;
	}

	public static RollerSimConstants getSimConstants() {
		RollerSimConstants constants = new RollerSimConstants();
		constants.gearing = kGearing;
		constants.motor = DCMotor.getKrakenX60(1);
		constants.momentOfInertia = 0.000553089;
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
