package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class ShooterConstants {

	public static final double kGearing = RobotConstants.isEpsilon
			? (12.0 / 12.0) * (24.0 / 18.0) * (1.0 / 1.0) * (1.0 / 1.0)
			: (12.0 / 12.0) * (18.0 / 18.0) * (1.0 / 1.0) * (1.0 / 1.0);

	public static final AngularVelocity kEpsilonThreshold = RPM.of(10.0); // TODO: Change Later

	public static final Time VELOCITY_THRESHOLD_DEBOUNCE_TIME = Seconds.of(0.0);

	public static final AngularVelocity kTestShot = RPM.of(4000.0); // TODO: Change Later
	public static final AngularVelocity kFerry = RPM.of(3000.0); // TODO: Change Later
	public static final AngularVelocity kSlow = RPM.of(2100.0); // TODO: Change Later

	public static TalonFXConfiguration ShooterFXConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.Feedback.SensorToMechanismRatio = kGearing;
		config.Slot1.kP = RobotConstants.isEpsilon ? 0.8 : 0.5;
		config.Slot1.kI = 0.0;
		config.Slot1.kD = 0.0;
		config.Slot1.kV = RobotConstants.isEpsilon
				? ((12.0) / (6000.0 / 60.0) * kGearing) * 0.995
				: ((12.0) / (6000.0 / 60.0) * kGearing * 0.955);
		config.Slot1.kS = RobotConstants.isEpsilon ? 0.32 : 0.19;
		config.Slot0.kP = 0.0;
		config.Slot0.kI = 0.0;
		config.Slot0.kD = 0.0;
		config.Slot0.kV = 0.0;
		config.Slot0.kS = 0.0;

		config.Slot2.kP = 0.0;
		config.Slot2.kI = 0.0;
		config.Slot2.kD = 0.0;
		config.Slot2.kV = 0.0;
		config.Slot2.kS = 0.0;

		config.CurrentLimits.SupplyCurrentLimit = 30.0; // TODO: Change Later
		config.CurrentLimits.SupplyCurrentLowerLimit = 30.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 1.0;
		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();

		config.CurrentLimits.StatorCurrentLimit = 80.0; // TODO: Change Later
		config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();

		config.Voltage.PeakForwardVoltage = 12.0; // TODO: Change Later
		config.Voltage.PeakReverseVoltage = 0.0; // TODO: Change Later
		config.TorqueCurrent.PeakForwardTorqueCurrent = 1000.0; // TODO: Change Later
		config.TorqueCurrent.PeakReverseTorqueCurrent = -1000.0; // TODO: Change Later
		config.TorqueCurrent.TorqueNeutralDeadband = 0.5; // TODO: Change Later

		config.MotionMagic.MotionMagicAcceleration = 200.0;
		config.MotionMagic.MotionMagicJerk = 400.0;

		config.Feedback.VelocityFilterTimeConstant = 0.1;

		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		return config;
	}

	public static MotorIOTalonFXConfig getIOconfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainConfig = ShooterFXConfig();
		config.followerConfig = ShooterFXConfig();
		config.time = Units.Minutes;
		config.unit = Units.Rotations;
		config.mainID = Ports.SHOOTER_MAIN.id;
		config.mainBus = Ports.SHOOTER_MAIN.bus;
		config.followerAlignment = new MotorAlignmentValue[] {
			MotorAlignmentValue.Aligned, MotorAlignmentValue.Opposed, MotorAlignmentValue.Opposed
		};
		config.followerIDs =
				new int[] {Ports.SHOOTER_FOLLOWER_1.id, Ports.SHOOTER_FOLLOWER_2.id, Ports.SHOOTER_FOLLOWER_3.id};
		config.followerBuses =
				new CANBus[] {Ports.SHOOTER_FOLLOWER_1.bus, Ports.SHOOTER_FOLLOWER_2.bus, Ports.SHOOTER_FOLLOWER_3.bus};

		return config;
	}

	public static MotorIOTalonFX getMotorIO() {
		if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOconfig());
		} else {
			return new MotorIOTalonFXSim(getIOconfig(), new RollerSim(getSimConstants()));
		}
	}

	public static RollerSimConstants getSimConstants() {
		RollerSimConstants simConstants = new RollerSimConstants();
		simConstants.motor = DCMotor.getKrakenX60(4);
		simConstants.gearing = kGearing;
		simConstants.momentOfInertia = 0.0166570492;
		return simConstants;
	}
}
