package frc.robot.subsystems.intakedeploy;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
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

public class IntakeDeployConstants {
	public static final Angle kDeployedAngle = Units.Degrees.of(0.0);
	public static final Angle kStowedAngle = Units.Degrees.of(130.0);
	public static final Angle kMorePartialIn = Units.Degrees.of(80.0);
	public static final Angle kPartialIn = Units.Degrees.of(50.0);
	public static final Angle kLessPartialIn = Units.Degrees.of(25.0);

	public static final Voltage kSimStaticFrictionVoltage = Units.Volts.of(0.0);
	public static final Voltage kSimKineticFrictionVoltage = Units.Volts.of(0.0);
	public static final AngularVelocity kSimStictionVelocity = Units.DegreesPerSecond.of(1.0);
	public static final double kSimViscousFrictionVoltsPerRadPerSec = 0.38;

	public static final double kGearing = (62.0 / 8.0) * (68.0 / 18.0) * (15.0 / 9.0);

	public static final Distance kArmLength = Units.Inches.of(12.804);

	public static final Distance INTAKE_ARM_TO_CONTACT = Meters.of(0.225980);

	public static final Angle kEpsilonThreshold = Units.Degrees.of(12.0);

	public static final Distance HOPPER_MAX_EXTENSION_DISTANCE =
			Meters.of(RobotConstants.isEpsilon ? 0.303211 : 0.167250);

	public static final Distance HOPPER_FLOOR_ARM_LENGTH = Meters.of(0.323759);

	public static final Angle HOPPER_MIN_COVERING_ANGLE = Degrees.of(100.0);

	public static TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.Slot2.kP = RobotConstants.isEpsilon ? 125.0 : 75.0;
		config.Slot2.kD = 0.0;
		config.Slot2.kS = 0.28;
		config.Slot2.kG = 0.3;

		config.Slot2.GravityType = GravityTypeValue.Arm_Cosine;
		config.Slot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

		config.MotionMagic.MotionMagicCruiseVelocity = 7.0;
		config.MotionMagic.MotionMagicAcceleration = 15.0;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 40.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 25.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.CurrentLimits.StatorCurrentLimit = 60.0;
		config.CurrentLimits.StatorCurrentLimitEnable = true;

		config.Feedback.SensorToMechanismRatio = kGearing;

		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kStowedAngle.in(Rotations);

		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kDeployedAngle.in(Rotations);
		return config;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.mainID = Ports.INTAKE_DEPLOY.id;
		config.mainBus = Ports.INTAKE_DEPLOY.bus;
		config.time = Units.Seconds;
		config.unit = Units.Degrees;
		return config;
	}

	public static MotorIOTalonFX getMotorIO() {
		if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		} else {
			return new MotorIOTalonFXSim(
					getIOConfig(),
					new PivotSimHelper(
							getSimConstants(),
							kSimStaticFrictionVoltage,
							kSimKineticFrictionVoltage,
							kSimStictionVelocity,
							kSimViscousFrictionVoltsPerRadPerSec));
		}
	}

	public static PivotSimConstants getSimConstants() {
		PivotSimConstants simConstants = new PivotSimConstants();
		simConstants.gearing = kGearing;
		simConstants.armLength = kArmLength;
		simConstants.momentOfInertia = Units.KilogramSquareMeters.of(0.2224061366);
		simConstants.motor = DCMotor.getKrakenX44Foc(1);
		simConstants.mechanismMaxHardStop = kStowedAngle;
		simConstants.mechanismMinHardStop = kDeployedAngle;
		simConstants.simGravity = false;
		simConstants.mechanismStartPos = kStowedAngle;

		return simConstants;
	}

	public static ServoHomingConfig getServoHomingConfig() {
		ServoHomingConfig config = new ServoHomingConfig();
		config.kHomePosition = kStowedAngle;
		config.kHomingTimeout = Units.Seconds.of(0.2);
		config.kHomingVoltage = Units.Volts.of(2.0);
		config.kSetHomedVelocity = Units.DegreesPerSecond.of(1.0);

		return config;
	}
}
