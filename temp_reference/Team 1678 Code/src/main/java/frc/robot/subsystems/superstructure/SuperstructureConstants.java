package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.trigger.CANRangeSensorIO;
import frc.lib.io.trigger.CANRangeSensorIO.CANRangeTriggerIOConfig;
import frc.lib.io.trigger.SensorIO;
import frc.lib.io.trigger.SimulatedSensorIO;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.shooting.Shooting.ShootingState;

public class SuperstructureConstants {

	public static final Distance TRENCH_SHOT_DISTANCE = Meters.of(3.85);
	public static final Distance TOWER_SHOT_DISTANCE = Meters.of(3.048);
	public static final Distance FENDER_SHOT_DISTANCE = Meters.of(1.172);

	class CANRanges {
		private static final double MIN_SIGNAL_THRESHOLD = 2500;

		public static final CANrangeConfiguration createBaseConfig() {
			return new CANrangeConfiguration()
					.withProximityParams(new ProximityParamsConfigs()
							.withMinSignalStrengthForValidMeasurement(MIN_SIGNAL_THRESHOLD)
							.withProximityHysteresis(0.01)
							.withProximityThreshold(0.4))
					.withToFParams(new ToFParamsConfigs()
							.withUpdateFrequency(50)
							.withUpdateMode(UpdateModeValue.ShortRangeUserFreq));
		}

		class Hopper {
			private static final Distance MAX_DISTANCE = Meters.of(0.3);
			private static final Time DEBOUNCE_TIME = Seconds.of(0.75);

			private static CANrangeConfiguration getConfig() {
				return createBaseConfig()
						.withFovParams(new FovParamsConfigs()
								.withFOVCenterX(Degrees.of(0))
								.withFOVCenterY(Degrees.of(0))
								.withFOVRangeX(Degrees.of(27))
								.withFOVRangeY(Degrees.of(6.75)));
			}

			private static final CANRangeTriggerIOConfig CONFIG = new CANRangeSensorIO.CANRangeTriggerIOConfig(
					Ports.HOPPER_CANRANGE.id,
					Ports.HOPPER_CANRANGE.bus,
					5,
					getConfig(),
					MIN_SIGNAL_THRESHOLD,
					MAX_DISTANCE,
					"Hopper CAN");

			public static final SensorIO getCANRange() {
				if (Robot.isReal()) {
					try {
						return new CANRangeSensorIO(CONFIG, DEBOUNCE_TIME);
					} catch (Exception e) {
						return new SimulatedSensorIO(() -> false, DEBOUNCE_TIME);
					}
				} else {
					return new SimulatedSensorIO(() -> false, DEBOUNCE_TIME);
				}
			}

			public static final SensorIO CAN = getCANRange();
		}

		class BallTunnel {
			private static final double MIN_SIGNAL_THRESHOLD = 2500;
			private static final Distance MAX_DISTANCE = Meters.of(0.6);
			private static final Time DEBOUNCE_TIME = Seconds.of(0.06);

			private static CANrangeConfiguration getConfig() {
				return createBaseConfig()
						.withFovParams(new FovParamsConfigs()
								.withFOVCenterX(Degrees.of(0))
								.withFOVCenterY(Degrees.of(0))
								.withFOVRangeX(Degrees.of(10.0))
								.withFOVRangeY(Degrees.of(6.75)));
			}

			private static final CANRangeTriggerIOConfig CONFIG = new CANRangeSensorIO.CANRangeTriggerIOConfig(
					Ports.TUNNEL_CANRANGE.id,
					Ports.TUNNEL_CANRANGE.bus,
					5,
					getConfig(),
					MIN_SIGNAL_THRESHOLD,
					MAX_DISTANCE,
					"Ball Tunnel CAN");

			public static final SensorIO getCANRange() {
				if (Robot.isReal()) {
					try {
						return new CANRangeSensorIO(CONFIG, DEBOUNCE_TIME);
					} catch (Exception e) {
						return new SimulatedSensorIO(() -> false, DEBOUNCE_TIME);
					}
				} else {
					return new SimulatedSensorIO(() -> false, DEBOUNCE_TIME);
				}
			}

			public static final SensorIO CAN = getCANRange();
		}

		public static void update() {
			SuperstructureConstants.CANRanges.Hopper.CAN.update();
			SuperstructureConstants.CANRanges.BallTunnel.CAN.update();
		}
	}

	public static Setpoint getShooterSetpointTower() {
		return Setpoint.withVelocitySetpoint(
				ShootingState.HUB.getShootingRegressions().getShooterVelocity(TOWER_SHOT_DISTANCE));
	}

	public static Setpoint getHoodSetpointTower() {
		return Setpoint.withMotionMagicSetpoint(
				ShootingState.HUB.getShootingRegressions().getHoodAngle(TOWER_SHOT_DISTANCE));
	}

	public static Setpoint getTunnelRollerSetpointTower() {
		return Setpoint.withVelocitySetpoint(
				ShootingState.HUB.getShootingRegressions().getTunnelVelocity(TOWER_SHOT_DISTANCE));
	}

	public static Setpoint getShooterSetpointTrench() {
		return Setpoint.withVelocitySetpoint(
				ShootingState.HUB.getShootingRegressions().getShooterVelocity(TRENCH_SHOT_DISTANCE));
	}

	public static Setpoint getHoodSetpointTrench() {
		return Setpoint.withMotionMagicSetpoint(
				ShootingState.HUB.getShootingRegressions().getHoodAngle(TRENCH_SHOT_DISTANCE));
	}

	public static Setpoint getTunnelRollerSetpointTrench() {
		return Setpoint.withVelocitySetpoint(
				ShootingState.HUB.getShootingRegressions().getTunnelVelocity(TRENCH_SHOT_DISTANCE));
	}

	public static Setpoint getShooterSetpointFender() {
		return Setpoint.withVelocitySetpoint(
				ShootingState.HUB.getShootingRegressions().getShooterVelocity(FENDER_SHOT_DISTANCE));
	}

	public static Setpoint getHoodSetpointFender() {
		return Setpoint.withMotionMagicSetpoint(
				ShootingState.HUB.getShootingRegressions().getHoodAngle(FENDER_SHOT_DISTANCE));
	}

	public static Setpoint getTunnelRollerSetpointFender() {
		return Setpoint.withVelocitySetpoint(
				ShootingState.HUB.getShootingRegressions().getTunnelVelocity(FENDER_SHOT_DISTANCE));
	}

	public static final Time LOOK_AHEAD_TIME = Milliseconds.of(100.0);

	public static final Time FIRST_VOLLEY_FORCED_TIME = Seconds.of(0.25);

	public static final Dimensionless SHOOTER_OVERRUN_PERCENT = Percent.of(0.0); // 6.5
	public static final Dimensionless SHOOTER_OVERRUN_PERCENT_WHILE_PRELOADED = Percent.of(16.7);
	public static final LinearVelocity MAX_SOTM_SPEED = MetersPerSecond.of(1.0);
	public static final LinearVelocity MAX_FOTM_SPEED = MetersPerSecond.of(Double.MAX_VALUE);

	public static final class AutoTuckConstants {

		public static final Time DISTANCE_FROM_TRENCH_DIRECTION_VELOCITY_DEBOUNCE_TIME = Milliseconds.of(10.0);

		public static final Distance DISTANCE_FROM_TRENCH_TO_STOW = Meters.of(1.5);
	}

	public static final class ForceHomeConstants {
		public static final Time INTAKE_FORCE_DEBOUNCE = Seconds.of(5.0);
		public static final AngularVelocity INTAKE_MIN_HOME_VELOCITY = DegreesPerSecond.of(5.0);
		public static final AngularVelocity HOOD_MIN_HOME_VELOCITY = DegreesPerSecond.of(2.0);

		public static final LinearVelocity CLIMBER_MIN_HOME_VELOCITY =
				Inches.of(3.0).per(Seconds);
	}
}
