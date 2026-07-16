package frc.lib.io;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.UnaryOperator;

/**
 * Class used to control a main TalonFX and any number of followers for a real mechanism.
 */
public class MotorIOTalonFX extends MotorIO {
	protected final TalonFX main;
	protected final TalonFX[] followers;
	protected TalonFXConfiguration config;
	protected TalonFXConfiguration followerConfig;
	private final ControlRequestGetter requestGetter;
	private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
	private ThreadPoolExecutor threadPoolExecutor =
			new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);
	private boolean configFailed = false;

	public void applyConfig(TalonFX fx, TalonFXConfiguration config) {
		threadPoolExecutor.submit(() -> {
			for (int i = 0; i < 5; i++) {
				StatusCode result = fx.getConfigurator().apply(config);
				if (result.isOK()) {
					break;
				} else {
					configFailed = true;
				}
			}
		});
	}

	@Override
	public void updateInputs() {
		updateMotorInputs(inputs, main);

		for (int i = 0; i < followers.length; i++) {
			updateMotorInputs(followerInputs[i], followers[i]);
		}
	}

	/**
	 * Updates one Inputs from readings of a TalonFX motor
	 *
	 * @param inputsToUpdate Inputs to update from reading.
	 * @param motor Motor to read from.
	 */
	protected void updateMotorInputs(Inputs inputsToUpdate, TalonFX motor) {
		inputsToUpdate.position = motor.getPosition().getValue();
		inputsToUpdate.velocity = motor.getVelocity().getValue();
		inputsToUpdate.statorCurrent = motor.getStatorCurrent().getValue();
		inputsToUpdate.supplyCurrent = motor.getSupplyCurrent().getValue();
		inputsToUpdate.motorVoltage = motor.getMotorVoltage().getValue();
		inputsToUpdate.pidVoltage = Units.Volts.of(motor.getClosedLoopOutput().getValue());
		inputsToUpdate.motorTemperature = motor.getDeviceTemp().getValue();
		inputsToUpdate.acceleration = motor.getAcceleration().getValue();
	}

	private void setControl(ControlRequest request) {
		main.setControl(request);
	}

	@Override
	public void setNeutralSetpoint() {
		setControl(new NeutralOut());
	}

	@Override
	public void setCoastSetpoint() {
		setControl(new CoastOut());
	}

	@Override
	protected void setVoltageSetpoint(Voltage voltage) {
		setControl(requestGetter.getVoltageRequest(voltage));
	}

	@Override
	protected void setDutyCycleSetpoint(Dimensionless percent) {
		setControl(requestGetter.getDutyCycleRequest(percent));
	}

	@Override
	protected void setMotionMagicSetpoint(Angle mechanismPosition) {
		setMotionMagicSetpoint(mechanismPosition, 0);
	}

	@Override
	protected void setMotionMagicSetpoint(Angle mechanismPosition, int slot) {
		setControl(requestGetter.getMotionMagicRequest(mechanismPosition, slot));
	}

	@Override
	protected void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
		setVelocitySetpoint(mechanismVelocity, 1);
	}

	@Override
	protected void setVelocitySetpoint(AngularVelocity mechanismVelocity, int slot) {
		setControl(requestGetter.getVelocityRequest(mechanismVelocity, slot));
	}

	@Override
	protected void setPositionSetpoint(Angle mechanismPosition) {
		setPositionSetpoint(mechanismPosition, 2);
	}

	@Override
	protected void setPositionSetpoint(Angle mechanismPosition, int slot) {
		setControl(requestGetter.getPositionRequest(mechanismPosition, slot));
	}

	@Override
	public void setCurrentPosition(Angle mechanismPosition) {
		threadPoolExecutor.submit(() -> {
			main.setPosition(mechanismPosition);
		});
	}

	@Override
	public void zeroSensors() {
		setCurrentPosition(Units.Rotations.of(0.0));
	}

	private void setNeutralMode(TalonFX fx, NeutralModeValue neutralMode) {
		SmartDashboard.putNumber("TALON FX NEUTRAL MODE SET!!", Timer.getFPGATimestamp());
		threadPoolExecutor.submit(() -> {
			fx.setNeutralMode(neutralMode);
		});
	}

	@Override
	public void setNeutralBrake(boolean wantsBrake) {
		NeutralModeValue neutralMode = wantsBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		config.MotorOutput.NeutralMode = neutralMode;
		setNeutralMode(main, neutralMode);
		for (TalonFX talon : followers) {
			setNeutralMode(talon, neutralMode);
		}
	}

	@Override
	public void useSoftLimits(boolean enable) {
		UnaryOperator<TalonFXConfiguration> configChanger = (config) -> {
			config.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
			config.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
			return config;
		};

		changeMainConfig(configChanger);
	}

	@Override
	public TalonFXConfiguration getMotorIOConfig() {
		return config;
	}

	@Override
	public void disabledPeriodic() {}

	/**
	 * Applies a TalonFXConfiguration to the main motor.
	 *
	 * @param configuration Configuration to apply.
	 */
	public void setMainConfig(TalonFXConfiguration configuration) {
		config = configuration;
		applyConfig(main, config);
	}

	/**
	 * Changes the currently applied main TalonFXConfiguration and applies the new configuration to the main motor.
	 *
	 * @param configChanger Mutating operation to apply on the current configuration.
	 */
	public void changeMainConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
		setMainConfig(configChanger.apply(config));
	}

	/**
	 * Applies a TalonFXConfiguration to all follower motors.
	 *
	 * @param configuration Configuration to apply.
	 */
	public void setFollowerConfig(TalonFXConfiguration configuration) {
		followerConfig = configuration;
		for (TalonFX talon : followers) {
			applyConfig(talon, followerConfig);
		}
	}

	/**
	 * Changes the currently applied follower TalonFXConfiguration and applies the new configuration to all follower motors.
	 *
	 * @param configChanger Mutating operation to apply on the current configuration.
	 */
	public void changeFollowerConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
		setFollowerConfig(configChanger.apply(followerConfig));
	}

	/**
	 * Creates a MotorIOTalonFX from a provided configuration.
	 *
	 * @param config Configuration to create MotorIOTalonFX from.
	 */
	public MotorIOTalonFX(MotorIOTalonFXConfig config) {
		super(config.unit, config.time, config.followerIDs.length);
		requestGetter = config.requestGetter;
		main = new TalonFX(config.mainID, config.mainBus);
		setMainConfig(config.mainConfig);

		followers = new TalonFX[config.followerIDs.length];
		for (int i = 0; i < config.followerIDs.length; i++) {
			followers[i] = new TalonFX(config.followerIDs[i], config.followerBuses[i]);
			followers[i].setControl(new Follower(config.mainID, config.followerAlignment[i]));
		}

		setFollowerConfig(config.followerConfig);
	}

	/**
	 * Configuration for a MotorIOTalonFX. Motion magic control is on slot 0, velocity on slot 1, and position PID on slot 2.
	 */
	public static class MotorIOTalonFXConfig {
		public AngleUnit unit = Units.Rotations;
		public TimeUnit time = Units.Seconds;
		public int mainID = -1;
		public CANBus mainBus = new CANBus("ASSIGN_BUS");
		public TalonFXConfiguration mainConfig = new TalonFXConfiguration();
		public int[] followerIDs = new int[0];
		public CANBus[] followerBuses = new CANBus[0];
		public TalonFXConfiguration followerConfig = new TalonFXConfiguration();

		public MotorAlignmentValue followerAlignment[] = new MotorAlignmentValue[0];
		public ControlRequestGetter requestGetter = new ControlRequestGetter();
	}

	public static class ControlRequestGetter {
		public ControlRequest getVoltageRequest(Voltage voltage) {
			return new VoltageOut(voltage.in(Units.Volts)).withEnableFOC(true);
		}

		public ControlRequest getDutyCycleRequest(Dimensionless percent) {
			return new DutyCycleOut(percent.in(Units.Percent));
		}

		public ControlRequest getMotionMagicRequest(Angle mechanismPosition, int slot) {
			return new MotionMagicExpoVoltage(mechanismPosition)
					.withSlot(slot)
					.withEnableFOC(true)
					.withUpdateFreqHz(Hertz.of(1000));
		}

		public ControlRequest getVelocityRequest(AngularVelocity mechanismVelocity, int slot) {
			return new MotionMagicVelocityVoltage(mechanismVelocity)
					.withUpdateFreqHz(Hertz.of(1000.0))
					.withEnableFOC(true)
					.withSlot(slot);
		}

		public ControlRequest getPositionRequest(Angle mechanismPosition, int slot) {
			return new PositionVoltage(mechanismPosition).withSlot(slot).withEnableFOC(true);
		}
	}

	@Override
	public boolean getConfigFailed() {
		return configFailed;
	}

	public TalonFX getMain() {
		return main;
	}
}
