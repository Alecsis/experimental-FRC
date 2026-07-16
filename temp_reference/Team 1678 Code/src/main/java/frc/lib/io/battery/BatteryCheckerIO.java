package frc.lib.io.battery;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.BatteryCheckerInputs;
import frc.lib.util.Stopwatch;
import java.util.function.Supplier;

public abstract class BatteryCheckerIO implements Sendable {

	public static class BatteryCheckerConfig {

		public String publishingName;
		public VoltageUnit publishingVoltageUnit;
		public TimeUnit publishingTimeUnit;

		public Time kBelowWarningTimeThreshold;
		public Time kBelowEmergencyThreshold;

		public Voltage kWarningVoltage;
		public Voltage kEmergencyVoltage;
		public Supplier<Voltage> voltageGetter;
	}

	private final BatteryCheckerConfig config;
	private boolean disabled = false;
	private final BatteryCheckerInputs inputs;

	private final Debouncer warningVoltageDebouncer;
	private final Debouncer emergencyVoltageDebouncer;

	private final Stopwatch atWarningVoltageStopWatch = new Stopwatch();
	private final Stopwatch atEmergencyVoltageStopWatch = new Stopwatch();

	/**
	 *
	 * @param config config to apply
	 */
	public BatteryCheckerIO(BatteryCheckerConfig config) {
		this.config = config;
		inputs = new BatteryCheckerInputs(config.publishingVoltageUnit, config.publishingTimeUnit);
		warningVoltageDebouncer = new Debouncer(config.kBelowWarningTimeThreshold.in(Seconds));
		emergencyVoltageDebouncer = new Debouncer(config.kBelowEmergencyThreshold.in(Seconds));
		atWarningVoltageStopWatch.reset();
		atEmergencyVoltageStopWatch.reset();
		SendableRegistry.add(this, config.publishingName);
		updateInputs();
	}

	public Voltage getSuppliedVoltage() {
		return inputs.currentVoltage;
	}

	/**
	 *
	 * @param suppliedVoltage current voltage
	 * @param lowerThreshold lower threshold for voltage
	 * @param debouncer timed evaluation if the current voltage is less than the lower
	 * @param stopWatch stopwatch that times how long the debounced value of when the current voltage is than the lower threshold
	 * @return debounced value of when the supplied voltage is less than the lower threshold
	 */
	private boolean runVoltageCheck(
			Voltage suppliedVoltage, Voltage lowerThreshold, Debouncer debouncer, Stopwatch stopWatch) {
		boolean status = debouncer.calculate(suppliedVoltage.lte(lowerThreshold));
		if (status) {
			stopWatch.startIfNotRunning();
		} else {
			stopWatch.reset();
		}
		return status;
	}

	public boolean runWarningVoltageCheck() {
		return runVoltageCheck(
				inputs.currentVoltage, inputs.warningThreshold, warningVoltageDebouncer, atWarningVoltageStopWatch);
	}

	public boolean runEmergencyVoltageCheck() {
		return runVoltageCheck(
				inputs.currentVoltage,
				inputs.emergencyThreshold,
				emergencyVoltageDebouncer,
				atEmergencyVoltageStopWatch);
	}

	public void runChecks() {
		runWarningVoltageCheck();
		runEmergencyVoltageCheck();
	}

	public boolean atWarningVoltage() {
		return inputs.atWarningThresholdTime.gt(BaseUnits.TimeUnit.zero());
	}

	public boolean atEmergencyVoltage() {
		return inputs.atEmergencyThresholdTime.gt(BaseUnits.TimeUnit.zero());
	}

	/**
	 * @apiNote function to be called to reset tracking
	 */
	public void reset() {
		inputs.reset();
	}

	/**
	 *
	 * @param disabled next disabled state
	 * @apiNote when disabled, periodic updates are prevented
	 */
	public void disable(boolean disabled) {
		this.disabled = disabled;
	}

	/**
	 * @apiNote sets the next disabled state to the opposite state of the current disabled state
	 */
	public void disable() {
		disable(!disabled);
	}

	/**
	 *
	 * @param disabled next disabled state
	 * @apiNote when disabled, periodic updates are prevented. calls {@link reset} after disable
	 */
	public void disableAndReset(boolean disabled) {
		disable(disabled);
		reset();
	}

	/**
	 *
	 * @apiNote sets the next disabled state to the opposite state of the current disabled state, calls {@link #reset} after disable
	 */
	public void disableAndReset() {
		disable();
		reset();
	}

	/* disable WPI commands */

	/**
	 *
	 * @param disabled next disabled state
	 * @apiNote when disabled, periodic updates are prevented
	 * @return a instant command that calls {@link #disable(boolean)}
	 */
	public Command disableCommand(boolean disabled) {
		return Commands.runOnce(() -> disable(disabled));
	}

	/**
	 * @apiNote sets the next disabled state to the opposite state of the current disabled state
	 * @return a instant command that calls {@link #disable()}
	 */
	public Command disableCommand() {
		return Commands.runOnce(() -> disable());
	}

	/**
	 *
	 * @param disabled next disabled state
	 * @apiNote when disabled, periodic updates are prevented. calls {@link reset} after disable
	 * @return a instant command that calls {@link #disableAndReset(boolean)}
	 */
	public Command disableAndResetCommand(boolean disabled) {
		return Commands.runOnce(() -> disableAndReset(disabled));
	}

	/**
	 *
	 * @apiNote sets the next disabled state to the opposite state of the current disabled state, calls {@link reset} after disable
	 * @return a instant command that calls {@link #disableAndReset()}
	 */
	public Command disableAndResetCommand() {
		return Commands.runOnce(() -> disableAndReset());
	}

	public void updateInputs() {
		inputs.update(
				config.voltageGetter.get(),
				config.kWarningVoltage,
				config.kEmergencyVoltage,
				atWarningVoltageStopWatch.getTime(),
				atEmergencyVoltageStopWatch.getTime());
	}

	/**
	 * @apiNote function that will handle updates
	 * if it is currently disabled, updates will not be called
	 * else, will call {@link #update}
	 */
	public void periodic() {
		if (disabled) return;
		else update();
	}

	/**
	 * @apiNote function where all periodic updates happen
	 * SHOULD ONLY BE CALLED BY {@link #periodic} TO HANDLE DISABLED STATES
	 */
	protected void update() {

		updateInputs();

		runChecks();

		if (atEmergencyVoltage()) {
			onEmergencyVoltage(inputs);
		}
		if (atWarningVoltage()) {
			onWarningVoltage(inputs);
		}
	}

	/**
	 *
	 * @param inputs inputs when the supplied voltage is below the warning lower threshold
	 * @apiNote called by {@link #update}
	 */
	public abstract void onWarningVoltage(BatteryCheckerInputs inputs);

	/**
	 *
	 * @param inputs inputs when the supplied voltage is below the emergency lower threshold
	 * @apiNote called by {@link #update}
	 */
	public abstract void onEmergencyVoltage(BatteryCheckerInputs inputs);

	public BatteryCheckerInputs getInputs() {
		return inputs;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		SendableRegistry.setName(inputs, "Inputs");
		SendableRegistry.addChild(this, inputs);
		builder.addBooleanProperty("Disabled", () -> disabled, null);
		inputs.initSendable(builder);
	}
}
