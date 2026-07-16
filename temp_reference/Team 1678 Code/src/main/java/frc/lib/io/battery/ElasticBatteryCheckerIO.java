package frc.lib.io.battery;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.util.BatteryCheckerInputs;
import frc.lib.util.Elastic;
import frc.lib.util.Elastic.Notification;
import frc.lib.util.Stopwatch;
import java.util.function.BiConsumer;

public class ElasticBatteryCheckerIO extends BatteryCheckerIO {

	private final Stopwatch checkStopWatch = new Stopwatch();

	public static class ElasticBatteryCheckerDisplayConfig {

		public Notification kWarningAlmostLowNotification;
		public Notification kEmergencyNotification;

		public String batteryWarningElasticTabName;
		public double warningNotificationQuantityToSend;

		public BiConsumer<String, Boolean> flickeringBooleanPublisher;

		public String flickeringBooleanKey;
		public double flickerScalar;

		public Time emergencySpawnDelayTime;
	}

	private final ElasticBatteryCheckerDisplayConfig config;
	private boolean lastStatus = false;

	private Time nextFlickerDelay = Seconds.of(Double.POSITIVE_INFINITY);
	private Time lastFlickerDelay = nextFlickerDelay;
	private final Stopwatch flickerStopwatch = new Stopwatch();
	private final Stopwatch emergencySpawnNotificationStopwatch = new Stopwatch();

	/**
	 *
	 * @param mainConfig main config that handles checking voltage
	 * @param config config that handles how to display warnings and alerts
	 */
	public ElasticBatteryCheckerIO(BatteryCheckerConfig mainConfig, ElasticBatteryCheckerDisplayConfig config) {
		super(mainConfig);
		this.config = config;
		checkStopWatch.reset();
	}

	/**
	 *
	 * @param status status to set
	 * @apiNote publishes a boolean to NT4 and changes the value from true to false to simulate flickering
	 */
	public void publishFlicker(boolean status) {
		lastStatus = status;
		config.flickeringBooleanPublisher.accept(config.flickeringBooleanKey, status);
	}

	/**
	 * @apiNote calls {@link #publishFlicker(boolean)} with the opposite of {@link #lastStatus}
	 */
	public void publishFlicker() {
		publishFlicker(!lastStatus);
	}

	/**
	 * @apiNote sets the elastic tab to {@link ElasticBatteryCheckerDisplayConfig#batteryWarningElasticTabName}
	 * ands flickers the boolean
	 */
	public void flicker() {
		Elastic.selectTab(config.batteryWarningElasticTabName);
		publishFlicker();
	}

	/**
	 *
	 * @param elapsed time elapsed from when the supplied voltage is below the min voltage
	 * @param currentVoltage supplied voltage
	 * @param minVoltage voltage that the supplied voltage is under
	 * @return time elapsed to next {@link #flicker()} call
	 */
	public Time calculateFlickerRate(Time elapsed, Voltage currentVoltage, Voltage minVoltage) {
		double raw = elapsed.in(BaseUnits.TimeUnit);
		double voltsRelativeToMin = minVoltage.minus(currentVoltage).in(Volts);
		double value = (1 - (1 / (1 + Math.pow(Math.E, -raw)))) * (config.flickerScalar * voltsRelativeToMin);
		return BaseUnits.TimeUnit.of(Math.max(0.5, Math.min(value, 5)));
	}

	@Override
	public void onWarningVoltage(BatteryCheckerInputs inputs) {
		checkStopWatch.startIfNotRunning();
		Notification currentNotification = config.kWarningAlmostLowNotification;
		if (checkStopWatch.getTime().gte(Milliseconds.of(currentNotification.getDisplayTimeMillis()))) {
			for (int i = 0; i < 4; i++) {
				Elastic.sendNotification(currentNotification);
			}
			checkStopWatch.reset();
		}
	}

	@Override
	public void onEmergencyVoltage(BatteryCheckerInputs inputs) {
		emergencySpawnNotificationStopwatch.startIfNotRunning();
		if (emergencySpawnNotificationStopwatch.getTime().gte(config.emergencySpawnDelayTime)) {
			Elastic.sendNotification(config.kEmergencyNotification);
			emergencySpawnNotificationStopwatch.resetAndStart();
		}
		Time elapsed = inputs.atEmergencyThresholdTime;
		Time calculatedFlickerTime = calculateFlickerRate(elapsed, inputs.currentVoltage, inputs.emergencyThreshold);
		nextFlickerDelay = calculatedFlickerTime;
	}

	@Override
	public void update() {
		super.update();
		if (!lastFlickerDelay.isEquivalent(nextFlickerDelay)) {
			flickerStopwatch.reset();
			if (Double.isFinite(nextFlickerDelay.in(Seconds))) {
				flickerStopwatch.start();
			}
		}
		flickerStopwatch.startIfNotRunning();
		if (flickerStopwatch.getTime().gte(nextFlickerDelay)) {
			flicker();
			flickerStopwatch.reset();
		}
		lastFlickerDelay = nextFlickerDelay;
	}

	@Override
	public void reset() {
		super.reset();
		publishFlicker(false);
		flickerStopwatch.reset();
		lastFlickerDelay = BaseUnits.TimeUnit.of(Double.POSITIVE_INFINITY);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		builder.addDoubleProperty("Time to Next Flicker MilliSeconds", () -> nextFlickerDelay.in(Milliseconds), null);
		builder.addDoubleProperty("Last Flicker Delay MillSeconds", () -> lastFlickerDelay.in(Milliseconds), null);
	}
}
