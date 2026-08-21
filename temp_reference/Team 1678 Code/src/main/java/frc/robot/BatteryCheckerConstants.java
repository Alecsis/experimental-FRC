package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.io.battery.BatteryCheckerIO;
import frc.lib.io.battery.BatteryCheckerIO.BatteryCheckerConfig;
import frc.lib.io.battery.ElasticBatteryCheckerIO;
import frc.lib.io.battery.ElasticBatteryCheckerIO.ElasticBatteryCheckerDisplayConfig;
import frc.lib.util.Elastic.Notification;
import frc.lib.util.Elastic.NotificationLevel;
import java.util.function.Supplier;

public class BatteryCheckerConstants {

	public static final Voltage kWarningVoltage = Volts.of(12.3);
	public static final Voltage kEmergencyVoltage = Volts.of(12.0);

	public static final Supplier<Voltage> voltageGetter = RobotController::getMeasureBatteryVoltage;

	public static final Time kWarningTimeThreshold = Seconds.of(5.0);
	public static final Time kEmergencyTimeThreshold = Seconds.of(2.0);

	public static final Time kEmergencyElasticNotificationSpawnDelay = Seconds.of(1.0);

	public static final Time kElasticWarningNotificationDisplayTime = Seconds.of(1.0);

	public static final double kFlickerScalar = 40;
	public static final double kQantityOfWarningsToSend = 4;

	public static final Notification createElasticWarningNotification() {
		return new Notification()
				.withLevel(NotificationLevel.WARNING)
				.withTitle("Battery Warning")
				.withDescription("Battery is below " + kWarningVoltage.in(Volts) + " Volts")
				.withAutomaticHeight()
				.withWidth(350)
				.withDisplaySeconds(kElasticWarningNotificationDisplayTime.in(Seconds));
	}

	public static final Notification createElasticEmergencyNotification() {
		return new Notification()
				.withLevel(NotificationLevel.ERROR)
				.withTitle("CHANGE BATTERY NOW!!!")
				.withDescription("Battery is below " + kEmergencyVoltage.in(Volts) + " Volts")
				.withAutomaticHeight()
				.withWidth(350)
				.withNoAutoDismiss();
	}

	public static final BatteryCheckerConfig createCheckingConfig() {
		BatteryCheckerConfig config = new BatteryCheckerConfig();
		config.publishingName = "Battery Checker";
		config.publishingVoltageUnit = Volts;
		config.publishingTimeUnit = Seconds;
		config.kEmergencyVoltage = kEmergencyVoltage;
		config.kWarningVoltage = kWarningVoltage;
		config.voltageGetter = voltageGetter;
		config.kBelowWarningTimeThreshold = kWarningTimeThreshold;
		config.kBelowEmergencyThreshold = kEmergencyTimeThreshold;
		return config;
	}

	public static final ElasticBatteryCheckerDisplayConfig createDisplayConfig() {
		ElasticBatteryCheckerDisplayConfig config = new ElasticBatteryCheckerDisplayConfig();
		config.batteryWarningElasticTabName = "Battery Warning";
		config.kWarningAlmostLowNotification = createElasticWarningNotification();
		config.kEmergencyNotification = createElasticEmergencyNotification();
		config.emergencySpawnDelayTime = kEmergencyElasticNotificationSpawnDelay;
		config.flickeringBooleanKey = "Change Battery";
		config.flickerScalar = kFlickerScalar;
		config.warningNotificationQuantityToSend = kQantityOfWarningsToSend;
		config.flickeringBooleanPublisher = (key, status) -> SmartDashboard.putBoolean(key, status);
		return config;
	}

	public static final BatteryCheckerIO createBatteryChecker() {
		return new ElasticBatteryCheckerIO(createCheckingConfig(), createDisplayConfig());
	}
}
