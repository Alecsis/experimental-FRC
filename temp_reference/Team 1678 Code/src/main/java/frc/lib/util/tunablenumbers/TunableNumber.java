package frc.lib.util.tunablenumbers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;

/**
 * Class creating a mutable object which can be mutated dynamically through SmartDashboard
 */
public class TunableNumber implements DoubleSupplier {
	private static final String TABLE_KEY = "TunableNumbers";
	private String key; // key associated with the object on SmartDashboard
	private double defaultValue; // default value
	private double currentValue; // current value in SmartDashboard
	private double lastCurrentValue; // last checked current value

	/**
	 * Constructs a new Tunable number
	 * @param smartDashboardKey key used in smartDashboard
	 * @param defaultValue
	 */
	public TunableNumber(String smartDashboardKey, double defaultValue) {
		key = TABLE_KEY + "/" + smartDashboardKey; // Constructs a new TunableNumber with smartDashboardKey
		publish(defaultValue); // Assigns the default value parameter as the default value and publishes the value into
		// SmartDashboard
		// smartDashboard
		this.currentValue = defaultValue;
		this.lastCurrentValue = currentValue;
	}
	/**
	 * Gets the default value
	 * @return
	 */
	public double getDefault() {
		return defaultValue;
	}
	/**
	 * Sets the default value
	 * Smartdashboard puts that value into network tables with the object's key
	 * @param valueToSet
	 */
	public void publish(double valueToSet) {
		defaultValue = valueToSet;
		SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, valueToSet));
	}
	/**
	 * Gets the current value on smartdashboard
	 */
	@Override
	public double getAsDouble() {
		return SmartDashboard.getNumber(key, 0.0);
	}
	/**
	 * Checks if the current value in smartdashboard has changed
	 * Returns true is the current value is different from the last current value
	 */
	public boolean hasChanged() {
		currentValue = getAsDouble();
		if (currentValue != lastCurrentValue) {
			lastCurrentValue = currentValue;
			return true;
		}
		return false;
	}
}
