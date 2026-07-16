package frc.lib.io.trigger;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Robot;

public abstract class SensorIO implements Sendable {

	private final Time debounceTime;
	private final Debouncer debouncer;
	protected final String name;

	public SensorIO(Time debounceTime, String name) {
		this.debounceTime = debounceTime;
		debouncer = new Debouncer(debounceTime.in(Seconds));
		this.name = name;
	}

	public abstract boolean get();

	public void update() {}

	public boolean getDebounced() {
		return debouncer.calculate(get());
	}

	public boolean getDebouncedIfReal() {
		return Robot.isReal() ? getDebounced() : true;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addBooleanProperty(name + "/Input", this::get, null);
		builder.addBooleanProperty(name + "/Input Debounced", this::getDebounced, null);
	}
}
