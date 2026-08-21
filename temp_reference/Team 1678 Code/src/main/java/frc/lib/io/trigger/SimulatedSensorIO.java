package frc.lib.io.trigger;

import edu.wpi.first.units.measure.Time;
import java.util.function.BooleanSupplier;

public class SimulatedSensorIO extends SensorIO {

	private final BooleanSupplier condition;

	public SimulatedSensorIO(BooleanSupplier condition, Time debounceTime) {
		super(debounceTime, "temp");
		this.condition = condition;
	}

	@Override
	public boolean get() {
		return condition.getAsBoolean();
	}
}
