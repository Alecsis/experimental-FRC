package frc.lib.io.trigger;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalInputSensorIO extends SensorIO {

	private final DigitalInput input;

	public DigitalInputSensorIO(int dioChannel, Time debounceTime) {
		super(debounceTime, "temp");
		this.input = new DigitalInput(dioChannel);
	}

	@Override
	public boolean get() {
		return input.get();
	}
}
