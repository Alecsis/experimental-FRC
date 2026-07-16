package frc.lib.util.lights.state;

import frc.lib.io.lights.LightsIO;
import frc.lib.util.lights.LEDClosedInterval;
import frc.lib.util.lights.RGBColor;

public class SolidLightsState extends LightsState {

	private final RGBColor color;

	public SolidLightsState(RGBColor color, LEDClosedInterval segment) {
		super(segment);
		this.color = color;
	}

	@Override
	public void apply(LightsIO io) {
		io.setLED(color, interval);
	}

	@Override
	public String toString() {
		return "Solid: " + color.toString();
	}
}
