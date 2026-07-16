package frc.lib.io.lights;

import frc.lib.util.lights.LEDClosedInterval;
import frc.lib.util.lights.RGBColor;

public class SimLightsIO extends LightsIO {

	public SimLightsIO(LEDClosedInterval segment) {
		super(segment);
	}

	@Override
	public void setLED(RGBColor color, LEDClosedInterval interval) {}
}
