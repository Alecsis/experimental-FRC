package frc.lib.util.lights.state;

import frc.lib.util.lights.LEDClosedInterval;
import frc.lib.util.lights.RGBColor;

public class EmptyLightsState extends SolidLightsState {

	public EmptyLightsState(LEDClosedInterval segment) {
		super(RGBColor.NONE, segment);
	}
}
