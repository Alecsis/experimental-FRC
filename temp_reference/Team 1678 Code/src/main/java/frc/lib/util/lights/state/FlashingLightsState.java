package frc.lib.util.lights.state;

import edu.wpi.first.units.measure.Time;
import frc.lib.io.lights.LightsIO;
import frc.lib.util.Stopwatch;
import frc.lib.util.lights.LEDClosedInterval;
import frc.lib.util.lights.RGBColor;

public class FlashingLightsState extends LightsState {

	private final Time timeInterval;
	protected final Stopwatch stopWatch = new Stopwatch();

	protected final RGBColor colors[];

	protected int currentIndex = 0;

	public FlashingLightsState(LEDClosedInterval segment, Time interval, RGBColor... colors) {
		super(segment);
		this.timeInterval = interval;
		this.colors = colors;
	}

	public boolean desireChange() {
		stopWatch.startIfNotRunning();
		return stopWatch.getTime().gte(timeInterval);
	}

	public RGBColor getRGBColor() {
		if (desireChange()) {
			currentIndex++;
			if (currentIndex == colors.length) {
				currentIndex = colors.length % currentIndex;
			}
			stopWatch.resetAndStart();
		}
		return colors[currentIndex];
	}

	@Override
	public void apply(LightsIO io) {
		io.setLED(getRGBColor(), super.interval);
	}

	@Override
	public String toString() {
		return "Flashing: " + colors[currentIndex].toString();
	}
}
