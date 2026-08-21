package frc.lib.util.lights.state;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.lights.LEDClosedInterval;
import frc.lib.util.lights.RGBColor;
import java.util.function.Supplier;

public class DynamicFlashingLightsState extends FlashingLightsState {

	private final Supplier<Time> timeIntervalGetter;
	private final Supplier<RGBColor[]> colorsGetter;

	private int lastLength = 0;

	public DynamicFlashingLightsState(
			LEDClosedInterval interval, Supplier<Time> timeIntervalGetter, Supplier<RGBColor[]> colorsGetter) {
		super(interval, BaseUnits.TimeUnit.zero(), RGBColor.NONE);
		this.timeIntervalGetter = timeIntervalGetter;
		this.colorsGetter = colorsGetter;
	}

	@Override
	public boolean desireChange() {
		stopWatch.startIfNotRunning();
		return stopWatch.getTime().gte(timeIntervalGetter.get());
	}

	@Override
	public RGBColor getRGBColor() {
		RGBColor currentColor[] = colorsGetter.get();
		if (desireChange()) {
			if (lastLength != currentColor.length) {
				currentIndex = 0;
			}
			currentIndex++;
			if (currentIndex == currentColor.length) {
				currentIndex = currentColor.length % currentIndex;
			}
			stopWatch.resetAndStart();
		}
		return currentColor[currentIndex];
	}
}
