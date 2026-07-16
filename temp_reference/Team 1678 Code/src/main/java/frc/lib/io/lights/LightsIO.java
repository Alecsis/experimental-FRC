package frc.lib.io.lights;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.util.lights.LEDClosedInterval;
import frc.lib.util.lights.RGBColor;
import frc.lib.util.lights.state.EmptyLightsState;
import frc.lib.util.lights.state.LightsState;
import java.util.ArrayList;

public abstract class LightsIO implements Sendable {

	private LightsState currentState;
	protected final LEDClosedInterval segment;

	protected final ArrayList<LightsState> runningStates = new ArrayList<>();

	public LightsIO(LEDClosedInterval segment) {
		this.currentState = new EmptyLightsState(segment);
		this.segment = segment;
	}

	public LEDClosedInterval getSegment() {
		return segment;
	}

	public abstract void setLED(RGBColor color, LEDClosedInterval interval);

	public LightsState getState() {
		return currentState;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addStringProperty("State", () -> getState().toString(), null);
	}
}
