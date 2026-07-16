package frc.lib.bases;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.lights.LightsIO;
import frc.lib.util.lights.state.EmptyLightsState;
import frc.lib.util.lights.state.LightsState;
import java.util.ArrayList;
import java.util.function.Consumer;

public class LightsSubsystem extends SubsystemBase {

	private final String name;
	private final LightsIO ios[];

	private ArrayList<LightsState> wantedStates = null;

	public LightsSubsystem(String name, LightsIO... ios) {
		super(name);
		this.name = name;
		this.ios = ios;
	}

	public void addWantedState(LightsState... states) {
		for (LightsState incomingState : states) {
			if (wantedStates == null) {
				wantedStates = new ArrayList<>();
				wantedStates.add(incomingState);
			} else {
				ArrayList<LightsState> buffer = new ArrayList<>();
				boolean appendIncomingDuringFinal = true;
				for (LightsState stored : wantedStates) {
					if (stored.getInterval().collides(incomingState.getInterval())) {
						buffer.add(stored.handleCollision(incomingState));
						appendIncomingDuringFinal = false;
					} else {
						buffer.add(stored);
					}
				}
				if (appendIncomingDuringFinal) {
					buffer.add(incomingState);
				}
				wantedStates = buffer;
			}
		}
	}

	public void removeWantedState(LightsState... states) {
		ArrayList<LightsState> nextStates = new ArrayList<>();
		for (LightsState stored : wantedStates) {
			boolean wantAppend = true;
			for (LightsState removeTarget : states) {
				if (removeTarget == stored) {
					wantAppend = false;
				}
			}
			if (wantAppend) nextStates.add(stored);
			else nextStates.add(new EmptyLightsState(stored.getInterval()));
		}
		wantedStates = nextStates;
	}

	public void clearStates() {
		if (wantedStates != null) wantedStates.clear();
		forEachIO(io -> new EmptyLightsState(io.getSegment()).apply(io));
	}

	public void forEachIO(Consumer<LightsIO> consumer) {
		for (LightsIO io : ios) consumer.accept(io);
	}

	protected void update() {
		if (wantedStates != null) {
			for (LightsIO io : ios) {
				for (LightsState state : wantedStates) {
					state.apply(io);
				}
			}
			wantedStates.clear();
		}
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		forEachIO(io -> io.initSendable(builder));
	}
}
