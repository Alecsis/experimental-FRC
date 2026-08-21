package frc.lib.util.lights.state;

import frc.lib.io.lights.LightsIO;
import frc.lib.util.lights.LEDClosedInterval;
import frc.lib.util.lights.LEDClosedInterval.LEDClosedIntervalCollisionBehavior;

public abstract class LightsState {

	protected final LEDClosedInterval interval;

	public LightsState(LEDClosedInterval interval) {
		this.interval = interval;
	}

	public LightsState withCollisionBehavior(LEDClosedIntervalCollisionBehavior collisionBehavior) {
		interval.withCollisionBehavior(collisionBehavior);
		return this;
	}

	public LEDClosedInterval getInterval() {
		return interval;
	}

	public LightsState handleCollision(LightsState colliding) {
		if (this.equals(colliding)) return this;
		if (getInterval().collides(colliding.getInterval())) {
			return switch (getInterval().getCollisionBehavior()) {
				case SELF_CANCEL -> colliding;
				case INCOMING_CANCEL -> this;
			};
		}
		return this;
	}

	public abstract void apply(LightsIO io);
}
