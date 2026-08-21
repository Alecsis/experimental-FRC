package frc.lib.util.lights;

import frc.lib.math.ClosedInterval;

public class LEDClosedInterval extends ClosedInterval {

	private LEDClosedIntervalCollisionBehavior collisionBehavior;

	public LEDClosedInterval(int start, int end, LEDClosedIntervalCollisionBehavior collisionBehavior) {
		super(start, end);
		this.collisionBehavior = collisionBehavior;
	}

	public LEDClosedInterval(int start, int end) {
		this(start, end, LEDClosedIntervalCollisionBehavior.SELF_CANCEL);
	}

	public LEDClosedInterval withCollisionBehavior(LEDClosedIntervalCollisionBehavior collisionBehavior) {
		this.collisionBehavior = collisionBehavior;
		return this;
	}

	public LEDClosedIntervalCollisionBehavior getCollisionBehavior() {
		return collisionBehavior;
	}

	public LEDClosedInterval decideCollision(LEDClosedInterval incoming) {
		return switch (collisionBehavior) {
			case SELF_CANCEL -> incoming;
			case INCOMING_CANCEL -> this;
		};
	}

	public enum LEDClosedIntervalCollisionBehavior {
		SELF_CANCEL,
		INCOMING_CANCEL
	}
}
