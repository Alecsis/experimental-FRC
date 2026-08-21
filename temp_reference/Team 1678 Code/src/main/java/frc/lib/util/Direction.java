package frc.lib.util;

public enum Direction {
	FORWARD,
	REVERSE,
	LEFT,
	RIGHT;

	public boolean isReversed() {
		return this == REVERSE;
	}

	public boolean isFoward() {
		return this == FORWARD;
	}
}
