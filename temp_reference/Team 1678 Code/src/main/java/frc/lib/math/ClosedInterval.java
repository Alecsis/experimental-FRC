package frc.lib.math;

import edu.wpi.first.hal.util.BoundaryException;
import frc.lib.util.MathHelpers;

public class ClosedInterval {

	protected final int start, end;

	public ClosedInterval(int start, int end) {
		this.start = start;
		this.end = end;
	}

	public int getStart() {
		return start;
	}

	public int getEnd() {
		return end;
	}

	public int getLength() {
		return end - start;
	}

	public int getIndex(int index) {
		int value = start + index;
		if (value > end) {
			throw new BoundaryException("Overflowing Closed Interval");
		}
		return value;
	}

	public ClosedInterval getFromIndexRange(int indexStart, int indexEnd) {
		int start = MathHelpers.clamp(indexStart + this.start, this.start, this.end);
		int end = MathHelpers.clamp(indexEnd + this.start, this.start, this.end);
		return new ClosedInterval(start, end);
	}

	public ClosedInterval getFromIndexRange(ClosedInterval indexInterval) {
		return getFromIndexRange(indexInterval.getStart(), indexInterval.getEnd());
	}

	public ClosedInterval getFromIndexRange(int endIndex) {
		return getFromIndexRange(0, endIndex);
	}

	public boolean collides(ClosedInterval other) {
		return Math.max(this.start, other.start) <= Math.min(this.end, other.end);
	}

	@Override
	public boolean equals(Object o) {
		if (o instanceof ClosedInterval) {
			ClosedInterval other = (ClosedInterval) o;
			return other.start == start && other.end == end;
		}
		return false;
	}
}
