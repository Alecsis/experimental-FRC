package frc.lib.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public enum ForwardDirection {
	POSITIVE(1),
	NEGATIVE(-1);
	public final double invertMultiplier;

	private ForwardDirection(double scalar) {
		this.invertMultiplier = scalar;
	}

	public double getInvertMultiplier() {
		return invertMultiplier;
	}

	public double apply(double scalar) {
		return scalar * invertMultiplier;
	}

	@SuppressWarnings("unchecked")
	public <M extends Measure<U>, U extends Unit> M apply(M measureScalar) {
		return (M) measureScalar.baseUnit().of(apply(measureScalar.baseUnitMagnitude()));
	}
}
