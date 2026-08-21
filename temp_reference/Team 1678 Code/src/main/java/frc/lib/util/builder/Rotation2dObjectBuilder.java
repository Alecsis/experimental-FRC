package frc.lib.util.builder;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Rotation2dObjectBuilder implements ObjectBuilder<Rotation2d> {

	private Angle angle = BaseUnits.AngleUnit.zero();

	public Rotation2dObjectBuilder() {}

	public Rotation2dObjectBuilder withAngle(Angle angle) {
		this.angle = angle;
		return this;
	}

	public Rotation2dObjectBuilder withAngle(Translation2d translation) {
		this.angle = translation.getAngle().getMeasure();
		return this;
	}

	public Rotation2dObjectBuilder withXY(Distance x, Distance y, DistanceUnit unit) {
		double xInUnit = x.in(unit);
		double yInUnit = y.in(unit);
		double norm = Math.hypot(xInUnit, yInUnit);

		if (!Double.isNaN(norm) && norm > 1e-6) {
			angle = Radians.of(Math.atan2(xInUnit, yInUnit));
		} else {
			angle = BaseUnits.AngleUnit.zero();
		}

		return this;
	}

	@Override
	public Rotation2d build() {
		return new Rotation2d(angle);
	}
}
