package frc.lib.util.builder;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.axis3d.TranslationAxis3d;

public class Translation3dObjectBuilder implements ObjectBuilder<Translation3d> {

	private Distance x;
	private Distance y;
	private Distance z;

	public Translation3dObjectBuilder(Distance x, Distance y, Distance z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public Translation3dObjectBuilder(Translation3d base) {
		this(base.getMeasureX(), base.getMeasureY(), base.getMeasureZ());
	}

	public Translation3dObjectBuilder() {
		this(BaseUnits.DistanceUnit.zero(), BaseUnits.DistanceUnit.zero(), BaseUnits.DistanceUnit.zero());
	}

	public Translation3dObjectBuilder with(TranslationAxis3d axis, Distance distance) {
		switch (axis) {
			case X:
				this.x = distance;
				break;
			case Y:
				this.y = distance;
				break;
			case Z:
				this.z = distance;
				break;
		}
		return this;
	}

	public Translation3dObjectBuilder withX(Distance x) {
		this.x = x;
		return this;
	}

	public Translation3dObjectBuilder withY(Distance y) {
		this.y = y;
		return this;
	}

	public Translation3dObjectBuilder withZ(Distance z) {
		this.z = z;
		return this;
	}

	@Override
	public Translation3d build() {
		return new Translation3d(x, y, z);
	}
}
