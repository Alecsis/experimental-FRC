package frc.lib.util.builder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Pose2dObjectBuilder implements ObjectBuilder<Pose2d> {

	private Distance x;
	private Distance y;
	private Rotation2d rotation;

	public Pose2dObjectBuilder(Distance x, Distance y, Rotation2d rotation) {
		this.x = x;
		this.y = y;
		this.rotation = rotation;
	}

	public Pose2dObjectBuilder(Pose2d base) {
		this(base.getMeasureX(), base.getMeasureY(), base.getRotation());
	}

	public Pose2dObjectBuilder() {
		this(BaseUnits.DistanceUnit.zero(), BaseUnits.DistanceUnit.zero(), Rotation2d.kZero);
	}

	public Pose2dObjectBuilder withX(Distance x) {
		this.x = x;
		return this;
	}

	public Pose2dObjectBuilder withY(Distance y) {
		this.y = y;
		return this;
	}

	public Pose2dObjectBuilder withRotation(Rotation2d rotation) {
		this.rotation = rotation;
		return this;
	}

	public Pose2dObjectBuilder withRotation(Angle rotation) {
		this.rotation = new Rotation2d(rotation);
		return this;
	}

	@Override
	public Pose2d build() {
		return new Pose2d(x, y, rotation);
	}
}
