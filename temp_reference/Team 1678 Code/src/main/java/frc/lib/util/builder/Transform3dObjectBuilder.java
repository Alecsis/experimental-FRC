package frc.lib.util.builder;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.axis3d.RotationAxis3d;
import frc.lib.util.axis3d.TranslationAxis3d;

public class Transform3dObjectBuilder implements ObjectBuilder<Transform3d> {

	private Distance x;
	private Distance y;
	private Distance z;

	private Angle roll;
	private Angle pitch;
	private Angle yaw;

	public Transform3dObjectBuilder(Transform3d transform) {
		this.x = transform.getMeasureX();
		this.y = transform.getMeasureY();
		this.z = transform.getMeasureZ();
		this.roll = transform.getRotation().getMeasureX();
		this.pitch = transform.getRotation().getMeasureY();
		this.yaw = transform.getRotation().getMeasureZ();
	}

	public Transform3dObjectBuilder(Distance distance, Angle angle) {
		this(new Transform3d(distance, distance, distance, new Rotation3d(angle, angle, angle)));
	}

	public Transform3dObjectBuilder() {
		this(BaseUnits.DistanceUnit.zero(), BaseUnits.AngleUnit.zero());
	}

	public Transform3dObjectBuilder withX(Distance x) {
		this.x = x;
		return this;
	}

	public Transform3dObjectBuilder withY(Distance y) {
		this.y = y;
		return this;
	}

	public Transform3dObjectBuilder withZ(Distance z) {
		this.z = z;
		return this;
	}

	public Transform3dObjectBuilder withOffset(TranslationAxis3d axis, Distance distance) {
		switch (axis) {
			case X:
				this.x = this.x.plus(distance);
				break;
			case Y:
				this.y = this.y.plus(distance);
				break;
			case Z:
				this.z = this.z.plus(distance);
				break;
		}
		return this;
	}

	public Transform3dObjectBuilder withOffsetX(Distance distance) {
		return withOffset(TranslationAxis3d.X, distance);
	}

	public Transform3dObjectBuilder withOffsetY(Distance distance) {
		return withOffset(TranslationAxis3d.Y, distance);
	}

	public Transform3dObjectBuilder withOffsetZ(Distance distance) {
		return withOffset(TranslationAxis3d.Z, distance);
	}

	public Transform3dObjectBuilder withRotationAxis(RotationAxis3d axis, Angle angle) {
		switch (axis) {
			case ROLL:
				this.roll = angle;
				break;
			case PITCH:
				this.pitch = angle;
				break;
			case YAW:
				this.yaw = angle;
				break;
		}
		return this;
	}

	public Transform3dObjectBuilder withTranslation(Translation3d translation) {
		return this.withX(translation.getMeasureX())
				.withY(translation.getMeasureY())
				.withZ(translation.getMeasureZ());
	}

	public Transform3dObjectBuilder withRoll(Angle roll) {
		this.roll = roll;
		return this;
	}

	public Transform3dObjectBuilder withPitch(Angle pitch) {
		this.pitch = pitch;
		return this;
	}

	public Transform3dObjectBuilder withYaw(Angle yaw) {
		this.yaw = yaw;
		return this;
	}

	public Transform3dObjectBuilder withRotation(Rotation3d rotation) {
		return this.withRoll(rotation.getMeasureX())
				.withPitch(rotation.getMeasureY())
				.withYaw(rotation.getMeasureZ());
	}

	@Override
	public Transform3d build() {
		return new Transform3d(x, y, z, new Rotation3d(roll, pitch, yaw));
	}
}
