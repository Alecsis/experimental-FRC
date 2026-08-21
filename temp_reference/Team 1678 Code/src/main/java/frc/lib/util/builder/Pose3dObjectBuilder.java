package frc.lib.util.builder;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.axis3d.RotationAxis3d;
import frc.lib.util.axis3d.TranslationAxis3d;

public class Pose3dObjectBuilder implements ObjectBuilder<Pose3d> {

	private Distance x = BaseUnits.DistanceUnit.zero();
	private Distance y = BaseUnits.DistanceUnit.zero();
	private Distance z = BaseUnits.DistanceUnit.zero();

	private Angle roll = BaseUnits.AngleUnit.zero();
	private Angle pitch = BaseUnits.AngleUnit.zero();
	private Angle yaw = BaseUnits.AngleUnit.zero();

	public Pose3dObjectBuilder(Pose3d pose) {
		this.x = pose.getMeasureX();
		this.y = pose.getMeasureY();
		this.z = pose.getMeasureZ();
		this.roll = pose.getRotation().getMeasureX();
		this.pitch = pose.getRotation().getMeasureY();
		this.yaw = pose.getRotation().getMeasureZ();
	}

	public Pose3dObjectBuilder() {}

	public Pose3dObjectBuilder withX(Distance x) {
		this.x = x;
		return this;
	}

	public Pose3dObjectBuilder withX(double xMeters) {
		return withX(Meters.of(xMeters));
	}

	public Pose3dObjectBuilder withY(Distance y) {
		this.y = y;
		return this;
	}

	public Pose3dObjectBuilder withY(double yMeters) {
		return withY(Meters.of(yMeters));
	}

	public Pose3dObjectBuilder withZ(Distance z) {
		this.z = z;
		return this;
	}

	public Pose3dObjectBuilder withZ(double zMeters) {
		return withZ(Meters.of(zMeters));
	}

	public Pose3dObjectBuilder withTranslation(Translation3d translation) {
		this.x = translation.getMeasureX();
		this.y = translation.getMeasureY();
		this.z = translation.getMeasureZ();
		return this;
	}

	public Pose3dObjectBuilder withRoll(Angle roll) {
		this.roll = roll;
		return this;
	}

	public Pose3dObjectBuilder withRoll(double rollRadians) {
		return withRoll(Radians.of(rollRadians));
	}

	public Pose3dObjectBuilder withPitch(Angle pitch) {
		this.pitch = pitch;
		return this;
	}

	public Pose3dObjectBuilder withPitch(double pitchRadians) {
		return withPitch(Radians.of(pitchRadians));
	}

	public Pose3dObjectBuilder withYaw(Angle yaw) {
		this.yaw = yaw;
		return this;
	}

	public Pose3dObjectBuilder withYaw(double yawRadians) {
		return withYaw(Radians.of(yawRadians));
	}

	public Pose3dObjectBuilder withRotation(Rotation3d rotation) {
		this.roll = rotation.getMeasureX();
		this.pitch = rotation.getMeasureY();
		this.yaw = rotation.getMeasureZ();
		return this;
	}

	public Pose3dObjectBuilder times(double scalar) {
		return withX(x.times(scalar))
				.withY(y.times(scalar))
				.withZ(z.times(scalar))
				.withRoll(roll.times(scalar))
				.withPitch(pitch.times(scalar))
				.withYaw(yaw.times(scalar));
	}

	public Pose3dObjectBuilder unaryMinus() {
		return times(-1.0);
	}

	public Pose3dObjectBuilder withOffset(TranslationAxis3d axis, Distance distance) {
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

	public Pose3dObjectBuilder withOffsetX(Distance distance) {
		return withOffset(TranslationAxis3d.X, distance);
	}

	public Pose3dObjectBuilder withOffsetY(Distance distance) {
		return withOffset(TranslationAxis3d.Y, distance);
	}

	public Pose3dObjectBuilder withOffsetZ(Distance distance) {
		return withOffset(TranslationAxis3d.Z, distance);
	}

	public Pose3dObjectBuilder withOffset(RotationAxis3d axis, Angle angle) {
		switch (axis) {
			case ROLL:
				this.roll = this.roll.plus(angle);
				break;
			case PITCH:
				this.pitch = this.pitch.plus(angle);
				break;
			case YAW:
				this.yaw = this.yaw.plus(angle);
				break;
		}
		return this;
	}

	public Pose3dObjectBuilder withOffsetRoll(Angle angle) {
		return withOffset(RotationAxis3d.ROLL, angle);
	}

	public Pose3dObjectBuilder withOffsetPitch(Angle angle) {
		return withOffset(RotationAxis3d.PITCH, angle);
	}

	public Pose3dObjectBuilder withOffsetYaw(Angle angle) {
		return withOffset(RotationAxis3d.YAW, angle);
	}

	@Override
	public Pose3d build() {
		return new Pose3d(x, y, z, new Rotation3d(roll, pitch, yaw));
	}
}
