package frc.lib.util.builder;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import frc.lib.util.axis3d.RotationAxis3d;

public class Rotation3dObjectBuilder implements ObjectBuilder<Rotation3d> {

	private Angle roll = BaseUnits.AngleUnit.zero();
	private Angle pitch = BaseUnits.AngleUnit.zero();
	private Angle yaw = BaseUnits.AngleUnit.zero();

	public Rotation3dObjectBuilder() {}

	public Rotation3dObjectBuilder withRoll(Angle roll) {
		this.roll = roll;
		return this;
	}

	public Rotation3dObjectBuilder with(RotationAxis3d axis, Angle angle) {
		switch (axis) {
			case ROLL:
				this.roll = angle;
				break;
			case PITCH:
				this.pitch = angle;
				break;
			case YAW:
				this.yaw = angle;
		}
		return this;
	}

	public Rotation3dObjectBuilder withPitch(Angle pitch) {
		this.pitch = pitch;
		return this;
	}

	public Rotation3dObjectBuilder withYaw(Angle yaw) {
		this.yaw = yaw;
		return this;
	}

	@Override
	public Rotation3d build() {
		return new Rotation3d(roll, pitch, yaw);
	}
}
