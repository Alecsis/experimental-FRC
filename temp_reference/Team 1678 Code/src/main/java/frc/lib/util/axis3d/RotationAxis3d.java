package frc.lib.util.axis3d;

public enum RotationAxis3d implements Axis3dConvertable {
	ROLL,
	PITCH,
	YAW;

	@Override
	public Axis3d toAxis3d() {
		return switch (this) {
			case ROLL -> Axis3d.ROLL;
			case PITCH -> Axis3d.PITCH;
			case YAW -> Axis3d.YAW;
		};
	}
}
