package frc.lib.util.axis3d;

public enum TranslationAxis3d implements Axis3dConvertable {
	X,
	Y,
	Z;

	public Axis3d toAxis3d() {
		return switch (this) {
			case X -> Axis3d.X;
			case Y -> Axis3d.Y;
			case Z -> Axis3d.Z;
		};
	}
}
