package frc.lib.util.axis3d;

import java.util.Optional;

public class AxisUtil {

	public static TranslationAxis3d getTranslationAxisFromRotationAxis(RotationAxis3d axis) {
		return switch (axis) {
			case ROLL -> TranslationAxis3d.X;
			case PITCH -> TranslationAxis3d.Y;
			case YAW -> TranslationAxis3d.Z;
		};
	}

	public static Optional<TranslationAxis3d> tryToGetTranslation(Axis3d axis) {
		return switch (axis) {
			case X -> Optional.of(TranslationAxis3d.X);
			case Y -> Optional.of(TranslationAxis3d.Y);
			case Z -> Optional.of(TranslationAxis3d.Z);
			default -> Optional.empty();
		};
	}

	public Optional<RotationAxis3d> tryToGetRotation(Axis3d axis) {
		return switch (axis) {
			case ROLL -> Optional.of(RotationAxis3d.ROLL);
			case PITCH -> Optional.of(RotationAxis3d.PITCH);
			case YAW -> Optional.of(RotationAxis3d.YAW);
			default -> Optional.empty();
		};
	}
}
