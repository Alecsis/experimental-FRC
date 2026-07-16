package frc.lib.mechviz.mechanism.handler;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.mechviz.mechanism.mech.Mechanism3d;
import frc.lib.util.ForwardDirection;
import frc.lib.util.axis3d.AxisUtil;
import frc.lib.util.axis3d.RotationAxis3d;
import frc.lib.util.axis3d.TranslationAxis3d;
import frc.lib.util.builder.Rotation3dObjectBuilder;
import frc.lib.util.builder.Transform3dObjectBuilder;
import frc.lib.util.builder.Translation3dObjectBuilder;
import java.util.function.Supplier;

public class RotatingTranslationMechanism3dHandler extends Mechanism3dHandler {

	private final Mechanism3d rotatingMechanism;
	private final Mechanism3d effectedMechanisms[];

	private final Supplier<Angle> rotationGetter;
	private final Distance armLength;
	private final RotationAxis3d rotatingAxis;
	private final ForwardDirection direction;

	public RotatingTranslationMechanism3dHandler(
			Mechanism3d rotatingMechanism,
			Supplier<Angle> rotationGetter,
			Distance armLength,
			RotationAxis3d rotatingAxis,
			ForwardDirection direction,
			Mechanism3d... effected) {
		this.rotatingMechanism = rotatingMechanism;
		this.rotationGetter = rotationGetter;
		this.armLength = armLength;
		this.rotatingAxis = rotatingAxis;
		this.direction = direction;
		this.effectedMechanisms = effected;

		registerMech(rotatingMechanism);
		registerMech(effected);
	}

	public Angle getRotation() {
		return rotationGetter.get();
	}

	public void setRotatingMechanismRotation(Angle angle) {
		rotatingMechanism.setMutatedTransform(new Transform3dObjectBuilder()
				.withRotationAxis(rotatingAxis, angle.times(direction.getInvertMultiplier()))
				.build());
	}

	public TranslationAxis3d getVerticalAxis(TranslationAxis3d horizontalAxis) {
		return switch (horizontalAxis) {
			case X -> TranslationAxis3d.Z;
			case Y -> TranslationAxis3d.X;
			case Z -> TranslationAxis3d.Y;
		};
	}

	public Translation3d getEffectedMechanismTranslation(
			Mechanism3d mechanism,
			TranslationAxis3d horizontalAxis,
			TranslationAxis3d verticalAxis,
			Distance radius,
			Angle rotation) {
		return new Translation3dObjectBuilder()
				.with(
						horizontalAxis,
						radius.times(Math.cos(rotation.in(Radians))).minus(radius))
				.with(verticalAxis, radius.times(Math.sin(rotation.in(Radians))))
				.build();
	}

	public Rotation3d getEffectedMechanismRotation(Mechanism3d mechanism, RotationAxis3d rotatingAxis, Angle rotation) {
		return new Rotation3dObjectBuilder()
				.with(rotatingAxis, rotation.times(direction.getInvertMultiplier()))
				.build();
	}

	public void setEffectedTranslation(Angle rotation) {

		Distance radius = armLength;

		TranslationAxis3d horizontalAxis = AxisUtil.getTranslationAxisFromRotationAxis(rotatingAxis);
		TranslationAxis3d verticalAxis = getVerticalAxis(horizontalAxis);

		for (Mechanism3d mechanism : effectedMechanisms) {
			Translation3d additionTranslation =
					getEffectedMechanismTranslation(mechanism, horizontalAxis, verticalAxis, radius, rotation);
			Rotation3d additionRotation = getEffectedMechanismRotation(mechanism, rotatingAxis, rotation);
			mechanism.setMutatedTransform(new Transform3d(additionTranslation, additionRotation));
		}
	}

	@Override
	public void updatePositions() {
		Angle rotation = getRotation();
		setRotatingMechanismRotation(rotation);
	}
}
