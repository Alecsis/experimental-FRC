package frc.lib.mechviz.mechanism.handler.linearstage;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.mechviz.mechanism.handler.Mechanism3dHandler;
import frc.lib.mechviz.mechanism.mech.Mechanism3d;
import frc.lib.util.ForwardDirection;
import frc.lib.util.MathHelpers;
import frc.lib.util.axis3d.RotationAxis3d;
import frc.lib.util.builder.Transform3dObjectBuilder;
import java.util.function.Supplier;

public class RotatingStringBarMechanism3dHandler extends Mechanism3dHandler {

	private final Mechanism3d barController;

	private final Supplier<Distance> stringExtensionGetter;
	private final Distance armLength;
	private final ForwardDirection direction;
	private final RotationAxis3d axis;

	public RotatingStringBarMechanism3dHandler(
			Mechanism3d barController,
			Supplier<Distance> stringExtensionGetter,
			Distance armLength,
			ForwardDirection direction,
			RotationAxis3d axis) {
		this.barController = barController;
		this.stringExtensionGetter = stringExtensionGetter;
		this.armLength = armLength;
		this.direction = direction;
		this.axis = axis;
		registerMech(barController);
	}

	public Distance getStringExtension() {
		return stringExtensionGetter.get();
	}

	public void setAngle(Angle angle) {
		barController.setMutatedTransform(new Transform3dObjectBuilder()
				.withRotationAxis(axis, angle.times(direction.getInvertMultiplier()))
				.build());
	}

	@Override
	public void updatePositions() {
		Distance stringExtension = getStringExtension();

		setAngle(Radians.of(MathHelpers.calculateTriangleAngleFromSides(
				stringExtension.in(Meters), armLength.in(Meters), armLength.in(Meters))));
	}
}
