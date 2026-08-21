package frc.robot.mechviz.bigchung;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.mechviz.mechanism.handler.Mechanism3dHandler;
import frc.lib.mechviz.mechanism.mech.Mechanism3d;
import frc.lib.util.ForwardDirection;
import frc.lib.util.MathHelpers;
import frc.lib.util.builder.Transform3dObjectBuilder;
import frc.robot.subsystems.intakedeploy.IntakeDeploy;
import frc.robot.tracking.HopperTracker;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

public class BigChungusIntakeMechanism3dHandler extends Mechanism3dHandler {

	private final BigChungusIntakeMechanism3dHandlerConfig config;
	private final Mechanism3d deploy, extension, floor, bridgeLeft, bridgeRight;
	private final BigChungusIntakeMechanism3dHandlerInput inputs = new BigChungusIntakeMechanism3dHandlerInput();

	public BigChungusIntakeMechanism3dHandler(BigChungusIntakeMechanism3dHandlerConfig config) {
		this.config = config;
		deploy = new Mechanism3d(config.deployOffset);
		extension = new Mechanism3d(config.extensionOffset);
		floor = new Mechanism3d(config.floorOffset);
		bridgeLeft = new Mechanism3d(config.bridgeOffsetWithYDirection.plus(new Transform3dObjectBuilder()
				.withOffsetY(config.bridgeYAbsoluteOffset)
				.build()));
		bridgeRight = new Mechanism3d(config.bridgeOffsetWithYDirection.plus(new Transform3dObjectBuilder()
				.withOffsetY(config.bridgeYAbsoluteOffset.unaryMinus())
				.build()));
		registerMech(deploy, extension, floor);
		SmartDashboard.putData("Big Chungus Inputs", inputs);
	}

	public void foreachBridge(BiConsumer<Integer, Mechanism3d> function) {
		function.accept(0, bridgeLeft);
		function.accept(1, bridgeRight);
	}

	public void foreachBridge(Consumer<Mechanism3d> function) {
		function.accept(bridgeLeft);
		function.accept(bridgeRight);
	}

	public void updateInputs() {
		inputs.deployAngle = IntakeDeploy.mInstance.getPosition();

		inputs.extensionExtendedDistance = HopperTracker.mInstance.getHopperExtension();

		inputs.bridgeAngle = IntakeDeploy.mInstance.getPosition();
		inputs.floorAngle = HopperTracker.mInstance.getFloorAngle();
	}

	public Angle convertAngleToSelfSpace(Angle angle) {
		return angle.times(config.angleDirection.invertMultiplier);
	}

	public void setBridgeAngle(Angle bridgeAngle, Angle angle) {
		Angle floorAngle = inputs.floorAngle;

		Distance length =
				config.floorArmLength.times(MathHelpers.cos(floorAngle)).minus(config.floorArmLength);
		Distance height = config.floorArmLength.times(MathHelpers.sin(floorAngle));

		Transform3d next = new Transform3dObjectBuilder()
				.withOffsetX(length)
				.withOffsetZ(height)
				.withPitch(angle.unaryMinus())
				.build();

		bridgeLeft.setMutatedTransform(next);
		bridgeRight.setMutatedTransform(next);
	}

	public void setFloorAngle(Angle angle) {
		Angle desiredAngle = convertAngleToSelfSpace(angle);
		floor.setMutatedTransform(
				new Transform3dObjectBuilder().withPitch(desiredAngle).build());
		setBridgeAngle(desiredAngle, angle.unaryMinus());
	}

	@Override
	public void updatePositions() {
		updateInputs();
		deploy.setMutatedTransform(new Transform3dObjectBuilder()
				.withPitch(inputs.deployAngle.times(config.angleDirection.invertMultiplier))
				.build());
		extension.setMutatedTransform(new Transform3dObjectBuilder()
				.withOffsetX(inputs.extensionExtendedDistance)
				.withOffsetX(inputs.extensionExtendedDistance.times(config.translationDirection.invertMultiplier))
				.build());
		setFloorAngle(inputs.floorAngle);
	}

	public static class BigChungusIntakeMechanism3dHandlerConfig {

		public Pose3d deployOffset, extensionOffset, floorOffset, bridgeOffsetWithYDirection;
		public Distance bridgeYAbsoluteOffset, floorArmLength, intakeArmLength, bridgeArmLength;
		public ForwardDirection angleDirection, translationDirection;
	}

	public class BigChungusIntakeMechanism3dHandlerInput implements Sendable {
		public Angle deployAngle = BaseUnits.AngleUnit.zero(),
				floorAngle = BaseUnits.AngleUnit.zero(),
				bridgeAngle = BaseUnits.AngleUnit.zero();
		public Distance extensionExtendedDistance = BaseUnits.DistanceUnit.zero();

		@Override
		public void initSendable(SendableBuilder builder) {
			builder.addDoubleProperty("Angle Degrees/Deploy", () -> deployAngle.in(Degrees), null);
			builder.addDoubleProperty("Angle Degrees/Floor", () -> floorAngle.in(Degrees), null);
			builder.addDoubleProperty("Angle Degrees/Bridge ", () -> bridgeAngle.in(Degrees), null);
			builder.addDoubleProperty(
					"Extended Meters/Extension Extended Distance", () -> extensionExtendedDistance.in(Meters), null);
		}
	}
}
