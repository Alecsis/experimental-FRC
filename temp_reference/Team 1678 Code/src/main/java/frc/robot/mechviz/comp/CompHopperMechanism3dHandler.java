package frc.robot.mechviz.comp;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Distance;
import frc.lib.mechviz.mechanism.handler.Mechanism3dHandler;
import frc.lib.mechviz.mechanism.mech.Mechanism3d;
import frc.lib.util.ForwardDirection;
import frc.lib.util.builder.Transform3dObjectBuilder;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import java.util.function.Supplier;

public class CompHopperMechanism3dHandler extends Mechanism3dHandler {

	private final CompHopperMechanism3dHandlerConfig config;
	private final Mechanism3d hopperExpansion, hopperExtension;

	public CompHopperMechanism3dHandler(CompHopperMechanism3dHandlerConfig config) {
		this.config = config;
		hopperExpansion = new Mechanism3d(config.expansionOffset);
		hopperExtension = new Mechanism3d(config.extensionOffset);
		registerMech(hopperExpansion, hopperExtension);
	}

	@Override
	public void updatePositions() {
		hopperExpansion.setMutatedTransform(new Transform3dObjectBuilder()
				.withOffsetZ(config.expansionDirection.apply(
						ClimberConstants.converter.toDistance(Climber.mInstance.getPosition())))
				.build());
		hopperExtension.setMutatedTransform(new Transform3dObjectBuilder()
				.withOffsetX(config.extensionDirection.apply(config.extensionGetter.get()))
				.build());
	}

	public static class CompHopperMechanism3dHandlerConfig {

		public Pose3d expansionOffset, extensionOffset;

		public ForwardDirection expansionDirection;
		public ForwardDirection extensionDirection;

		public Supplier<Distance> expansionGetter;
		public Supplier<Distance> extensionGetter;
	}
}
