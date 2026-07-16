package frc.lib.mechviz.mechanism.handler.linearstage;

import edu.wpi.first.units.measure.Distance;
import java.util.function.Supplier;

public class DividingCascadingMultiSingleAxisLinearStageMechanism3dHandler
		extends MultiSingleAxisLinearStageMechanism3dHandler {

	public DividingCascadingMultiSingleAxisLinearStageMechanism3dHandler(
			Supplier<Distance> distanceTravelled, Distance startPosition, SingleAxisLinearStageMechanism3d... stages) {
		super(distanceTravelled, startPosition, stages);
	}

	@Override
	public void updatePositions() {
		for (int i = 0; i < stages.length; i++) {
			int divisor = stages.length - i;
			SingleAxisLinearStageMechanism3d stage = stages[i];
			Distance travelDistance = getDistanceTravelled();

			stage.setStageTravelDistance(travelDistance.div(divisor));
		}
	}
}
