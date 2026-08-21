package frc.lib.mechviz.mechanism.handler.linearstage;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Distance;
import java.util.function.Supplier;

public class CascadingMultiSingleAxisLinearStageMechanism3dHandler
		extends MultiSingleAxisLinearStageMechanism3dHandler {

	private final SingleAxisLinearStageMechanism3d stages[];

	public CascadingMultiSingleAxisLinearStageMechanism3dHandler(
			Supplier<Distance> distanceTravelled, Distance startPosition, SingleAxisLinearStageMechanism3d... stages) {
		super(distanceTravelled, startPosition, stages);
		this.stages = new SingleAxisLinearStageMechanism3d[stages.length];
		for (int i = 0; i < stages.length; i++) {
			this.stages[i] = stages[stages.length - 1 - i];
		}
	}

	public boolean travel(SingleAxisLinearStageMechanism3d stage, Distance distance) {
		stage.setStageTravelDistance(distance);
		return true;
	}

	@Override
	public void updatePositions() {
		Distance positionTravelDistance = getDistanceTravelled();
		for (int i = 0; i < stages.length; i++) {
			SingleAxisLinearStageMechanism3d current = stages[i];
			if (current.getStartDistance().lte(positionTravelDistance)) {
				double dividen = i + 1;
				travel(current, positionTravelDistance.div(dividen));
			} else {
				travel(current, BaseUnits.DistanceUnit.zero());
			}
		}
	}
}
