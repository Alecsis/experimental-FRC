package frc.lib.mechviz.mechanism.handler.linearstage;

import edu.wpi.first.units.measure.Distance;
import frc.lib.mechviz.mechanism.handler.Mechanism3dHandler;
import java.util.function.Supplier;

public abstract class MultiSingleAxisLinearStageMechanism3dHandler extends Mechanism3dHandler {

	protected final SingleAxisLinearStageMechanism3d stages[];
	protected final Supplier<Distance> distanceTravelledGetter;

	public MultiSingleAxisLinearStageMechanism3dHandler(
			Supplier<Distance> distanceTravelled, Distance startPosition, SingleAxisLinearStageMechanism3d... stages) {
		for (SingleAxisLinearStageMechanism3d stage : stages) {
			stage.setStageTravelDistance(startPosition);
		}
		registerMech(stages);
		this.stages = stages;
		this.distanceTravelledGetter = distanceTravelled;
	}

	public Distance getDistanceTravelled() {
		return distanceTravelledGetter.get();
	}

	public SingleAxisLinearStageMechanism3d[] getStages() {
		return stages;
	}

	public int getStagesSize() {
		return stages.length;
	}
}
