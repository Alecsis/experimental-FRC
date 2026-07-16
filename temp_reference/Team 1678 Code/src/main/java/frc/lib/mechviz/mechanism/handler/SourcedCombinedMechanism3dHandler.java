package frc.lib.mechviz.mechanism.handler;

import edu.wpi.first.math.Pair;
import frc.lib.mechviz.mechanism.mech.Mechanism3d;
import frc.lib.util.axis3d.Axis3d;

public class SourcedCombinedMechanism3dHandler extends Mechanism3dHandler {

	private final Mechanism3d source;
	private final Pair<Mechanism3d, Axis3d[]> effected[];

	public SourcedCombinedMechanism3dHandler(Mechanism3d source, Pair<Mechanism3d, Axis3d[]>... effected) {
		super();

		this.source = source;
		this.effected = effected;

		registerMech(source);
		for (Pair<Mechanism3d, ?> mech : effected) {
			registerMech(mech.getFirst());
		}
	}

	@Override
	public void updatePositions() {
		for (Pair<Mechanism3d, Axis3d[]> pair : effected) {
			Mechanism3d mech = pair.getFirst();
			Axis3d travelAxes[] = pair.getSecond();

			if (travelAxes.length == 0) {
				continue;
			}
		}
	}
}
