package frc.lib.mechviz.mechanism.mech;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;

public class PivotingMechanism3d extends Mechanism3d {

	private final Supplier<Transform3d> mutatedGetter;

	public PivotingMechanism3d(Pose3d offsetPose, Supplier<Transform3d> mutatedGetter) {
		super(offsetPose);
		this.mutatedGetter = mutatedGetter;
	}

	@Override
	public void loop() {
		super.loop();
		setMutatedTransform(mutatedGetter.get());
	}
}
