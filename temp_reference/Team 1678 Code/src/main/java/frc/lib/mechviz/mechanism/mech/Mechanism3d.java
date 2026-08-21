package frc.lib.mechviz.mechanism.mech;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.util.Loopable;

public class Mechanism3d implements Loopable {

	private final Pose3d offsetPose;
	private Transform3d mutatedTransform;

	public Mechanism3d(Pose3d offsetPose) {
		this.offsetPose = offsetPose;
		mutatedTransform = Transform3d.kZero;
	}

	public Pose3d getPivotedPose() {
		return offsetPose.plus(mutatedTransform);
	}

	public void setMutatedTransform(Transform3d mutatedTransform) {
		this.mutatedTransform = mutatedTransform;
	}

	public Transform3d getMutationOffset() {
		return mutatedTransform;
	}

	public Pose3d getOffsetPose() {
		return offsetPose;
	}

	@Override
	public void loop() {}
}
