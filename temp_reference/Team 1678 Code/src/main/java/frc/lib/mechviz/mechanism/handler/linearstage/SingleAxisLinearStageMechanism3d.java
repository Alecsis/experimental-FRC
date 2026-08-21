package frc.lib.mechviz.mechanism.handler.linearstage;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.lib.mechviz.mechanism.mech.Mechanism3d;
import frc.lib.util.ForwardDirection;
import frc.lib.util.Util;
import frc.lib.util.axis3d.TranslationAxis3d;

public class SingleAxisLinearStageMechanism3d extends Mechanism3d {

	private final TranslationAxis3d travelAxis;
	private final Distance maxStageDistanceTravel;
	private final Distance startDistance;
	private final ForwardDirection direction;
	private Distance stageTravelDistance;

	public SingleAxisLinearStageMechanism3d(
			Pose3d offset,
			TranslationAxis3d travelAxis,
			ForwardDirection direction,
			Distance startDistance,
			Distance maxStageDistanceTravel) {
		super(offset);
		this.travelAxis = travelAxis;
		this.direction = direction;
		this.startDistance = startDistance;
		this.maxStageDistanceTravel = maxStageDistanceTravel;
	}

	public Translation3d getTranslationPositionOffset() {
		return Util.addToTranslation3d(
				super.getMutationOffset().getTranslation(),
				travelAxis,
				getStageDistanceTravelled().times(direction.getInvertMultiplier()));
	}

	public void setStageTravelDistance(Distance travelDistance) {
		this.stageTravelDistance = travelDistance;
	}

	public void setStageTravelDistanceToMaxDistance() {
		setStageTravelDistance(getMaxStageDistanceTravel());
	}

	public Distance getStageDistanceTravelled() {
		return stageTravelDistance;
	}

	public Distance getMaxStageDistanceTravel() {
		return maxStageDistanceTravel;
	}

	public Distance getStartDistance() {
		return startDistance;
	}

	@Override
	public Pose3d getPivotedPose() {
		return super.getOffsetPose()
				.plus(new Transform3d(
						Util.addToTranslation3d(new Translation3d(), travelAxis, stageTravelDistance),
						Rotation3d.kZero));
	}
}
