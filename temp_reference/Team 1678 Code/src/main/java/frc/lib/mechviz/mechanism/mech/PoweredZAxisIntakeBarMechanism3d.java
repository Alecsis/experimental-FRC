package frc.lib.mechviz.mechanism.mech;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import java.util.function.Supplier;

public class PoweredZAxisIntakeBarMechanism3d extends ZAxisIntakeBarMechanism3d {

	private final Supplier<Angle> suppliedAngle;

	public PoweredZAxisIntakeBarMechanism3d(Pose3d offsetPose, Supplier<Angle> pitchGetter) {
		super(offsetPose);
		this.suppliedAngle = pitchGetter;
	}

	public Angle getMechAngle() {
		return suppliedAngle.get();
	}
}
