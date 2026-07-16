package frc.lib.mechviz.mechanism.mech;

import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.MathHelpers;
import frc.lib.util.builder.Transform3dObjectBuilder;

public class UnpoweredZAxisIntakeBarMechanism3d extends ZAxisIntakeBarMechanism3d {

	private Distance lengthFromSelfToPowered = BaseUnits.DistanceUnit.zero();
	private final Angle poweredToSelfPositionCoeffecent;

	public UnpoweredZAxisIntakeBarMechanism3d(
			Pose3d offsetPose, Distance lengthFromSelfToPowered, Angle poweredToSelfPositionCoeffecent) {
		super(offsetPose);
		setMutatedTransform(Transform3d.kZero);
		this.lengthFromSelfToPowered = lengthFromSelfToPowered;
		this.poweredToSelfPositionCoeffecent = poweredToSelfPositionCoeffecent;
	}

	public void calculateAndSetOffsetFromBase(Angle poweredAngle) {
		Angle currentPosition = poweredAngle.times(poweredToSelfPositionCoeffecent.in(Radian));
		Translation2d off = MathHelpers.get2dCordsFromCircleAngle(lengthFromSelfToPowered, currentPosition);
		setMutatedTransform(new Transform3dObjectBuilder()
				.withX(lengthFromSelfToPowered.minus(off.getMeasureX()))
				.withZ(off.getMeasureY())
				.withPitch(currentPosition)
				.build());
	}
}
