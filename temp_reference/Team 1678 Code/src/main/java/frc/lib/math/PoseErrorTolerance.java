package frc.lib.math;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public record PoseErrorTolerance(Distance linearErrorTolerance, Angle angularErrorTolerance) {
	public PoseErrorTolerance(double linearErrorTolerance, Rotation2d angularErrorTolerance) {
		this(Meters.of(linearErrorTolerance), angularErrorTolerance.getMeasure());
	}

	public boolean atPose(Pose2d expected, Pose2d actual) {
		var linearError = expected.getTranslation().getDistance(actual.getTranslation());

		// Linear error within tolerance
		return MathUtil.isNear(0, linearError, linearErrorTolerance.in(Meters))
				&&
				// Rotation error within tolerance
				MathUtil.isNear(
						expected.getRotation().getDegrees(),
						actual.getRotation().getDegrees(),
						angularErrorTolerance.in(Degree),
						-180,
						180);
	}
}
