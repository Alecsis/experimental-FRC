package frc.robot.autos;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.FieldLayout;

public class AutoConstants {

	public static final Distance kAutoLinearEpsilon = Units.Centimeters.of(4.0);
	public static final Angle kAutoAngleEpsilon = Units.Degrees.of(1.0);
	public static final Time kDelayTime = Units.Milliseconds.of(80); // for when you reach the pose
	public static final Time kDefaultTrajectoryTimeout =
			Units.Seconds.of(1.0); // for when you're unable to reach the pose
	public static final LinearVelocity OVER_BUMP_FORWARD_SPEED = Units.MetersPerSecond.of(3.0);
	public static final LinearVelocity COLLISION_CLEAR_SPEED = Units.MetersPerSecond.of(3.0);

	// Midfield contact recovery window.
	public static final Distance MIDFIELD_CONTACT_WINDOW_MIN_X = Units.Meters.of(5.6);
	public static final Distance MIDFIELD_CONTACT_WINDOW_MAX_X = Units.Meters.of(11.0);
	public static final Distance MIDFIELD_CONTACT_WINDOW_MIN_Y = Units.Meters.of(0.0);
	public static final Distance MIDFIELD_CONTACT_WINDOW_MAX_Y = FieldLayout.kFieldWidth;

	public static final Time MIDFIELD_CONTACT_HIT_DEBOUNCE = Units.Seconds.of(0.15);
	public static final double MIDFIELD_CONTACT_MAX_ACTUAL_SPEED_FRACTION_OF_EXPECTED = 0.45;
	public static final Distance MIDFIELD_CONTACT_MIN_DISTANCE_TO_FINAL_POSE = Units.Meters.of(1.0);

	public static final Distance MIDFIELD_RECOVERY_TRANSLATION_TOLERANCE = Units.Meters.of(0.25);
	public static final Angle MIDFIELD_RECOVERY_ROTATION_TOLERANCE = Units.Degrees.of(4.0);

	public static final Vector<N3> BUMP_STD_DEVS = VecBuilder.fill(0.02, 0.02, 9999999999.9999);
}
