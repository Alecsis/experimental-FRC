package frc.robot.shooting.verification;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.FieldLayout;
import frc.lib.util.tunablenumbers.TunableNumber;

public class ShotVerifierConstants {
	// Gravity constant
	public static final LinearVelocity kGravityConstant =
			Units.MetersPerSecond.of(-9.8); // TODO: yo why is this not an acceleration???
	public static final Distance kShooterAltitude = Units.Inches.of(25.0);
	public static final Angle kAcceptableHubAngleError = Units.Degrees.of(1.3);
	public static final Angle kAcceptableFerryAngleError = Units.Degrees.of(4.0);
	public static final Angle IDEAL_ANGLE_ERROR = Degrees.of(0.5);
	public static final Angle kAcceptableTiltError = Units.Degrees.of(5.0);
	public static final Time kAutoAlignmentMaxWaitTime = Units.Seconds.of(0.45);

	public static final Time LOOKAHEAD_TIME = Milliseconds.of(40.0);

	public static final TunableNumber LOOKAHEAD_TIME_TUNNABLE = new TunableNumber("Look Ahead Time Secs", 0.12);

	public static final TunableNumber TOLERRABLE_ANGLE_RANGE_TUNNABLE =
			new TunableNumber("Drive Tracker/Tolerrable Angle Degrees", 50.0);

	// temp is in range
	public static final Distance kAcceptableRange = Units.Meters.of(5.0);

	public static final Angle TOLERRABLE_ANGLE = Degrees.of(50.0);

	// Back = closer to blue DS
	// Front = further from blue DS
	// Left is from looking from blue DS
	// Right is from looking from blue DS
	public static final Translation2d blueFrontLeftHubCorner = new Translation2d(
			FieldLayout.kBlueHub.getMeasureX().plus(Units.Inches.of(23.5)).plus(Units.Inches.of(10.26)),
			FieldLayout.kFieldWidth.div(2).plus(Units.Inches.of(58.41).div(2)));

	public static final Translation2d blueFrontRightHubCorner = new Translation2d(
			FieldLayout.kBlueHub.getMeasureX().plus(Units.Inches.of(23.5)).plus(Units.Inches.of(10.26)),
			FieldLayout.kFieldWidth.div(2).minus(Units.Inches.of(58.41).div(2)));

	public static final Translation2d blueBackLeftHubCorner = new Translation2d(
			FieldLayout.kBlueHub.getMeasureX().minus(Units.Inches.of(23.5)),
			FieldLayout.kFieldWidth.div(2).plus(Units.Inches.of(58.41).div(2)));

	public static final Translation2d blueBackRightHubCorner = new Translation2d(
			FieldLayout.kBlueHub.getMeasureX().minus(Units.Inches.of(23.5)),
			FieldLayout.kFieldWidth.div(2).minus(Units.Inches.of(58.41).div(2)));
}
