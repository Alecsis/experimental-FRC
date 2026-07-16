package frc.lib.sim.field.rebuilt;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class FieldSimConstants {
	/** Shooter angle (fixed). */
	public static final Angle kShooterAngle = Units.Degrees.of(6.0);

	/** Radius of the shooter flywheel */
	public static final Distance kShooterFlywheelRadius = Units.Inches.of(2.0);

	/** Base simulation period (seconds). */
	public static final Time kBasePeriod = Units.Seconds.of(0.02); // 20 ms

	public static final Time kShootingWait = Units.Seconds.of(0.25);
	/** Small-velocity threshold used when clamping near-zero vertical motion. */
	public static final LinearVelocity kSmallVelocityThreshold = Units.MetersPerSecond.of(0.05);

	/** Shooter transform relative to robot pose */
	public static final Transform3d kShooterOffset = new Transform3d(
			new Translation3d(Units.Meters.of(0.252), Units.Meters.of(-0.000434), Units.Meters.of(0.668)),
			Rotation3d.kZero);

	/** Radius of a fuel */
	public static final Distance kFuelRadius = Units.Meters.of(0.075);

	/** Number of sub-ticks per simulation step for a fuel. */
	public static final int kFuelSubticks = 5;

	public static final LinearAcceleration kGravity = Units.MetersPerSecondPerSecond.of(9.806);

	/** Gravity vector applied to airborne fuels (m/s^2). */
	public static final Translation3d kGravityVector = new Translation3d(0, 0, -kGravity.in(MetersPerSecondPerSecond));

	/** Proportion of horizontal velocity lost per second while on the ground. */
	public static final double kFrictionCoefficient = 0.1;

	/** Interval between trajectory samples used for visualization (seconds). */
	public static final Time kFuelArcInterval = Units.Seconds.of(0.04);
}
