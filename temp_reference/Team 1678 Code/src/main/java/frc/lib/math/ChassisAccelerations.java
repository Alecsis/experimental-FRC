package frc.lib.math;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;

/**
 * Represents the chassis accelerations of a robot in 2D planar space
 * with optional angular acceleration about the vertical axis.
 * <p>
 * This class provides utilities for frame transformations (field ↔ robot),
 * as well as basic operations such as addition and scaling.
 */
public class ChassisAccelerations {

	/** Linear acceleration along the X-axis (forward/backward) */
	public double m_ax;

	/** Linear acceleration along the Y-axis (sideways) */
	public double m_ay;

	/** Angular acceleration around the vertical axis (rotation) */
	public double m_alpha;

	/** Default constructor initializes all accelerations to zero */
	public ChassisAccelerations() {
		this(0.0, 0.0, 0.0);
	}

	/**
	 * Constructor for planar linear accelerations only (no angular acceleration).
	 *
	 * @param ax linear acceleration along X-axis
	 * @param ay linear acceleration along Y-axis
	 */
	public ChassisAccelerations(double ax, double ay) {
		this(ax, ay, 0.0);
	}

	/**
	 * Constructor for full 2D linear and angular accelerations.
	 *
	 * @param ax linear acceleration along X-axis
	 * @param ay linear acceleration along Y-axis
	 * @param ao angular acceleration around vertical axis
	 */
	public ChassisAccelerations(double ax, double ay, double ao) {
		this.m_ax = ax;
		this.m_ay = ay;
		this.m_alpha = ao;
	}

	/**
	 * Constructor for full 2D linear and angular acceleration
	 *
	 * @param ax linear acceleration along the X axis
	 * @param ay linear acceleration along the Y axis
	 * @param ao angular acceleration
	 */
	public ChassisAccelerations(LinearAcceleration ax, LinearAcceleration ay, AngularAcceleration ao) {
		this(ax.in(MetersPerSecondPerSecond), ay.in(MetersPerSecondPerSecond), ao.in(RadiansPerSecondPerSecond));
	}

	/** Getters */
	public double getAx() {
		return m_ax;
	}

	public double getAy() {
		return m_ay;
	}

	public double getAo() {
		return m_alpha;
	}

	/** Setters */
	public void setAx(double ax) {
		m_ax = ax;
	}

	public void setAy(double ay) {
		m_ay = ay;
	}

	public void setAo(double ao) {
		m_alpha = ao;
	}

	/**
	 * Returns the planar magnitude (norm) of the linear acceleration.
	 *
	 * @return sqrt(ax^2 + ay^2)
	 */
	public double getNorm() {
		return Math.hypot(m_ax, m_ay);
	}

	/**
	 * Rotates the acceleration from field-relative coordinates to robot-relative coordinates.
	 *
	 * @param robotAngle the current robot heading (rotation of robot relative to field)
	 * @return a new ChassisAccelerations in robot-relative frame
	 */
	public ChassisAccelerations toRobotRelative(Rotation2d robotAngle) {
		Translation2d rotated = new Translation2d(m_ax, m_ay).rotateBy(robotAngle.unaryMinus());
		return new ChassisAccelerations(rotated.getX(), rotated.getY(), m_alpha);
	}

	/**
	 * Rotates the acceleration from robot-relative coordinates to field-relative coordinates.
	 *
	 * @param robotAngle the current robot heading (rotation of robot relative to field)
	 * @return a new ChassisAccelerations in field-relative frame
	 */
	public ChassisAccelerations toFieldRelative(Rotation2d robotAngle) {
		Translation2d rotated = new Translation2d(m_ax, m_ay).rotateBy(robotAngle);
		return new ChassisAccelerations(rotated.getX(), rotated.getY(), m_alpha);
	}

	/**
	 * Adds another ChassisAccelerations to this one.
	 *
	 * @param other the other ChassisAccelerations
	 * @return a new ChassisAccelerations representing the sum
	 */
	public ChassisAccelerations plus(ChassisAccelerations other) {
		return new ChassisAccelerations(m_ax + other.m_ax, m_ay + other.m_ay, m_alpha + other.m_alpha);
	}

	/**
	 * Scales the accelerations by a constant factor.
	 *
	 * @param scalar the factor to scale by
	 * @return a new ChassisAccelerations representing the scaled accelerations
	 */
	public ChassisAccelerations times(double scalar) {
		return new ChassisAccelerations(m_ax * scalar, m_ay * scalar, m_alpha * scalar);
	}

	/**
	 * Returns a human-readable string representation of this acceleration.
	 */
	@Override
	public String toString() {
		return String.format("ChassisAccelerations(ax=%.3f, ay=%.3f, ao=%.3f)", m_ax, m_ay, m_alpha);
	}
}
