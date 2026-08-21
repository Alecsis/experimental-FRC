package frc.lib.math;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;

public class KinematicHelpers {

	/**
	 * First-order (Euler) integration.
	 *
	 * Computes the new value of a quantity using its first derivative,
	 * assuming the derivative remains constant over the time step.
	 *
	 * Δx = v * dt
	 *
	 * @param derivative First derivative (v), e.g., velocity
	 * @param dt         Time step over which to integrate
	 * @return           Change in position after the time step
	 */
	public static double firstOrderApproximate(double derivative, double dt) {
		return derivative * dt;
	}

	/**
	 * First-order (Euler) integration.
	 *
	 * Computes the new value of a quantity using its first derivative,
	 * assuming the derivative remains constant over the time step.
	 *
	 * Δx = v * dt
	 * x_new = x + Δx
	 *
	 * @param measure    Current value (x), e.g., position
	 * @param derivative First derivative (v), e.g., velocity
	 * @param dt         Time step over which to integrate
	 * @return Updated value after dt (x_new)
	 */
	public static double firstOrderApproximate(double measure, double derivative, double dt) {
		return measure + firstOrderApproximate(derivative, dt);
	}

	/**
	 * Second-order (constant acceleration) integration.
	 *
	 * Computes the new value of a quantity assuming constant acceleration over the
	 * time step.
	 *
	 * Δx = v * dt + (1/2) * a * dt^2
	 * x_new = x + Δx
	 *
	 * @param derivative       First derivative (v), e.g., velocity
	 * @param secondDerivative Second derivative (a), e.g., acceleration
	 * @param dt               Time step over which to integrate (seconds)
	 * @return                 Change in position after the time step
	 */
	public static double secondOrderApproximate(double derivative, double secondDerivative, double dt) {
		return firstOrderApproximate(derivative, dt) + 0.5 * secondDerivative * dt * dt;
	}

	/**
	 * Second-order (constant acceleration) integration.
	 *
	 * Computes the new value of a quantity assuming constant acceleration over the
	 * time step.
	 *
	 * Δx = v * dt + (1/2) * a * dt^2
	 * x_new = x + Δx
	 *
	 * @param derivative       First derivative (v), e.g., velocity
	 * @param secondDerivative Second derivative (a), e.g., acceleration
	 * @param dt               Time step over which to integrate (seconds)
	 * @return Updated value after dt (x_new)
	 */
	public static double secondOrderApproximate(double measure, double derivative, double secondDerivative, double dt) {
		return measure + secondOrderApproximate(derivative, secondDerivative, dt);
	}

	public static Twist2d firstOrderApproximate(double vx, double vy, double vo, double dt) {
		return new Twist2d(firstOrderApproximate(vx, dt), firstOrderApproximate(vy, dt), firstOrderApproximate(vo, dt));
	}

	/**
	 *
	 * First-order (Euler) integration.
	 *
	 * Computes the change in position of a quantity using the robot's velocity,
	 * assuming the robot's velocity remains constant over the time step.
	 *
	 * @param driveSpeeds      Robot's robot centric chasis speeds
	 * @param dt               Time step over which to integrate (seconds)
	 * @return                 Twist2d representing the change in pose over dt seconds
	 */
	public static Twist2d firstOrderApproximate(ChassisSpeeds driveSpeeds, double dt) {
		return new Twist2d(
				firstOrderApproximate(driveSpeeds.vxMetersPerSecond, dt),
				firstOrderApproximate(driveSpeeds.vyMetersPerSecond, dt),
				firstOrderApproximate(driveSpeeds.omegaRadiansPerSecond, dt));
	}

	/**
	 * Second-order (constant acceleration) integration.
	 *
	 * Computes the new value of a quantity assuming constant acceleration over the
	 * time step.
	 *
	 * Δx = v * dt + (1/2) * a * dt^2
	 * x_new = x + Δx
	 *
	 * @return                  Twist2d representing the change in pose over dt seconds
	 */
	public static Twist2d secondOrderApproximate(
			double vx, double vy, double vo, double ax, double ay, double ao, double dt) {
		return new Twist2d(
				secondOrderApproximate(vx, ax, dt),
				secondOrderApproximate(vy, ay, dt),
				secondOrderApproximate(vo, ao, dt));
	}

	/**
	 * Second-order (constant acceleration) integration.
	 *
	 * Computes the new value of a quantity assuming constant acceleration over the
	 * time step.
	 *
	 * Δx = v * dt + (1/2) * a * dt^2
	 * x_new = x + Δx
	 *
	 * @param driveSpeeds       Robot's robot centric chasis speeds
	 * @param acceleration      Robot's acceleration
	 * @dt                      Time step (seconds)
	 * @return                  Twist2d representing the change in pose over dt seconds
	 */
	public static Twist2d secondOrderApproximate(
			ChassisSpeeds driveSpeeds, ChassisAccelerations acceleration, double dt) {
		double vx = driveSpeeds.vxMetersPerSecond;
		double vy = driveSpeeds.vyMetersPerSecond;
		double vo = driveSpeeds.omegaRadiansPerSecond;

		double ax = acceleration.m_ax;
		double ay = acceleration.m_ay;
		double ao = acceleration.m_alpha;

		return secondOrderApproximate(vx, vy, vo, ax, ay, ao, dt);
	}

	/**
	 * Second-order (constant acceleration) integration.
	 *
	 * Computes the new value of a quantity assuming constant acceleration over the
	 * time step.
	 *
	 * Δx = v * dt + (1/2) * a * dt^2
	 * x_new = x + Δx
	 *
	 * @param driveSpeeds       Robot's robot centric chasis speeds
	 * @param acceleration      Robot's acceleration
	 * @dt                      Time step
	 * @return                  Twist2d representing the change in pose over dt seconds
	 */
	public static Twist2d secondOrderApproximate(
			ChassisSpeeds driveSpeeds, ChassisAccelerations acceleration, Time dt) {
		return secondOrderApproximate(driveSpeeds, acceleration, dt.in(Seconds));
	}

	/**
	 *
	 * First-order (Euler) integration.
	 *
	 * Computes the change in position of a quantity using the robot's velocity,
	 * assuming the robot's velocity remains constant over the time step.
	 *
	 * @param pose             Current pose (Meters & Radians)
	 * @param driveSpeeds      Robot's robot centric chasis speeds
	 * @param dt               Time step over which to integrate (seconds)
	 * @return                 Pose after the time step
	 */
	public static Pose2d firstOrderApproximate(Pose2d pose, ChassisSpeeds driveSpeeds, double dt) {
		return pose.exp(firstOrderApproximate(driveSpeeds, dt));
	}

	/**
	 *
	 * First-order (Euler) integration.
	 *
	 * Computes the change in position of a quantity using the robot's velocity,
	 * assuming the robot's velocity remains constant over the time step.
	 *
	 * @param pose             Current pose (Meters & Radians)
	 * @param driveSpeeds      Robot's robot centric chasis speeds
	 * @param dt               Time step over which to integrate
	 * @return                 Twist2d representing the change in pose over dt seconds
	 */
	public static Pose2d firstOrderApproximate(Pose2d pose, ChassisSpeeds driveSpeeds, Time dt) {
		return firstOrderApproximate(pose, driveSpeeds, dt.in(Seconds));
	}

	/**
	 * Second-order (constant acceleration) integration.
	 *
	 * Computes the new value of a quantity assuming constant acceleration over the
	 * time step.
	 *
	 * Δx = v * dt + (1/2) * a * dt^2
	 * x_new = x + Δx
	 *
	 * @param pose              Current pose
	 * @param driveSpeeds       Robot's robot centric chasis speeds
	 * @param acceleration      Robot's acceleration
	 * @dt                      Time step
	 * @return                  Twist2d representing the change in pose over dt seconds
	 */
	public static Pose2d secondOrderApproximate(
			Pose2d pose, ChassisSpeeds driveSpeeds, ChassisAccelerations acceleration, double dt) {
		return pose.exp(secondOrderApproximate(driveSpeeds, acceleration, dt));
	}

	/**
	 * Second-order (constant acceleration) integration.
	 *
	 * Computes the new value of a quantity assuming constant acceleration over the
	 * time step.
	 *
	 * Δx = v * dt + (1/2) * a * dt^2
	 * x_new = x + Δx
	 *
	 * @param pose              Current pose
	 * @param driveSpeeds       Robot's robot centric chasis speeds
	 * @param acceleration      Robot's acceleration
	 * @dt                      Time step
	 * @return                  Twist2d representing the change in pose over dt seconds
	 */
	public static Pose2d secondOrderApproximate(
			Pose2d pose, ChassisSpeeds driveSpeeds, ChassisAccelerations acceleration, Time dt) {
		return secondOrderApproximate(pose, driveSpeeds, acceleration, dt.in(Seconds));
	}
}
