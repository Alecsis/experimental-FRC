// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * THE robot-centre to shooter-origin transform, and the only place it is defined.
 *
 * <p><b>The problem this solves.</b> Production vision shots size the flywheel from the distance
 * between the ROBOT POSE and the hub, and aim by pointing the robot centre at the hub. The physical
 * shooter is not at the robot centre, so both of those are wrong by however far the launcher sits
 * from centre -- and the error is heading-dependent, because the offset rotates with the robot.
 *
 * <p><b>The chain.</b> field robot pose -> rotate the robot-relative shooter offset by the robot
 * heading -> field shooter position -> vector from the shooter to the target. That ONE vector
 * supplies both the distance handed to the RPM lookup table and the heading used for aiming, so the
 * two can never disagree about where the shot comes from.
 *
 * <p><b>This is NOT a camera transform.</b> A camera-to-robot transform describes where a sensor
 * observes the field from and is consumed inside the pose estimator (for the Limelights here, it is
 * configured on the camera itself, not in this repository). This describes where a game piece
 * LEAVES the robot. They are different quantities about different hardware and must never be
 * substituted for one another.
 *
 * <p><b>Aiming is a fixed point, not a closed form.</b> Rotating the robot moves the shooter, so
 * "the robot heading at which the shooter points at the target" satisfies an implicit equation.
 * {@link #aimHeading} evaluates the vector from the shooter's CURRENT position, which the heading
 * controller then drives to convergence over successive ticks -- standard practice, and exact for
 * the degenerate zero-offset case below.
 */
public final class ShooterGeometry {
  /**
   * Distance from the robot centre to the shooter origin along the robot's +X (forward) axis,
   * metres. Positive is toward the front of the robot.
   *
   * <p><b>MEASURE ON ROBOT BEFORE FINAL AIM CALIBRATION.</b> Held at 0.0 deliberately so that every
   * distance and heading this class produces is bit-identical to the pre-existing robot-centre
   * behaviour until a real measurement replaces it. Do not guess this number from CAD.
   */
  public static final double SHOOTER_FORWARD_OFFSET_METERS = 0.0;

  /**
   * Distance from the robot centre to the shooter origin along the robot's +Y (left) axis, metres.
   * Positive is toward the left of the robot, per WPILib convention.
   *
   * <p><b>MEASURE ON ROBOT BEFORE FINAL AIM CALIBRATION.</b> Held at 0.0 for the same reason as
   * {@link #SHOOTER_FORWARD_OFFSET_METERS}.
   */
  public static final double SHOOTER_LEFT_OFFSET_METERS = 0.0;

  /** The shooter origin expressed in the ROBOT frame. Rotate by the robot heading to reach field. */
  public static final Translation2d ROBOT_TO_SHOOTER =
      new Translation2d(SHOOTER_FORWARD_OFFSET_METERS, SHOOTER_LEFT_OFFSET_METERS);

  private ShooterGeometry() {
  }

  /** The shooter origin in FIELD coordinates for a given robot pose. */
  public static Translation2d shooterPosition(Pose2d robotPose) {
    return shooterPosition(robotPose, ROBOT_TO_SHOOTER);
  }

  /** The one authoritative shot vector: from the shooter origin to a field target. */
  public static Translation2d shooterToTarget(Pose2d robotPose, Translation2d target) {
    return shooterToTarget(robotPose, target, ROBOT_TO_SHOOTER);
  }

  /** Length of {@link #shooterToTarget}, metres. This is what the RPM lookup table must be fed. */
  public static double shooterDistanceMeters(Pose2d robotPose, Translation2d target) {
    return shooterDistanceMeters(robotPose, target, ROBOT_TO_SHOOTER);
  }

  /** Bearing of {@link #shooterToTarget}. This is what aiming/Orbit must drive the heading to. */
  public static Rotation2d aimHeading(Pose2d robotPose, Translation2d target) {
    return aimHeading(robotPose, target, ROBOT_TO_SHOOTER);
  }

  /*
   * The explicit-offset forms below carry the ONE implementation of the transform; the production
   * forms above are thin delegates that pin ROBOT_TO_SHOOTER into them. They exist so the rotation
   * behaviour can be exercised at real, non-zero offsets while the shipped constants stay at 0.0
   * until somebody measures the robot -- without that, the only offset any test could ever cover
   * would be the degenerate one where the transform does nothing.
   */

  /** The shooter origin in FIELD coordinates, for an arbitrary robot-frame shooter offset. */
  public static Translation2d shooterPosition(Pose2d robotPose, Translation2d robotToShooter) {
    return robotPose.getTranslation().plus(robotToShooter.rotateBy(robotPose.getRotation()));
  }

  /** The shot vector for an arbitrary robot-frame shooter offset. */
  public static Translation2d shooterToTarget(
      Pose2d robotPose, Translation2d target, Translation2d robotToShooter) {
    return target.minus(shooterPosition(robotPose, robotToShooter));
  }

  /** Shot distance in metres for an arbitrary robot-frame shooter offset. */
  public static double shooterDistanceMeters(
      Pose2d robotPose, Translation2d target, Translation2d robotToShooter) {
    return shooterToTarget(robotPose, target, robotToShooter).getNorm();
  }

  /** Aim heading for an arbitrary robot-frame shooter offset. */
  public static Rotation2d aimHeading(
      Pose2d robotPose, Translation2d target, Translation2d robotToShooter) {
    return shooterToTarget(robotPose, target, robotToShooter).getAngle();
  }
}
