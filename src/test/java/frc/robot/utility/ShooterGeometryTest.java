// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

/**
 * The robot-centre to shooter-origin transform, tested as pure geometry.
 *
 * <p>No HAL, no singletons, no physics: {@link ShooterGeometry} is deliberately a static function of
 * (robot pose, target, offset), which is what makes it testable at offsets the robot has not been
 * measured for yet. The production constants are still 0.0 -- the whole point of the first test
 * below -- so every non-zero case here goes through the explicit-offset overloads that carry the
 * single shared implementation.
 */
class ShooterGeometryTest {
  private static final double kEps = 1e-9;

  /** An arbitrary, clearly off-centre robot pose; nothing here depends on its particular values. */
  private static final Translation2d kHub = new Translation2d(8.0, 4.0);

  private static Pose2d poseAt(double x, double y, double headingDeg) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg));
  }

  /* ------------------------------------------------------- 1. zero offset is a no-op --------- */

  /**
   * The shipped constants are 0.0 until somebody measures the robot, and while they are, every
   * number this class produces must be BIT-IDENTICAL to the robot-centre arithmetic the code used
   * before the transform existed. This is what makes introducing the transform a no-behaviour-change
   * commit, and it will start failing the moment a real measurement is dropped in -- which is
   * correct, and is the signal to re-verify aim calibration.
   */
  @Test
  void zeroShooterOffsetReproducesTheOldRobotCentreGeometryAtEveryHeading() {
    assertEquals(0.0, ShooterGeometry.SHOOTER_FORWARD_OFFSET_METERS, 0.0,
        "the shipped forward offset must stay 0.0 until measured on the robot");
    assertEquals(0.0, ShooterGeometry.SHOOTER_LEFT_OFFSET_METERS, 0.0,
        "the shipped lateral offset must stay 0.0 until measured on the robot");

    for (double headingDeg : new double[] {0, 37, 90, 180, 270, -45}) {
      Pose2d pose = poseAt(2.5, 6.25, headingDeg);

      double oldDistance = pose.getTranslation().getDistance(kHub);
      double oldHeadingRad = kHub.minus(pose.getTranslation()).getAngle().getRadians();

      assertEquals(oldDistance, ShooterGeometry.shooterDistanceMeters(pose, kHub), kEps,
          "distance must be unchanged at heading " + headingDeg);
      assertEquals(oldHeadingRad, ShooterGeometry.aimHeading(pose, kHub).getRadians(), kEps,
          "aim heading must be unchanged at heading " + headingDeg);
      assertEquals(pose.getX(), ShooterGeometry.shooterPosition(pose).getX(), kEps);
      assertEquals(pose.getY(), ShooterGeometry.shooterPosition(pose).getY(), kEps);
    }
  }

  /* ------------------------------------------------- 2. a forward offset rotates correctly --- */

  /**
   * A purely FORWARD offset must follow the robot's nose: at 0 deg it adds to +X, at 90 deg to +Y,
   * at 180 deg it subtracts from +X, at 270 deg it subtracts from +Y. Getting the sense of the
   * rotation wrong is the single most likely way to implement this transform incorrectly, and it
   * would be invisible at heading 0.
   */
  @Test
  void forwardOffsetRotatesWithTheRobotHeading() {
    Translation2d forward = new Translation2d(0.30, 0.0);
    Translation2d origin = new Translation2d(4.0, 4.0);

    assertShooterPosition(origin, 0, forward, 4.30, 4.00);
    assertShooterPosition(origin, 90, forward, 4.00, 4.30);
    assertShooterPosition(origin, 180, forward, 3.70, 4.00);
    assertShooterPosition(origin, 270, forward, 4.00, 3.70);
  }

  /* ------------------------------------------------- 3. a lateral offset rotates correctly --- */

  /**
   * A purely LEFT offset is the forward case rotated a further 90 deg, per WPILib's +Y-is-left
   * convention. Tested separately because a transform that happened to swap the two components
   * would still pass the forward-only case at heading 0.
   */
  @Test
  void lateralOffsetRotatesWithTheRobotHeading() {
    Translation2d left = new Translation2d(0.0, 0.20);
    Translation2d origin = new Translation2d(4.0, 4.0);

    assertShooterPosition(origin, 0, left, 4.00, 4.20);
    assertShooterPosition(origin, 90, left, 3.80, 4.00);
    assertShooterPosition(origin, 180, left, 4.00, 3.80);
    assertShooterPosition(origin, 270, left, 4.20, 4.00);
  }

  /* ------------------------------------------ 4. distance and heading use the SHOOTER point -- */

  /**
   * The distance handed to the RPM lookup table must be measured from the shooter, not the robot
   * centre -- and with a 0.30 m forward offset pointed straight at the hub, the two differ by
   * exactly that offset. If this ever reads the robot-centre distance again, the LUT is being asked
   * for the wrong shot.
   */
  @Test
  void shotDistanceIsMeasuredFromTheShooterOriginNotTheRobotCentre() {
    Translation2d forward = new Translation2d(0.30, 0.0);
    Pose2d pose = poseAt(4.0, 4.0, 0); // nose pointed at a hub directly downfield
    Translation2d hub = new Translation2d(10.0, 4.0);

    double robotCentreDistance = pose.getTranslation().getDistance(hub);
    double shooterDistance = ShooterGeometry.shooterDistanceMeters(pose, hub, forward);

    assertEquals(6.0, robotCentreDistance, kEps, "precondition: the robot centre is 6 m out");
    assertEquals(5.70, shooterDistance, kEps,
        "the shooter sits 0.30 m closer, so the shot is 0.30 m shorter");
  }

  /**
   * Aiming must solve the same vector. With a purely LATERAL offset the robot centre and the
   * shooter are NOT collinear with the hub, so the bearing genuinely differs -- which is exactly
   * the error a robot-centre aim leaves on the field.
   */
  @Test
  void aimHeadingIsMeasuredFromTheShooterOriginNotTheRobotCentre() {
    Translation2d left = new Translation2d(0.0, 1.0);
    Pose2d pose = poseAt(0.0, 0.0, 0); // shooter therefore sits at (0, 1)
    Translation2d hub = new Translation2d(4.0, 0.0);

    assertEquals(0.0, kHubBearingFromRobotCentre(pose, hub), kEps,
        "precondition: from the robot centre this target is dead ahead");
    assertEquals(Math.atan2(-1.0, 4.0),
        ShooterGeometry.aimHeading(pose, hub, left).getRadians(), kEps,
        "from a shooter 1 m to the left, the same target is below the shooter axis");
  }

  /**
   * Distance and heading must come from ONE vector, so they can never describe different shots.
   * Reconstructing the shooter position from the (distance, heading) pair must land back on the
   * target exactly.
   */
  @Test
  void distanceAndAimHeadingDescribeTheSameVector() {
    Translation2d offset = new Translation2d(0.25, -0.12);

    for (double headingDeg : new double[] {0, 45, 130, 200, 315}) {
      Pose2d pose = poseAt(3.1, 5.4, headingDeg);
      Translation2d shooter = ShooterGeometry.shooterPosition(pose, offset);
      double distance = ShooterGeometry.shooterDistanceMeters(pose, kHub, offset);
      Rotation2d aim = ShooterGeometry.aimHeading(pose, kHub, offset);

      Translation2d rebuilt = shooter.plus(new Translation2d(distance, aim));
      assertEquals(kHub.getX(), rebuilt.getX(), 1e-9, "at heading " + headingDeg);
      assertEquals(kHub.getY(), rebuilt.getY(), 1e-9, "at heading " + headingDeg);
    }
  }

  /* -------------------------------------------------------------------------------- helpers -- */

  private static double kHubBearingFromRobotCentre(Pose2d pose, Translation2d target) {
    return target.minus(pose.getTranslation()).getAngle().getRadians();
  }

  private static void assertShooterPosition(Translation2d robotOrigin, double headingDeg,
      Translation2d robotToShooter, double expectedX, double expectedY) {
    Pose2d pose = new Pose2d(robotOrigin, Rotation2d.fromDegrees(headingDeg));
    Translation2d shooter = ShooterGeometry.shooterPosition(pose, robotToShooter);
    assertEquals(expectedX, shooter.getX(), 1e-9, "X at heading " + headingDeg);
    assertEquals(expectedY, shooter.getY(), 1e-9, "Y at heading " + headingDeg);
  }
}
