// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Field geometry derived from the loaded AprilTag layout rather than hard-coded measurements.
 * WPILib has no built-in "starting position" constants -- the layout only publishes the field
 * bounds, so the alliance starting poses below are derived from those bounds.
 */
public final class FieldConstants {
  private static final AprilTagFieldLayout layout =
      AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();

  public static final double fieldLengthMeters = layout.getFieldLength();
  public static final double fieldWidthMeters = layout.getFieldWidth();

  /**
   * Robot spawns centered on the field's width, bumper against its own alliance wall, facing
   * downfield. TODO: confirm against the real game manual's starting-zone rules -- the layout only
   * gives field bounds, so the wall standoff is derived from our own bumper size.
   */
  public static final Pose2d blueMiddleStart = new Pose2d(
      Constants.kBumperLengthXMeters / 2.0,
      fieldWidthMeters / 2.0,
      Rotation2d.kZero);

  public static final Pose2d redMiddleStart = new Pose2d(
      fieldLengthMeters - (Constants.kBumperLengthXMeters / 2.0),
      fieldWidthMeters / 2.0,
      Rotation2d.k180deg);

  /** Returns the middle starting pose for the given alliance. */
  public static Pose2d middleStartFor(Alliance alliance) {
    return alliance == Alliance.Red ? redMiddleStart : blueMiddleStart;
  }

  private FieldConstants() {}
}
