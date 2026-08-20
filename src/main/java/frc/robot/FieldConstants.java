// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.driverstation.Alliance;

/**
 * Field geometry derived from the loaded AprilTag layout rather than hard-coded measurements.
 * WPILib has no built-in "starting position" constants -- the layout only publishes the field
 * bounds, so the alliance starting poses below are derived from those bounds.
 */
public final class FieldConstants {
  private static final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

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

  /**
   * Open patch near the field's center, used only to give MapleSim practice/teleop sessions a
   * clear spot to drive from -- it does NOT represent a legal match starting position (that's
   * {@link #middleStartFor}). {@code blueMiddleStart}/{@code redMiddleStart} sit against the
   * alliance wall on the field's width centerline, which in the MapleSim collision model overlaps
   * the center tower/stage structure, making manual practice driving difficult right at spawn.
   */
  public static Pose2d simPracticeSpawn(Alliance alliance) {
    return new Pose2d(
        fieldLengthMeters / 2.0,
        fieldWidthMeters / 2.0,
        alliance == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero);
  }

  private FieldConstants() {}
}
