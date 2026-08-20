// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import org.wpilib.math.geometry.Pose2d;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean[] hasTarget = new boolean[0];
    public int[] tagCount = new int[0];
    public double[] avgTagDist = new double[0];
    public double[] latencySeconds = new double[0];
    public double[] timestampSeconds = new double[0];
    public double[] poseX = new double[0];
    public double[] poseY = new double[0];
    public double[] poseThetaRad = new double[0];
  }

  /** Updates the set of loggable inputs. Arrays are indexed by camera, in configured order. */
  default void updateInputs(VisionIOInputs inputs) {}

  /** Publishes the robot's current field-relative heading for MegaTag2 orientation-assisted pose estimation. */
  default void setRobotOrientation(double yawDegrees) {}

  /** Sets the camera's internal IMU fusion mode (Limelight IMU modes 0-4). No-op without a real IMU-fused camera. */
  default void setIMUMode(int mode) {}

  /** Sets the internal-IMU assist weighting used by MegaTag2 orientation fusion. No-op without a real IMU-fused camera. */
  default void setIMUAssistAlpha(double alpha) {}

  /**
   * Gets a MegaTag1 pose estimate suitable for seeding odometry on reset. Deliberately independent
   * of MegaTag2 (and {@link #setRobotOrientation}), since MT2 requires an already-correct yaw --
   * which is exactly what's unknown at reset time.
   */
  default Optional<Pose2d> getPoseResetEstimate() {
    return Optional.empty();
  }
}
