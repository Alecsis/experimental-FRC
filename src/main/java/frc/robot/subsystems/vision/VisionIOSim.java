// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.system.Timer;

/**
 * VisionIO implementation backed by the drivetrain's simulated ground-truth pose. No
 * NetworkTables/vendor dependencies -- returns a single perfect synthetic MegaTag2-style
 * measurement each cycle, matching the trust-filter shape VisionIOReal produces.
 */
public class VisionIOSim implements VisionIO {
  private final Supplier<Pose2d> groundTruthPoseSupplier;

  public VisionIOSim(Supplier<Pose2d> groundTruthPoseSupplier) {
    this.groundTruthPoseSupplier = groundTruthPoseSupplier;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Pose2d pose = groundTruthPoseSupplier.get();

    // No ground truth yet (e.g. the drivetrain's sim thread hasn't constructed its physics sim on
    // this exact tick) -- report no target this cycle rather than publishing a stale/fake reading.
    if (pose == null) {
      inputs.hasTarget = new boolean[] { false };
      inputs.tagCount = new int[] { 0 };
      inputs.avgTagDist = new double[] { 0.0 };
      inputs.latencySeconds = new double[] { 0.0 };
      inputs.timestampSeconds = new double[] { Timer.getFPGATimestamp() };
      inputs.poseX = new double[] { 0.0 };
      inputs.poseY = new double[] { 0.0 };
      inputs.poseThetaRad = new double[] { 0.0 };
      return;
    }

    inputs.hasTarget = new boolean[] { true };
    inputs.tagCount = new int[] { 1 };
    inputs.avgTagDist = new double[] { 1.0 };
    inputs.latencySeconds = new double[] { 0.0 };
    inputs.timestampSeconds = new double[] { Timer.getFPGATimestamp() };
    inputs.poseX = new double[] { pose.getX() };
    inputs.poseY = new double[] { pose.getY() };
    inputs.poseThetaRad = new double[] { pose.getRotation().getRadians() };
  }
}
