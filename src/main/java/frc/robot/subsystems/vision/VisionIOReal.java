// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.LimelightHelpers.PoseEstimate;

/** VisionIO implementation for real Limelight cameras. The only file allowed to touch LimelightHelpers/NetworkTables. */
public class VisionIOReal implements VisionIO {
  private final String[] cameraNames;

  public VisionIOReal(String[] cameraNames) {
    this.cameraNames = cameraNames;
  }

  @Override
  public void setRobotOrientation(double yawDegrees) {
    for (String name : cameraNames) {
      LimelightHelpers.SetRobotOrientation(name, yawDegrees, 0, 0, 0, 0, 0);
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    int n = cameraNames.length;
    inputs.hasTarget = new boolean[n];
    inputs.tagCount = new int[n];
    inputs.avgTagDist = new double[n];
    inputs.latencySeconds = new double[n];
    inputs.timestampSeconds = new double[n];
    inputs.poseX = new double[n];
    inputs.poseY = new double[n];
    inputs.poseThetaRad = new double[n];

    for (int i = 0; i < n; i++) {
      PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraNames[i]);

      inputs.hasTarget[i] = LimelightHelpers.validPoseEstimate(mt2);
      inputs.tagCount[i] = mt2.tagCount;
      inputs.avgTagDist[i] = mt2.avgTagDist;
      inputs.latencySeconds[i] = mt2.latency / 1000.0;
      inputs.timestampSeconds[i] = mt2.timestampSeconds;
      inputs.poseX[i] = mt2.pose.getX();
      inputs.poseY[i] = mt2.pose.getY();
      inputs.poseThetaRad[i] = mt2.pose.getRotation().getRadians();
    }
  }

  @Override
  public Optional<Pose2d> getPoseResetEstimate() {
    if (cameraNames.length == 0) {
      return Optional.empty();
    }
    PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraNames[0]);
    if (mt1 != null && mt1.tagCount > 0 && mt1.pose.getX() != 0) {
      return Optional.of(mt1.pose);
    }
    return Optional.empty();
  }
}
