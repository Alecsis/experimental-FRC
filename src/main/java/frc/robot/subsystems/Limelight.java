// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.LimelightHelpers.PoseEstimate;

@SuppressWarnings("unused")

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private final String[] cams;

  public Limelight(String... names) {
    this.cams = names;
  }

  public List<Measurement> getMeasurement(Pose2d robotPose) {
    List<Measurement> result = new ArrayList<>();
    for (String name : cams) {
      LimelightHelpers.SetRobotOrientation(name, robotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      PoseEstimate MT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

      if (MT2 != null && MT2.tagCount > 0 && MT2.avgTagDist < 5 && MT2.pose.getX() != 0 && MT2.pose.getY() != 0) {
        double trust = MT2.avgTagDist * (1 / MT2.tagCount);
        double dev = .1 * trust;

        result.add(new Measurement(MT2, VecBuilder.fill(dev, dev, 999999)));
      }
    }
    return result;
  }

  public static class Measurement {
    public final PoseEstimate poseEstimate;
    public final Matrix<N3, N1> standardDeviations;

    public Measurement(PoseEstimate MT2, Matrix<N3, N1> standardDeviations) {
      this.poseEstimate = MT2;
      this.standardDeviations = standardDeviations;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}