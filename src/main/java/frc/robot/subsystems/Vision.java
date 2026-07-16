// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("unused")
public class Vision extends SubsystemBase {
  private final Limelight limelight;
  private final CommandSwerveDrivetrain swerve;

  /** Creates a new Vision. */
  public Vision(Limelight limelight, CommandSwerveDrivetrain swerve) {
    this.limelight = limelight;
    this.swerve = swerve;
  }

  private double smoothDistance = 0.0;

  public Optional<Pose2d> getVisionPose(Pose2d currentRobotPose) {
    // return limelight.getMeasurement(currentRobotPose) //!!! CHANGE WHEN IN ROBOT
    // .map(m -> m.poseEstimate.pose);
    return Optional.of(currentRobotPose); // SIMULATION ONLY
  }

  public Translation2d hubTranslation() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return new Translation2d(Meters.of(11.915394), Meters.of(4.0132));
    } else {
      return new Translation2d(Meters.of(4.625594), Meters.of(4.0132));
    }
  }

  public OptionalDouble hubDistanceMeters(Pose2d currentRobotPose) {
    var poseOpt = getVisionPose(currentRobotPose);
    if (poseOpt.isEmpty()) {
      return OptionalDouble.empty();
    }
    Translation2d hub = hubTranslation();
    return OptionalDouble.of(poseOpt.get().getTranslation().getDistance(hub));
  }

  @Override
  public void periodic() {
    Pose2d currentPose = swerve.getState().Pose;

    hubDistanceMeters(currentPose).ifPresentOrElse(distanceMeters -> {
      double distanceFeet = Units.metersToFeet(distanceMeters);
      // smoothDistance = .7 * smoothDistance + .3 * distanceFeet;
      SmartDashboard.putNumber("Vision/Distance_To_Hub", distanceFeet);
      SmartDashboard.putString("Vision/convert", String.format("Distance = %.2f ft", distanceFeet));
      Logger.recordOutput("Hub Distance", String.format("Distance = %.2f ft", distanceFeet));
    }, () -> {
      SmartDashboard.putNumber("Vision/Distance_To_Hub", 0.0);
      SmartDashboard.putString("Vision/convert", "vision disconnected");
        Logger.recordOutput("Hub Distance", "vision disconnected");
    });
    // Logs Vision Data
  }
}
