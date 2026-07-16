// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.TreeMap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Singleton Vision subsystem. Applies the MegaTag2 trust filter to raw camera inputs and pushes
 * accepted measurements directly into the drivetrain's pose estimator.
 */
public class Vision extends SubsystemBase {
  private static final double kMaxOmegaRadPerSec = 2 * Math.PI;

  private final AprilTagFieldLayout fieldLayout =
      AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();
  private final int[] redHubIds = { 2, 5, 8, 9, 10, 11 };
  private final int[] blueHubIds = { 18, 21, 24, 25, 26, 27 };
  private final int[] redPassIds = { 9, 10, 15, 16 };
  private final int[] bluePassIds = { 25, 26, 31, 32 };

  private static final TreeMap<Double, Double> RPMLUT = new TreeMap<>();
  static {
    RPMLUT.put(12.4, 1625.0);
    RPMLUT.put(8.8, 1475.0);
    RPMLUT.put(14.5, 2150.0);
    RPMLUT.put(10.6, 1550.0);
    RPMLUT.put(7.5, 1500.0);
    RPMLUT.put(11.3, 1650.0);
  }

  private static Vision instance;

  /** Creates (on first call) or returns the Vision Singleton. */
  public static Vision getInstance(CommandSwerveDrivetrain drivetrain) {
    if (instance == null) {
      instance = new Vision(
          Constants.currentMode == Constants.Mode.REAL
              ? new VisionIOReal(Constants.visionCameraNames)
              : new VisionIOSim(() -> drivetrain.getState().Pose),
          drivetrain);
    }
    return instance;
  }

  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final CommandSwerveDrivetrain drivetrain;

  /** Creates a new Vision. Use {@link #getInstance(CommandSwerveDrivetrain)} instead of constructing directly. */
  private Vision(VisionIO io, CommandSwerveDrivetrain drivetrain) {
    this.io = io;
    this.drivetrain = drivetrain;
  }

  public Optional<Translation2d> getHubPosition() {
    DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
    int[] validIds = alliance == DriverStation.Alliance.Red ? redHubIds : blueHubIds;

    double totalX = 0;
    double totalY = 0;
    int count = 0;
    for (int id : validIds) {
      var pose = fieldLayout.getTagPose(id);
      if (pose.isPresent()) {
        Translation2d t = pose.get().toPose2d().getTranslation();
        totalX += t.getX();
        totalY += t.getY();
        count++;
      }
    }

    if (count == 0) {
      return Optional.empty();
    }
    return Optional.of(new Translation2d(totalX / count, totalY / count));
  }

  public Optional<Rotation2d> getHubHeading() {
    Optional<Translation2d> hubPosOpt = getHubPosition();
    if (hubPosOpt.isEmpty()) {
      return Optional.empty();
    }
    Translation2d delta = hubPosOpt.get().minus(drivetrain.getState().Pose.getTranslation());
    return Optional.of(delta.getAngle());
  }

  public Optional<Translation2d> getPassTargetPosition() {
    DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
    int[] validIds = alliance == DriverStation.Alliance.Red ? redPassIds : bluePassIds;

    Translation2d sum = new Translation2d();
    int count = 0;
    for (int id : validIds) {
      var pose = fieldLayout.getTagPose(id);
      if (pose.isPresent()) {
        sum = sum.plus(pose.get().toPose2d().getTranslation());
        count++;
      }
    }

    if (count == 0) {
      return Optional.empty();
    }
    return Optional.of(sum.div(count));
  }

  public Optional<Rotation2d> getPassHeading() {
    Optional<Translation2d> targetOpt = getPassTargetPosition();
    if (targetOpt.isEmpty()) {
      return Optional.empty();
    }
    Translation2d delta = targetOpt.get().minus(drivetrain.getState().Pose.getTranslation());
    return Optional.of(delta.getAngle());
  }

  /** MegaTag1-based pose estimate suitable for seeding odometry on reset. See {@link VisionIO#getPoseResetEstimate}. */
  public Optional<Pose2d> getPoseResetEstimate() {
    return io.getPoseResetEstimate();
  }

  private Optional<Pose2d> getVisionPose(Pose2d currentRobotPose) {
    return Optional.of(currentRobotPose);
  }

  public OptionalDouble hubDistanceMeters(Pose2d currentRobotPose) {
    var poseOpt = getVisionPose(currentRobotPose);
    var hubOpt = getHubPosition();
    if (poseOpt.isEmpty() || hubOpt.isEmpty()) {
      return OptionalDouble.empty();
    }
    return OptionalDouble.of(poseOpt.get().getTranslation().getDistance(hubOpt.get()));
  }

  public OptionalDouble calculateRPM() {
    OptionalDouble distanceMetersOpt = hubDistanceMeters(drivetrain.getState().Pose);
    if (distanceMetersOpt.isEmpty()) {
      return OptionalDouble.empty();
    }
    double distanceFeet = Units.metersToFeet(distanceMetersOpt.getAsDouble());
    double rpm = interpolateRPM(distanceFeet);
    rpm = MathUtil.clamp(rpm, 1200, 6000);
    return OptionalDouble.of(rpm);
  }

  private static double interpolateRPM(double distanceFeet) {
    if (distanceFeet <= RPMLUT.firstKey()) {
      return RPMLUT.firstEntry().getValue();
    }
    if (distanceFeet >= RPMLUT.lastKey()) {
      return RPMLUT.lastEntry().getValue();
    }
    var lo = RPMLUT.floorEntry(distanceFeet);
    var hi = RPMLUT.ceilingEntry(distanceFeet);
    double t = (distanceFeet - lo.getKey()) / (hi.getKey() - lo.getKey());
    return lo.getValue() + t * (hi.getValue() - lo.getValue());
  }

  private void fuseMeasurements() {
    double omega = drivetrain.getState().Speeds.omegaRadiansPerSecond;
    if (Math.abs(omega) > kMaxOmegaRadPerSec) {
      return;
    }

    for (int i = 0; i < inputs.hasTarget.length; i++) {
      if (!inputs.hasTarget[i]
          || inputs.tagCount[i] <= 0
          || inputs.avgTagDist[i] >= Constants.kVisionMaxTagDistMeters
          || inputs.poseX[i] == 0
          || inputs.poseY[i] == 0) {
        continue;
      }

      double trust = inputs.avgTagDist[i] * (1.0 / inputs.tagCount[i]);
      double dev = Constants.kVisionStdDevCoefficient * trust;
      Pose2d measuredPose = new Pose2d(
          inputs.poseX[i], inputs.poseY[i], new Rotation2d(inputs.poseThetaRad[i]));

      drivetrain.addVisionMeasurement(
          measuredPose, inputs.timestampSeconds[i], VecBuilder.fill(dev, dev, 999999));
    }
  }

  @Override
  public void periodic() {
    Pose2d currentPose = drivetrain.getState().Pose;

    io.setRobotOrientation(currentPose.getRotation().getDegrees());
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    fuseMeasurements();

    hubDistanceMeters(currentPose).ifPresentOrElse(distanceMeters -> {
      double distanceFeet = Units.metersToFeet(distanceMeters);
      SmartDashboard.putNumber("Vision/Distance_To_Hub", distanceFeet);
      SmartDashboard.putString("Vision/convert", String.format("Distance = %.2f ft", distanceFeet));
      Logger.recordOutput("Hub Distance", String.format("Distance = %.2f ft", distanceFeet));
    }, () -> {
      SmartDashboard.putNumber("Vision/Distance_To_Hub", 0.0);
      SmartDashboard.putString("Vision/convert", "vision disconnected");
      Logger.recordOutput("Hub Distance", "vision disconnected");
    });
  }
}
