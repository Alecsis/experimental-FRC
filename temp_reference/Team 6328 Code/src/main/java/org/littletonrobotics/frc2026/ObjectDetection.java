// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2026.subsystems.vision.VisionConstants;
import org.littletonrobotics.frc2026.util.EqualsUtil;
import org.littletonrobotics.frc2026.util.FuelSim;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.frc2026.util.geometry.Bounds;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;

@ExtensionMethod({GeomUtil.class})
public class ObjectDetection {
  private Set<RobotPoseRecord> robotPoses = new HashSet<>();

  private static final LoggedTunableNumber fuelPersistanceTime =
      new LoggedTunableNumber("ObjectDetection/FuelPersistanceTime", 3.0);
  private static final LoggedTunableNumber allowedRoll =
      new LoggedTunableNumber("ObjectDetection/AllowedRoll", Units.degreesToRadians(5));
  private static final LoggedTunableNumber allowedPitch =
      new LoggedTunableNumber("ObjectDetection/AllowedPitch", Units.degreesToRadians(5));
  private static final LoggedTunableNumber fuelOverlap =
      new LoggedTunableNumber("ObjectDetection/FuelOverlap", FieldConstants.fuelDiameter / 2.0);
  private static final double maxPossibleFuel = 504;

  private static final LoggedTunableNumber robotPersistanceTime =
      new LoggedTunableNumber("ObjectDetection/RobotPersistanceTime", 2.0);
  private static final LoggedTunableNumber robotOverlap =
      new LoggedTunableNumber("ObjectDetection/RobotOverlap", Units.inchesToMeters(35.0));

  private static final Bounds fieldBounds =
      new Bounds(0, FieldConstants.fieldLength, 0, FieldConstants.fieldWidth);

  // Spatial hash grid for fuel
  private Map<GridCoord, Set<FuelPoseRecord>> spatialGrid = new HashMap<>();

  private static ObjectDetection instance;
  @Setter private static FuelSim fuelSim;

  public static ObjectDetection getInstance() {
    if (instance == null) instance = new ObjectDetection();
    return instance;
  }

  /** Gets the grid coordinate for a given real-world translation. */
  private GridCoord getGridCoord(Translation2d translation) {
    // Assume 1 meter cell size
    return new GridCoord(
        (int) Math.floor(translation.getX()), (int) Math.floor(translation.getY()));
  }

  /** Initializes the fuel poses from assumed positions */
  public void initializeFuelPoses() {
    GridCoord cell;
    Translation2d fuelPose;
    Translation2d center = FieldConstants.fieldCenter;
    double currentTime = Timer.getTimestamp();
    for (int i = 0; i < 15; i++) {
      for (int j = 0; j < 6; j++) {
        fuelPose = center.plus(new Translation2d(0.076 + 0.152 * j, 0.0254 + 0.076 + 0.152 * i));
        cell = getGridCoord(fuelPose);
        spatialGrid
            .computeIfAbsent(cell, k -> new HashSet<>())
            .add(new FuelPoseRecord(fuelPose, currentTime));
        fuelPose = center.plus(new Translation2d(-0.076 - 0.152 * j, 0.0254 + 0.076 + 0.152 * i));
        cell = getGridCoord(fuelPose);
        spatialGrid
            .computeIfAbsent(cell, k -> new HashSet<>())
            .add(new FuelPoseRecord(fuelPose, currentTime));

        fuelPose = center.plus(new Translation2d(0.076 + 0.152 * j, -0.0254 - 0.076 - 0.152 * i));
        cell = getGridCoord(fuelPose);
        spatialGrid
            .computeIfAbsent(cell, k -> new HashSet<>())
            .add(new FuelPoseRecord(fuelPose, currentTime));

        fuelPose = center.plus(new Translation2d(-0.076 - 0.152 * j, -0.0254 - 0.076 - 0.152 * i));
        cell = getGridCoord(fuelPose);
        spatialGrid
            .computeIfAbsent(cell, k -> new HashSet<>())
            .add(new FuelPoseRecord(fuelPose, currentTime));
      }
    }
  }

  /** Clears fuel that is too old or in the robot. */
  public void clearOldFuelPoses() {
    Rectangle2d robot =
        new Rectangle2d(
            RobotState.getInstance()
                .getEstimatedPose()
                .transformBy(
                    new Translation2d(
                            ((DriveConstants.intakeFarX - DriveConstants.frameWidthX) / 2.0), 0.0)
                        .toTransform2d()),
            DriveConstants.intakeFarX,
            DriveConstants.frameWidthY);

    double currentTime = Timer.getTimestamp();
    double persistTime = fuelPersistanceTime.get();

    spatialGrid
        .values()
        .forEach(
            cellSet ->
                cellSet.removeIf(
                    x ->
                        (currentTime - x.timestamp() >= persistTime)
                            || robot.contains(x.translation())));

    // Clean up empty buckets
    spatialGrid.values().removeIf(Set::isEmpty);
  }

  /** Clears fuel that is within the FOV of the specified camera. */
  public void clearFuelInCameraFOV(double timestamp, int camera) {
    // Get field to robot
    var fieldToRobotOptional = RobotState.getInstance().getEstimatedPoseAtTimestamp(timestamp);
    if (fieldToRobotOptional.isEmpty()) {
      return;
    }
    Pose2d fieldToRobot = fieldToRobotOptional.get();

    // Get field to camera
    var robotToCameraOptional = VisionConstants.cameras[camera].poseFunction().apply(timestamp);
    if (robotToCameraOptional.isEmpty()) {
      return;
    }
    Pose3d robotToCamera = robotToCameraOptional.get();
    Pose2d fieldToCamera = fieldToRobot.transformBy(robotToCamera.toPose2d().toTransform2d());

    // Clear poses
    Translation2d camTranslation = fieldToCamera.getTranslation();
    Rotation2d camRotation = fieldToCamera.getRotation();

    // Cache constants outside the loop
    double halfFov = VisionConstants.cameras[camera].fovRads() / 2.0;
    double camX = camTranslation.getX();
    double camY = camTranslation.getY();
    double camAngleRads = camRotation.getRadians();

    spatialGrid
        .values()
        .forEach(
            cellSet ->
                cellSet.removeIf(
                    x -> {
                      // Primitive math is faster than instantiating Transform2d
                      double dx = x.translation().getX() - camX;
                      double dy = x.translation().getY() - camY;
                      double angleToFuelRads = Math.atan2(dy, dx);

                      // Calculate angular difference and wrap to -pi/pi
                      double angleDiff =
                          Math.abs(
                              edu.wpi.first.math.MathUtil.angleModulus(
                                  angleToFuelRads - camAngleRads));

                      // Remove if it IS inside the FOV
                      return angleDiff <= halfFov;
                    }));

    // Clean up empty buckets
    spatialGrid.values().removeIf(Set::isEmpty);
  }

  public void addFuelTxTyObservation(FuelTxTyObservation observation) {
    if (!DriverStation.isAutonomous() && Constants.getMode() == Constants.Mode.REAL) {
      return;
    }

    Optional<Rotation3d> estimatedRotation3d =
        RobotState.getInstance().getEstimatedRotation3dAtTimestamp(observation.timestamp);
    if (estimatedRotation3d.isPresent()) {
      if (!(EqualsUtil.epsilonEquals(estimatedRotation3d.get().getX(), 0, allowedRoll.get())
          && EqualsUtil.epsilonEquals(estimatedRotation3d.get().getY(), 0, allowedPitch.get()))) {
        return;
      }
    }

    int currentTotalFuel = spatialGrid.values().stream().mapToInt(Set::size).sum();
    if (currentTotalFuel >= maxPossibleFuel) {
      return;
    }

    var fieldToRobotOptional =
        RobotState.getInstance().getEstimatedPoseAtTimestamp(observation.timestamp());
    if (fieldToRobotOptional.isEmpty()) {
      return;
    }
    Pose2d fieldToRobot = fieldToRobotOptional.get();

    var robotToCameraOptional =
        VisionConstants.cameras[observation.camera()].poseFunction().apply(observation.timestamp());
    if (robotToCameraOptional.isEmpty()) {
      return;
    }
    Pose3d robotToCamera = robotToCameraOptional.get();

    // Find midpoint of width of top tx ty
    double tx = (observation.tx()[0] + observation.tx()[1]) / 2;
    double ty = (observation.ty()[0] + observation.ty()[1]) / 2;

    // Account for camera roll
    Translation2d txyxTranslation =
        new Translation2d(Math.tan(tx), Math.tan(-ty))
            .rotateBy(new Rotation2d(-robotToCamera.getRotation().getX()));
    tx = Math.atan(txyxTranslation.getX());
    ty = -Math.atan(txyxTranslation.getY());

    // No fuel above camera
    double cameraToFuelAngle = -robotToCamera.getRotation().getY() - ty;
    if (cameraToFuelAngle >= 0) {
      return;
    }

    // Top down distance to fuel from camera
    double cameraToFuelNorm =
        (-robotToCamera.getZ() + FieldConstants.fuelDiameter)
            / Math.tan(-robotToCamera.getRotation().getY() - ty)
            / Math.cos(tx);

    Pose2d fieldToCamera = fieldToRobot.transformBy(robotToCamera.toPose2d().toTransform2d());
    Pose2d fieldToFuel =
        fieldToCamera
            .transformBy(new Transform2d(Translation2d.kZero, new Rotation2d(-tx)))
            .transformBy(new Transform2d(new Translation2d(cameraToFuelNorm, 0), Rotation2d.kZero));
    Translation2d fieldToFuelTranslation2d = fieldToFuel.getTranslation();

    // No fuel outside the field
    if (!fieldBounds.contains(fieldToFuelTranslation2d)) return;

    FuelPoseRecord fuelPoseRecord =
        new FuelPoseRecord(fieldToFuelTranslation2d, observation.timestamp());

    // Spatial hash insertion & deduplication
    GridCoord targetCell = getGridCoord(fieldToFuelTranslation2d);
    double overlapDist = fuelOverlap.get();

    // Check the target cell and its 8 immediate neighbors for overlaps
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        GridCoord checkCell = new GridCoord(targetCell.x() + dx, targetCell.y() + dy);
        Set<FuelPoseRecord> cellSet = spatialGrid.get(checkCell);

        if (cellSet != null) {
          cellSet.removeIf(
              x -> x.translation().getDistance(fieldToFuelTranslation2d) <= overlapDist);

          if (cellSet.isEmpty()) {
            spatialGrid.remove(checkCell);
          }
        }
      }
    }

    // Add to the appropriate grid cell
    spatialGrid.computeIfAbsent(targetCell, k -> new HashSet<>()).add(fuelPoseRecord);
  }

  public void addRobotTxTyObservation(RobotTxTyObservation observation) {
    Optional<Rotation3d> estimatedRotation3d =
        RobotState.getInstance().getEstimatedRotation3dAtTimestamp(observation.timestamp);
    if (estimatedRotation3d.isPresent()) {
      if (!(EqualsUtil.epsilonEquals(estimatedRotation3d.get().getX(), 0, allowedRoll.get())
          && EqualsUtil.epsilonEquals(estimatedRotation3d.get().getY(), 0, allowedPitch.get()))) {
        return;
      }
    }

    var fieldToRobotOptional =
        RobotState.getInstance().getEstimatedPoseAtTimestamp(observation.timestamp());
    if (fieldToRobotOptional.isEmpty()) {
      return;
    }
    Pose2d fieldToRobot = fieldToRobotOptional.get();

    var robotToCameraOptional =
        VisionConstants.cameras[observation.camera()].poseFunction().apply(observation.timestamp());
    if (robotToCameraOptional.isEmpty()) {
      return;
    }
    Pose3d robotToCamera = robotToCameraOptional.get();

    // Find midpoint of width of bottom tx ty
    double tx = (observation.tx()[2] + observation.tx()[3]) / 2;
    double ty = (observation.ty()[2] + observation.ty()[3]) / 2;

    // Account for camera roll
    Translation2d txyxTranslation =
        new Translation2d(Math.tan(tx), Math.tan(-ty))
            .rotateBy(new Rotation2d(-robotToCamera.getRotation().getX()));
    tx = Math.atan(txyxTranslation.getX());
    ty = -Math.atan(txyxTranslation.getY());

    // No bots above camera
    double cameraToBotAngle = -robotToCamera.getRotation().getY() - ty;
    if (cameraToBotAngle >= 0) {
      return;
    }

    // Top down distance to bot from camera
    double cameraToBotNorm =
        (-robotToCamera.getZ()) / Math.tan(-robotToCamera.getRotation().getY() - ty) / Math.cos(tx)
            + DriveConstants.driveBaseRadius;

    Pose2d fieldToCamera = fieldToRobot.transformBy(robotToCamera.toPose2d().toTransform2d());
    Pose2d fieldToBot =
        fieldToCamera
            .transformBy(new Transform2d(Translation2d.kZero, new Rotation2d(-tx)))
            .transformBy(new Transform2d(new Translation2d(cameraToBotNorm, 0), Rotation2d.kZero));

    Translation2d fieldToBotTranslation2d = fieldToBot.getTranslation();

    // No bots outside the field
    if (!fieldBounds.contains(fieldToBotTranslation2d)) return;

    RobotPoseRecord robotPoseRecord =
        new RobotPoseRecord(fieldToBotTranslation2d, observation.timestamp());

    robotPoses =
        robotPoses.stream()
            .filter((x) -> x.translation.getDistance(fieldToBotTranslation2d) > robotOverlap.get())
            .collect(Collectors.toSet());
    robotPoses.add(robotPoseRecord);
  }

  public Set<Translation2d> getFuelTranslations() {
    if (Constants.getMode() == Constants.Mode.SIM) {
      return fuelSim.getFuels();
    }
    // Flatten the spatial grid values into a single set of translations
    return spatialGrid.values().stream()
        .flatMap(Set::stream)
        .map(FuelPoseRecord::translation)
        .collect(Collectors.toSet());
  }

  public Set<Translation2d> getRobotTranslations() {
    return robotPoses.stream()
        .filter((x) -> Timer.getTimestamp() - x.timestamp() < robotPersistanceTime.get())
        .map(RobotPoseRecord::translation)
        .collect(Collectors.toSet());
  }

  public record GridCoord(int x, int y) {}

  public record FuelTxTyObservation(int camera, double[] tx, double[] ty, double timestamp) {}

  public record RobotTxTyObservation(int camera, double[] tx, double[] ty, double timestamp) {}

  public record FuelPoseRecord(Translation2d translation, double timestamp) {}

  public record RobotPoseRecord(Translation2d translation, double timestamp) {}
}
