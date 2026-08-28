// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utility.ShooterGeometry;

/**
 * Operator surface and shot-characterization log for shooter tuning.
 *
 * <p>Deliberately NOT a Subsystem and deliberately NOT part of {@link Shooter}. {@code Shooter}
 * owns the two mechanism-level ownership rules (who may write the flywheel target, who may write
 * the feed path) and nothing else; this owns the readouts, the +/- controls, and the sample log,
 * which need the drivetrain pose and the hub position that {@code Shooter} has no business holding.
 * Being a plain object with a {@code periodic()} keeps it out of the CommandScheduler's requirement
 * arbitration entirely -- it never writes an actuator, so it can never become a second writer.
 *
 * <p><b>It never modifies the production LUT.</b> Recording a sample writes to the log and the
 * dashboard only. Feeding real numbers back into {@code Vision}'s table is a separate, deliberate,
 * human step once enough points exist.
 */
public class ShooterTuning {
  /**
   * How recently a vision measurement must have been fused for a recorded sample's pose to be
   * treated as vision-corrected rather than dead-reckoned. Half a second is roughly 25 robot loops
   * -- long enough to survive a few dropped frames, short enough that a sample taken after the
   * cameras lost every tag is flagged rather than silently trusted.
   */
  private static final double kVisionRecentSeconds = 0.5;

  private final Shooter shooter;
  private final Supplier<Pose2d> robotPose;
  private final Supplier<Optional<Translation2d>> hubPosition;
  private final DoubleSupplier secondsSinceVisionMeasurement;

  private long sampleIndex = 0;
  private String lastSampleCsv = "(none)";
  /** Previous tick's feed state, so a sample can be snapshotted on the feed's RISING edge. */
  private boolean feedWasActive = false;

  public ShooterTuning(Shooter shooter, Supplier<Pose2d> robotPose,
      Supplier<Optional<Translation2d>> hubPosition, DoubleSupplier secondsSinceVisionMeasurement) {
    this.shooter = shooter;
    this.robotPose = robotPose;
    this.hubPosition = hubPosition;
    this.secondsSinceVisionMeasurement = secondsSinceVisionMeasurement;
  }

  /* ----------------------------------------------------------------------- operator controls -- */

  /**
   * Nudges the manual target up by one step. Adjusts a NUMBER only: it touches neither the indexer
   * nor the agitator, so it is safe to expose as a dashboard button. Feeding is exclusively
   * {@link Shooter#tuningFeedCmd()}, which is a physical hold.
   */
  public Command stepUpCmd() {
    return Commands.runOnce(() -> shooter.stepTuningTargetRPM(1)).withName("Tuning +StepRPM");
  }

  /** Nudges the manual target down by one step. Same safety argument as {@link #stepUpCmd()}. */
  public Command stepDownCmd() {
    return Commands.runOnce(() -> shooter.stepTuningTargetRPM(-1)).withName("Tuning -StepRPM");
  }

  /**
   * Records one characterization sample: the full geometry/RPM tuple at this instant, written to
   * the log and the dashboard. Read-only with respect to every mechanism.
   *
   * <p>{@code ignoringDisable(true)} on purpose -- the useful moment to record a point is often
   * right after a shot, with the robot already disabled, and recording moves nothing.
   */
  public Command recordSampleCmd() {
    return Commands.runOnce(this::recordSample)
        .ignoringDisable(true)
        .withName("Record Tuning Sample");
  }

  /* --------------------------------------------------------------------------- telemetry ------ */

  /** Call once per scheduler run. Publishes the live tuning readouts and auto-samples on feed. */
  public void periodic() {
    Pose2d pose = robotPose.get();
    Optional<Translation2d> hubOpt = hubPosition.get();

    double targetRpm = shooter.getTargetRPM();
    double actualRpm = shooter.getActualRPM();
    boolean feedActive = shooter.isTuningFeedActive();

    Logger.recordOutput("Shooter/Tuning/Enabled", shooter.isTuningMode());
    Logger.recordOutput("Shooter/Tuning/TargetRPM", targetRpm);
    Logger.recordOutput("Shooter/Tuning/ActualRPM", actualRpm);
    Logger.recordOutput("Shooter/Tuning/ErrorRPM", targetRpm - actualRpm);
    Logger.recordOutput("Shooter/Tuning/StepRPM", shooter.getTuningStepRPM());
    Logger.recordOutput("Shooter/Tuning/Ready", shooter.shooterAtSpeed(targetRpm));
    Logger.recordOutput("Shooter/Tuning/FeedActive", feedActive);
    Logger.recordOutput("Shooter/Tuning/Alliance", allianceName());
    Logger.recordOutput(
        "Shooter/Tuning/VisionMeasurementRecent", visionRecent());
    Logger.recordOutput(
        "Shooter/Tuning/SecondsSinceVisionMeasurement", secondsSinceVisionMeasurement.getAsDouble());

    /*
     * Poses are published as SCALAR COMPONENTS, never as a Pose2d struct output. This is not a
     * style choice -- logging a Pose2d here crashes the robot program intermittently.
     *
     * AdvantageKit's LogTable.put(String, Struct, value) packs through ONE shared StructBuffer per
     * struct type, and Telemetry.telemeterize() already publishes "Odometry/Robot" as a Pose2d --
     * from the CTRE Phoenix odometry thread, because that is the thread CTRE invokes a
     * registerTelemetry() callback on. This class runs on the robot thread. Two threads packing the
     * same struct type therefore share one buffer's position, and the loser overruns it:
     *
     *   java.nio.BufferOverflowException
     *     at edu.wpi.first.math.geometry.struct.Translation2dStruct.pack
     *     at edu.wpi.first.util.struct.StructBuffer.write
     *     at org.littletonrobotics.junction.LogTable.put
     *
     * That exception escapes robotPeriodic() and kills startCompetition(). Measured, with the
     * Pose2d outputs present: 3 failures in 9 SKILLS/run_headless_sim.py launches, and 0 in 6
     * raw-capture launches (which sampled a shorter window). The pristine base commit failed 0 of 5
     * through the same harness -- it logs no Pose2d on the robot thread at all, because PathPlanner
     * is the only other robot-thread Pose2d writer and the headless gate boots disabled. After
     * switching to the scalar outputs below: 8 of 8 clean.
     *
     * <p>Doubles do not go through StructBuffer, so they cannot collide -- and the components are
     * what a characterization sheet needs anyway. Do not "tidy" these back into a pose output.
     *
     * <p>NOTE: the same hazard still exists in baseline autonomous-critical logging (Pose2d,
     * ChassisSpeeds and SwerveModuleState[] written from both the odometry thread and the robot
     * thread during path following). That is deliberately OUT OF SCOPE for this branch and is
     * recorded as a follow-up in docs/claudex/followups/struct-logging-thread-safety.md.
     */
    Logger.recordOutput("Shooter/Tuning/RobotX", pose.getX());
    Logger.recordOutput("Shooter/Tuning/RobotY", pose.getY());
    Logger.recordOutput("Shooter/Tuning/RobotHeadingDeg", pose.getRotation().getDegrees());

    Translation2d shooterPos = ShooterGeometry.shooterPosition(pose);
    Logger.recordOutput("Shooter/Tuning/ShooterX", shooterPos.getX());
    Logger.recordOutput("Shooter/Tuning/ShooterY", shooterPos.getY());

    Logger.recordOutput("Shooter/Tuning/RobotToHubFt", robotToHubFeet(pose, hubOpt));
    Logger.recordOutput("Shooter/Tuning/ShooterToHubFt", shooterToHubFeet(pose, hubOpt));
    Logger.recordOutput("Shooter/Tuning/AimHeadingDeg", aimHeadingDegrees(pose, hubOpt));
    Logger.recordOutput("Shooter/Tuning/HeadingErrorDeg", headingErrorDegrees(pose, hubOpt));

    SmartDashboard.putBoolean("Shooter/Tuning/Enabled", shooter.isTuningMode());
    SmartDashboard.putNumber("Shooter/Tuning/TargetRPM", targetRpm);
    SmartDashboard.putNumber("Shooter/Tuning/ActualRPM", actualRpm);
    SmartDashboard.putNumber("Shooter/Tuning/ErrorRPM", targetRpm - actualRpm);
    SmartDashboard.putNumber("Shooter/Tuning/StepRPM", shooter.getTuningStepRPM());
    SmartDashboard.putBoolean("Shooter/Tuning/Ready", shooter.shooterAtSpeed(targetRpm));
    SmartDashboard.putNumber("Shooter/Tuning/ShooterToHubFt", shooterToHubFeet(pose, hubOpt));
    SmartDashboard.putString("Shooter/Tuning/LastSample", lastSampleCsv);

    // Snapshot on the RISING edge of a deliberate feed, so every tuning shot is characterized even
    // if the operator never presses the record control. The record control stays available for the
    // "judge the result, then record" workflow, where the interesting instant is after the shot.
    if (feedActive && !feedWasActive) {
      recordSample();
    }
    feedWasActive = feedActive;
  }

  /* ------------------------------------------------------------------------ sample recording -- */

  /**
   * Writes one characterization tuple. The schema is deliberately flat and fully self-contained:
   * every field needed to rebuild a distance-to-RPM table later is present in the same record, so
   * nothing has to be correlated against other log topics by timestamp after the fact.
   */
  public void recordSample() {
    Pose2d pose = robotPose.get();
    Optional<Translation2d> hubOpt = hubPosition.get();
    Translation2d shooterPos = ShooterGeometry.shooterPosition(pose);
    Translation2d hub = hubOpt.orElse(new Translation2d(Double.NaN, Double.NaN));

    double timestamp = Timer.getFPGATimestamp();
    double robotToHubFt = robotToHubFeet(pose, hubOpt);
    double shooterToHubFt = shooterToHubFeet(pose, hubOpt);
    double aimHeadingDeg = aimHeadingDegrees(pose, hubOpt);
    double headingErrorDeg = headingErrorDegrees(pose, hubOpt);
    double targetRpm = shooter.getTargetRPM();
    double actualRpm = shooter.getActualRPM();

    sampleIndex++;

    String prefix = "Shooter/Tuning/Sample/";
    Logger.recordOutput(prefix + "Index", sampleIndex);
    Logger.recordOutput(prefix + "TimestampSeconds", timestamp);
    Logger.recordOutput(prefix + "Alliance", allianceName());
    Logger.recordOutput(prefix + "RobotX", pose.getX());
    Logger.recordOutput(prefix + "RobotY", pose.getY());
    Logger.recordOutput(prefix + "RobotHeadingDeg", pose.getRotation().getDegrees());
    Logger.recordOutput(prefix + "ShooterX", shooterPos.getX());
    Logger.recordOutput(prefix + "ShooterY", shooterPos.getY());
    Logger.recordOutput(prefix + "HubX", hub.getX());
    Logger.recordOutput(prefix + "HubY", hub.getY());
    Logger.recordOutput(prefix + "RobotToHubFt", robotToHubFt);
    Logger.recordOutput(prefix + "DistanceFt", shooterToHubFt);
    Logger.recordOutput(prefix + "AimHeadingDeg", aimHeadingDeg);
    Logger.recordOutput(prefix + "HeadingErrorDeg", headingErrorDeg);
    Logger.recordOutput(prefix + "TargetRPM", targetRpm);
    Logger.recordOutput(prefix + "ActualRPM", actualRpm);
    Logger.recordOutput(prefix + "ErrorRPM", targetRpm - actualRpm);
    Logger.recordOutput(prefix + "VisionMeasurementRecent", visionRecent());
    Logger.recordOutput(prefix + "FeedActive", shooter.isTuningFeedActive());
    Logger.recordOutput(prefix + "TuningEnabled", shooter.isTuningMode());
    // No Pose2d struct output here either -- see the StructBuffer note in periodic(). RobotX/RobotY/
    // RobotHeadingDeg above already carry the whole pose.

    // One greppable line carrying the same tuple, so a sample can be read straight off the
    // dashboard or out of a log without opening a plotting tool.
    lastSampleCsv = String.format(
        "#%d t=%.3f %s distFt=%.2f robotFt=%.2f targetRPM=%.0f actualRPM=%.0f errRPM=%.0f "
            + "headingErrDeg=%.2f pose=(%.2f,%.2f,%.1fdeg) visionRecent=%b",
        sampleIndex, timestamp, allianceName(), shooterToHubFt, robotToHubFt, targetRpm, actualRpm,
        targetRpm - actualRpm, headingErrorDeg, pose.getX(), pose.getY(),
        pose.getRotation().getDegrees(), visionRecent());
    Logger.recordOutput(prefix + "Csv", lastSampleCsv);
    SmartDashboard.putString("Shooter/Tuning/LastSample", lastSampleCsv);
    SmartDashboard.putNumber("Shooter/Tuning/SampleIndex", sampleIndex);
  }

  /** The number of samples recorded this power cycle. */
  public long getSampleCount() {
    return sampleIndex;
  }

  /** The most recently recorded sample, as the same one-line record written to the log. */
  public String getLastSampleCsv() {
    return lastSampleCsv;
  }

  /* ---------------------------------------------------------------------------- geometry ------ */

  private boolean visionRecent() {
    return secondsSinceVisionMeasurement.getAsDouble() <= kVisionRecentSeconds;
  }

  private static String allianceName() {
    return DriverStation.getAlliance().map(Enum::toString).orElse("Unknown");
  }

  private static double robotToHubFeet(Pose2d pose, Optional<Translation2d> hubOpt) {
    return hubOpt
        .map(hub -> Units.metersToFeet(pose.getTranslation().getDistance(hub)))
        .orElse(Double.NaN);
  }

  private static double shooterToHubFeet(Pose2d pose, Optional<Translation2d> hubOpt) {
    return hubOpt
        .map(hub -> Units.metersToFeet(ShooterGeometry.shooterDistanceMeters(pose, hub)))
        .orElse(Double.NaN);
  }

  private static double aimHeadingDegrees(Pose2d pose, Optional<Translation2d> hubOpt) {
    return hubOpt
        .map(hub -> ShooterGeometry.aimHeading(pose, hub).getDegrees())
        .orElse(Double.NaN);
  }

  private static double headingErrorDegrees(Pose2d pose, Optional<Translation2d> hubOpt) {
    return hubOpt
        .map(hub -> {
          Rotation2d aim = ShooterGeometry.aimHeading(pose, hub);
          return Math.toDegrees(
              MathUtil.angleModulus(aim.getRadians() - pose.getRotation().getRadians()));
        })
        .orElse(Double.NaN);
  }
}
