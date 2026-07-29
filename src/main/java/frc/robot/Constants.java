// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // AdvantageKit's own LogFileUtil.findReplayLog() (called from Robot.java's REPLAY branch) and
  // the replayWatch Gradle task both already treat this environment variable as the highest
  // priority replay-log source (decompiled akit-java-26.0.2-sources.jar, LogFileUtil.java:17,51,
  // ReplayWatch.java:49,168). Deriving currentMode from it is what makes that branch reachable --
  // previously nothing here consulted it, so REPLAY was unreachable no matter how AKIT_LOG_PATH
  // was set. Unset in every existing sim/test invocation path, so this changes no default
  // behavior; it only takes effect when a developer explicitly sets it to run a replay.
  private static final String kReplayLogPathEnvVar = "AKIT_LOG_PATH";

  public static final Mode simMode = Mode.SIM;

  /** Package-private and pure so it's independently testable -- see ConstantsReplayModeTest. */
  static Mode resolveCurrentMode(boolean isReal, String replayLogPathEnvVar, Mode simMode) {
    if (isReal) {
      return Mode.REAL;
    }
    return replayLogPathEnvVar != null ? Mode.REPLAY : simMode;
  }

  public static final Mode currentMode =
      resolveCurrentMode(RobotBase.isReal(), System.getenv(kReplayLogPathEnvVar), simMode);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // Intake
  public static final int intakePivot = 2;
  public static final int roller = 5;

    // shooter
  public static final int shootMotor = 3;
  public static final int indexMotor = 4;
  public static final int agitator = 6;

  // Vision
  public static final String[] visionCameraNames = { "limelight-bow", "limelight-intake" };
  public static final double kVisionMaxTagDistMeters = 5.0; // MegaTag2 trust filter cutoff
  public static final double kVisionStdDevCoefficient = 0.1; // dev = coefficient * (avgTagDist / tagCount)
  public static final double kMaxVisionJumpMeters = 1.0; // reject a vision read this far from the current estimate
  public static final double kVisionStdDevMinMeters = 0.05; // floor so no single frame gets near-total trust

  // Intake pivot/roller sim & physical constants (placeholder estimates -- TODO: measure/tune against real robot)
  public static final double kIntakePivotReduction = 17.50;
  public static final double kIntakePivotMassKg = 0.9; // TODO: measure real arm mass
  public static final double kIntakePivotLengthMeters = 0.25; // TODO: measure real arm length
  public static final double kIntakePivotMinAngleDeg = -260; // TODO: measure real range of motion
  public static final double kIntakePivotMaxAngleDeg = 20; // TODO: measure real range of motion
  public static final double kIntakeRollerReduction = 1.0;
  public static final double kIntakeRollerMOI = 0.001; // kg*m^2, TODO: tune, typical roller estimate

  // Shooter/Index/Agitator sim & physical constants (placeholder estimates -- TODO: measure/tune against real robot)
  public static final double kShooterReduction = 1.0;
  public static final double kShooterFlywheelMOI = 0.004; // kg*m^2, TODO: tune, typical shooter flywheel estimate
  public static final double kIndexReduction = 1.0;
  public static final double kIndexMOI = 0.001; // kg*m^2, TODO: tune
  public static final double kAgitatorReduction = 1.0;
  public static final double kAgitatorMOI = 0.0005; // kg*m^2, TODO: tune

  // Drivetrain simulation (maple-sim) physical constants -- kept in sync with
  // pathplanner/settings.json so PathPlanner's trajectory generation and MapleSim's physics
  // simulate the same robot. Mass/footprint below match settings.json's robotMass/robotLength/
  // robotWidth (mentor-confirmed against a real scale/tape measurement, 2026-07-18).
  public static final double kRobotMassWithBumpersKg = 43.545;
  public static final double kBumperLengthXMeters = 0.686; // = settings.json robotLength (X, fore-aft)
  public static final double kBumperWidthYMeters = 0.813; // = settings.json robotWidth (Y, left-right)
  // Neither this nor settings.json's wheelCOF is a measured value -- 0.8 is kept identical to
  // settings.json purely for cross-consistency. TODO: validate against the real wheel/carpet
  // pairing (e.g. a static drag or incline test) before trusting either for tuning.
  public static final double kWheelCOF = 0.8;

  // Intake simulation (maple-sim) physical constants (placeholder estimates -- TODO: measure/tune against real robot)
  public static final double kIntakeSimWidthMeters = 0.7; // TODO: measure real intake width
  public static final int kIntakeSimCapacity = 1; // TODO: confirm real max held-piece count

}
