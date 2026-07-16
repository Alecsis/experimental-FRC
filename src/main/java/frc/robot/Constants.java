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
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

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

}
