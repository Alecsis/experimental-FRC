// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private static final double kJamStatorCurrentAmps = 80;
  private static final double kJamVelocityThresholdRadPerSec = RotationsPerSecond.of(5).in(RadiansPerSecond);
  private static final double kJamRecoveryPulseSeconds = 0.3;
  private static final double kHardstopStatorCurrentAmps = 65;
  private static final double kHomingVoltage = 3.0;

  private static Intake instance;

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake(
          Constants.currentMode == Constants.Mode.REAL ? new IntakeIOReal() : new IntakeIOSim());
    }
    return instance;
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private boolean homed = false;
  private boolean deploy = false;
  private boolean sysIdActive = false;
  private Roller currentRoller = Roller.STOP;
  private boolean jamRecoveryActive = false;
  private double jamRecoveryStartTimestamp = 0.0;

  private boolean testJamOverrideActive = false;
  private double testJamStatorCurrentAmps;
  private double testJamVelocityRadsPerSec;

  public enum Roller {
    STOP(0),
    INTAKE(800),
    EJECT(-500);

    private final double rpm;

    Roller(double rpm) {
      this.rpm = rpm;
    }

    public double getRPM() {
      return rpm;
    }
  }

  public enum PivotState {
    HOME(-10),
    STOW(-245),
    AGITATE(-95),
    DOWN(-10);

    private final double deg;

    PivotState(double deg) {
      this.deg = deg;
    }

    public Angle angle() {
      return Degrees.of(deg);
    }
  }

  private final SysIdRoutine m_rollerSysId;

  /** Creates a new Intake. Use {@link #getInstance()} instead of constructing directly. */
  private Intake(IntakeIO io) {
    this.io = io;
    m_rollerSysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> Logger.recordOutput("Intake/RollerSysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> io.setRollerVoltage(volts.in(Volts)),
            log -> {
              log.motor("Roller")
                  .voltage(Volts.of(inputs.rollerAppliedVolts))
                  .angularVelocity(RadiansPerSecond.of(inputs.rollerVelocityRadsPerSec))
                  .angularPosition(Radians.of(inputs.rollerPositionRads));
            },
            this));
    SmartDashboard.putBoolean("Intake/Zeroed", false);
  }

  public void setRoller(Roller state) {
    if (state != Roller.INTAKE) {
      // STOP or EJECT always wins immediately, aborting any in-progress recovery pulse.
      jamRecoveryActive = false;
      applyRoller(state);
      return;
    }

    if (!jamRecoveryActive && isJammed()) {
      jamRecoveryActive = true;
      jamRecoveryStartTimestamp = Timer.getFPGATimestamp();
    }

    if (jamRecoveryActive) {
      if (Timer.getFPGATimestamp() - jamRecoveryStartTimestamp < kJamRecoveryPulseSeconds) {
        applyRoller(Roller.EJECT); // recovery pulse in progress
        return;
      }
      jamRecoveryActive = false; // pulse elapsed, fall through to resume INTAKE
    }

    applyRoller(Roller.INTAKE);
  }

  private void applyRoller(Roller state) {
    if (state == Roller.STOP) {
      io.setRollerVoltage(0);
    } else {
      io.setRollerVelocity(RotationsPerSecond.of(state.getRPM() / 60.0).in(RadiansPerSecond));
    }
    currentRoller = state;
    Logger.recordOutput("Intake/TargetState", state);
    Logger.recordOutput("Intake/JamRecoveryActive", jamRecoveryActive);
  }

  /** The roller state most recently sent via {@link #setRoller(Roller)}. */
  public Roller getRollerState() {
    return currentRoller;
  }

  private boolean isJammed() {
    return inputs.rollerStatorCurrentAmps > kJamStatorCurrentAmps
        && inputs.rollerVelocityRadsPerSec < kJamVelocityThresholdRadPerSec;
  }

  /** Test-only: forces isJammed()'s inputs, bypassing physics IntakeIOSim can't model (no
   *  load/obstruction). Package-private -- only for JUnit tests in this package. */
  void forceJamConditionForTest(double statorCurrentAmps, double velocityRadsPerSec) {
    testJamOverrideActive = true;
    testJamStatorCurrentAmps = statorCurrentAmps;
    testJamVelocityRadsPerSec = velocityRadsPerSec;
  }

  /** Test-only: stops overriding sensor readings, resumes real IntakeIOSim physics. */
  void clearJamOverrideForTest() {
    testJamOverrideActive = false;
  }

  public Command intakeJamReverse() {
    return Commands.sequence(
        Commands.runOnce(() -> setRoller(Roller.INTAKE)),
        Commands.waitUntil(this::isJammed),
        Commands.runOnce(() -> setRoller(Roller.EJECT)),
        Commands.waitSeconds(0.3),
        Commands.runOnce(() -> setRoller(Roller.INTAKE))).repeatedly().finallyDo(() -> setRoller(Roller.STOP));
  }

  public Command rollerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.runOnce(() -> sysIdActive = true)
        .andThen(m_rollerSysId.quasistatic(direction))
        .finallyDo(() -> {
          sysIdActive = false;
          setRoller(Roller.STOP);
        });
  }

  public Command rollerSysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.runOnce(() -> sysIdActive = true)
        .andThen(m_rollerSysId.dynamic(direction))
        .finallyDo(() -> {
          sysIdActive = false;
          setRoller(Roller.STOP);
        });
  }

  public void goTo(PivotState pos) {
    io.setPivotPosition(pos.angle().in(Radians));
    Logger.recordOutput("Intake/TargetPivotAngle", pos.angle());
  }

  public void goToDegrees(double deg) {
    io.setPivotPosition(Degrees.of(deg).in(Radians));
  }

  public Command agitatePivot() {
    return Commands.sequence(
        Commands.sequence(
            Commands.runOnce(() -> setRoller(Roller.INTAKE)),
            Commands.runOnce(() -> goTo(PivotState.AGITATE)),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> goTo(PivotState.DOWN)),
            Commands.waitSeconds(0.5)).repeatedly())
        .finallyDo(() -> goTo(PivotState.DOWN));
  }

  public Command intake() {
    return Commands.either(
        Commands.sequence(
            Commands.either(
                Commands.sequence(
                    Commands.runOnce(() -> {
                      goTo(PivotState.DOWN);
                      deploy = true;
                    }, this),
                    Commands.waitSeconds(0.5)),
                Commands.none(),
                () -> !deploy),
            intakeJamReverse()),
        intakeJamReverse(),
        () -> homed);
  }

  public Command eject() {
    return Commands.startEnd(
        () -> setRoller(Roller.EJECT),
        () -> setRoller(Roller.STOP),
        this);
  }

  public Command startRoller() {
    return Commands.startEnd(
        () -> setRoller(Roller.INTAKE),
        () -> setRoller(Roller.STOP),
        this);
  }

  public boolean hardstop() {
    return inputs.pivotStatorCurrentAmps > kHardstopStatorCurrentAmps;
  }

  public Command homing() {
    return Commands.sequence(
        Commands.run(() -> io.setPivotVoltage(kHomingVoltage), this)
            .until(this::hardstop)
            .withTimeout(1),
        Commands.runOnce(() -> {
          io.setPivotEncoderPosition(0.0);
          io.setPivotVoltage(0.0);
          homed = true;
          deploy = false;
          SmartDashboard.putBoolean("Intake/Zeroed", true);
        }, this),
        Commands.runOnce(() -> goTo(PivotState.DOWN), this));
  }

  public Command stopRoller() {
    return Commands.runOnce(() -> setRoller(Roller.STOP));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    if (testJamOverrideActive) {
      inputs.rollerStatorCurrentAmps = testJamStatorCurrentAmps;
      inputs.rollerVelocityRadsPerSec = testJamVelocityRadsPerSec;
    }
    Logger.processInputs("Intake", inputs);

    SmartDashboard.putBoolean("Intake/Roller Stall", inputs.rollerStatorCurrentAmps > kJamStatorCurrentAmps);
    SmartDashboard.putNumber("Intake/Pivot Deg", Radians.of(inputs.pivotPositionRads).in(Degrees));
  }
}
