// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;
import org.wpilib.util.sendable.SendableBuilder;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  public static final AngularVelocity kFreeSpeed = RPM.of(10000);
  private static final AngularVelocity tolerance = RPM.of(100);
  private static final double kJamStatorCurrentAmps = 70;
  private static final double kJamVelocityThresholdRadPerSec = RotationsPerSecond.of(5).in(RadiansPerSecond);

  private static Shooter instance;

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter(
          Constants.currentMode == Constants.Mode.REAL ? new ShooterIOReal() : new ShooterIOSim());
    }
    return instance;
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public double targetRPM = 0.0;
  public boolean shooterTuningModeEnable = false;
  private Agitate currentAgitate = Agitate.STOP;
  private indexing currentIndexing = indexing.STOP;

  private boolean testJamOverrideActive = false;
  private double testJamStatorCurrentAmps;
  private double testJamVelocityRadsPerSec;

  public enum Agitate {
    STOP(0),
    IN(0.35), // if shred lower 5%
    OUT(0.25);

    private final double percent;

    Agitate(double percent) {
      this.percent = percent;
    }

    public Voltage voltage() {
      return Volts.of(percent * 12);
    }
  }

  public enum indexing {
    INDEX(0.60),
    STOP(0),
    EJECT(-0.40);

    private final double percent;

    indexing(double percent) {
      this.percent = percent;
    }

    public Voltage voltage() {
      return Volts.of(percent * 12);
    }
  }

  /** Creates a new Shooter. Use {@link #getInstance()} instead of constructing directly. */
  private Shooter(ShooterIO io) {
    this.io = io;
  }

  public void setAgitator(Agitate state) {
    io.setAgitatorVoltage(state.voltage().in(Volts));
    currentAgitate = state;
    Logger.recordOutput("Agitator State", state);
  }

  /** The agitator state most recently sent via {@link #setAgitator(Agitate)}. */
  public Agitate getAgitateState() {
    return currentAgitate;
  }

  public Command agitate() {
    return Commands.startEnd(
        () -> setAgitator(Agitate.IN),
        () -> setAgitator(Agitate.STOP),
        this);
  }

  public Command agitateReverse() {
    return Commands.startEnd(
        () -> setAgitator(Agitate.OUT),
        () -> setAgitator(Agitate.STOP),
        this);
  }

  public Command agitateStop() {
    return Commands.run(() -> setAgitator(Agitate.STOP));
  }

  /** Package-private (not private) so in-package JUnit tests can assert on it directly -- same
   *  reasoning as Intake's identical pattern. */
  boolean isJammed() {
    return inputs.indexStatorCurrentAmps > kJamStatorCurrentAmps
        && inputs.indexVelocityRadsPerSec < kJamVelocityThresholdRadPerSec;
  }

  /** Test-only: forces isJammed()'s inputs, bypassing physics ShooterIOSim can't model (no
   *  load/obstruction). Package-private -- only for JUnit tests in this package. */
  void forceJamConditionForTest(double statorCurrentAmps, double velocityRadsPerSec) {
    testJamOverrideActive = true;
    testJamStatorCurrentAmps = statorCurrentAmps;
    testJamVelocityRadsPerSec = velocityRadsPerSec;
  }

  /** Test-only: stops overriding sensor readings, resumes real ShooterIOSim physics. */
  void clearJamOverrideForTest() {
    testJamOverrideActive = false;
  }

  public Command indexJam() {
    return Commands.sequence(
        Commands.runOnce(() -> indexControl(Shooter.indexing.INDEX)),
        Commands.waitUntil(this::isJammed),
        Commands.runOnce(() -> indexControl(Shooter.indexing.EJECT)),
        Commands.waitSeconds(0.3),
        Commands.runOnce(() -> indexControl(Shooter.indexing.INDEX)).repeatedly()
            .finallyDo(() -> indexControl(Shooter.indexing.EJECT)));
  }

  public void setRPMShooter(double rpm) {
    io.setShootVelocity(RPM.of(rpm).in(RadiansPerSecond));
  }

  public void targetRPMShooter(double rpm) {
    targetRPM = rpm;
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public void setOutputShooter(double percent) {
    io.setShootVoltage(percent * 12);
  }

  public void stopShooter() {
    targetRPMShooter(0);
  }

  public Command spin(double rpm) {
    return Commands.sequence(
        Commands.runOnce(() -> targetRPMShooter(rpm)),
        Commands.waitUntil(() -> shooterAtSpeed(rpm)));
  }

  public Command dashSpin() {
    return defer(() -> spin(targetRPM));
  }

  public boolean shooterAtSpeed(double targetRPM) {
    AngularVelocity current = RadiansPerSecond.of(inputs.shootVelocityRadsPerSec);
    return current.isNear(RPM.of(targetRPM), tolerance);
  }

  public void indexControl(indexing state) {
    io.setIndexVoltage(state.voltage().in(Volts));
    currentIndexing = state;
    Logger.recordOutput("Indexer State", state);
  }

  /** The indexer state most recently sent via {@link #indexControl(indexing)}. */
  public indexing getIndexerState() {
    return currentIndexing;
  }

  public Command index() {
    return Commands.startEnd(
        () -> indexControl(indexing.INDEX),
        () -> indexControl(indexing.STOP),
        this);
  }

  public Command indexEject() {
    return Commands.startEnd(
        () -> indexControl(indexing.EJECT),
        () -> indexControl(indexing.STOP),
        this);
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Current RPM",
        () -> RadiansPerSecond.of(inputs.shootVelocityRadsPerSec).in(RPM),
        null);

    builder.addDoubleProperty(
        "Current Volt",
        () -> inputs.shootAppliedVolts,
        null);

    builder.addDoubleProperty(
        "Target RPM",
        () -> targetRPM,
        null);

    builder.addDoubleProperty(
        "Dashboard RPM",
        () -> targetRPM,
        value -> targetRPM = value);

    builder.addDoubleProperty(
        "Error",
        () -> targetRPM - RadiansPerSecond.of(inputs.shootVelocityRadsPerSec).in(RPM),
        null);

    builder.addBooleanProperty("Shooter Tuning Mode",
        () -> shooterTuningModeEnable,
        value -> shooterTuningModeEnable = value);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    if (testJamOverrideActive) {
      inputs.indexStatorCurrentAmps = testJamStatorCurrentAmps;
      inputs.indexVelocityRadsPerSec = testJamVelocityRadsPerSec;
    }
    Logger.processInputs("Shooter", inputs);

    SmartDashboard.putBoolean(
        "Shooter/Index Stall", inputs.indexStatorCurrentAmps > kJamStatorCurrentAmps);
    Logger.recordOutput("Shooter/Jammed", isJammed());

    if (targetRPM > 0) {
      setRPMShooter(targetRPM);
    } else {
      io.setShootVoltage(0);
    }

    Logger.recordOutput("Shooter Target RPM", targetRPM);
  }
}
