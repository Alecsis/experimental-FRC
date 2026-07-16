// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    Logger.recordOutput("Agitator State", state);
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

  private boolean isJammed() {
    return inputs.indexStatorCurrentAmps > kJamStatorCurrentAmps
        && inputs.indexVelocityRadsPerSec < kJamVelocityThresholdRadPerSec;
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
    Logger.recordOutput("Indexer State", state);
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
    Logger.processInputs("Shooter", inputs);

    SmartDashboard.putBoolean(
        "Shooter/Index Stall", inputs.indexStatorCurrentAmps > kJamStatorCurrentAmps);

    if (targetRPM > 0) {
      setRPMShooter(targetRPM);
    } else {
      io.setShootVoltage(0);
    }

    Logger.recordOutput("Shooter Target RPM", targetRPM);
  }
}
