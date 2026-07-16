// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public static final AngularVelocity kFreeSpeed = RPM.of(10000);
  public TalonFX shootMotor;
  public TalonFX indexMotor;
  private static final AngularVelocity tolerance = RPM.of(100);
  private final VelocityTorqueCurrentFOC velocityFOC = new VelocityTorqueCurrentFOC(0);
  private final VoltageOut volt = new VoltageOut(0);
  private final VoltageOut indexVolt = new VoltageOut(0);
  public double targetRPM = 0.0;
  private final VoltageOut agiPower = new VoltageOut(0);
  public TalonFXS agitator;

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

  public Shooter() {
    shootMotor = new TalonFX(Constants.shootMotor);
    indexMotor = new TalonFX(Constants.indexMotor);
    agitator = new TalonFXS(Constants.agitator);

    addChild("shooter", shootMotor);
    addChild("indexMotor", indexMotor);
    agitator();
    configure(shootMotor);
    confIndex(indexMotor);
  }

  private void agitator() {
    TalonFXSConfiguration conf = new TalonFXSConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                // flip if needed lolz
                .withInverted(InvertedValue.CounterClockwise_Positive))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(80))
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true))
        .withCommutation(
            new CommutationConfigs()
                .withMotorArrangement(MotorArrangementValue.NEO550_JST));
    agitator.getConfigurator().apply(conf);
  }

  // ctre inteaded way of doing this shit
  private void configure(TalonFX motor) {
    TalonFXConfiguration conf = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                // flip if needed lolz
                .withInverted(InvertedValue.Clockwise_Positive))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(80))
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true))
        .withSlot0(
            new Slot0Configs()// set kp and ka to 0 and do fixed rpm, tune kv first util matches fixed rpm.
                              // then do kP to close the gap .5 ish then kA for extra current
                .withKP(13)
                .withKA(50)
                .withKV(0.215));
    motor.getConfigurator().apply(conf);

  }

  private void confIndex(TalonFX motor) {
    TalonFXConfiguration conf = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                // flip if needed lolz
                .withInverted(InvertedValue.Clockwise_Positive)

        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(70))
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true));
    motor.getConfigurator().apply(conf);
  }

  public void setAgitator(Agitate state) {
    agitator.setControl(agiPower.withOutput(state.voltage()));
    Logger.recordOutput("Agitator State", state);
  }

  public Command agitate() {
    return Commands.startEnd(
        () -> {
          setAgitator(Agitate.IN);
        },
        () -> setAgitator(Agitate.STOP),
        this);
  }

  public Command agitateReverse() {
    return Commands.startEnd(
        () -> {
          setAgitator(Agitate.OUT);
        },
        () -> setAgitator(Agitate.STOP),
        this);
  }

  public Command agitateStop() {
    return Commands.run(
        () -> setAgitator(Agitate.STOP));
  }

  private boolean isJammed() {
    return indexMotor.getStatorCurrent().getValue().in(Amps) > 70
        && indexMotor.getVelocity().getValue().in(RotationsPerSecond) < 5;
  }

  public Command indexJam() {
    return Commands.sequence(
        Commands.runOnce(() -> indexControl(Shooter.indexing.INDEX)),
        Commands.waitUntil(() -> isJammed()),
        Commands.runOnce(() -> indexControl(Shooter.indexing.EJECT)),
        Commands.waitSeconds(0.3),
        Commands.runOnce(() -> indexControl(Shooter.indexing.INDEX)).repeatedly()
            .finallyDo(() -> indexControl(Shooter.indexing.EJECT)));
  }

  // mootr god burrr
  public void setRPMShooter(double rpm) {
    shootMotor.setControl(velocityFOC.withVelocity(RPM.of(rpm)));
  }


  public void targetRPMShooter(double rpm) {
    targetRPM = rpm;
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public void setOutputShooter(double percent) {
    shootMotor.setControl(volt.withOutput(Volts.of(percent * 12)));
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
    AngularVelocity current = shootMotor.getVelocity().getValue();
    return current.isNear(RPM.of(targetRPM), tolerance);
  }

  public void indexControl(indexing state) {
    indexMotor.setControl(indexVolt.withOutput(state.voltage()));
    Logger.recordOutput("Indexer State", state);
  }

  public Command index() {
    return Commands.startEnd(
        () -> {
          indexControl(indexing.INDEX);
        },
        () -> {
          indexControl(indexing.STOP);
        },
        this);
  }

  public Command indexEject() {
    return Commands.startEnd(
        () -> {
          indexControl(indexing.EJECT);
        },
        () -> {
          indexControl(indexing.STOP);
        },
        this);
  }

  public void initSendable(edu.wpi.first.util.sendable.SendableBuilder builder) {
    builder.addDoubleProperty(
        "Current RPM",

        () -> shootMotor.getVelocity().getValue().in(RPM),
        null);

    builder.addDoubleProperty(
        "Current Volt",
        () -> shootMotor.getMotorVoltage().getValue().in(Volts),
        null

    );
    builder.addDoubleProperty(
        "Target RPM",
        () -> velocityFOC.getVelocityMeasure().in(RPM),
        null);

    builder.addDoubleProperty(
        "Dashboard RPM",
        () -> targetRPM,
        value -> targetRPM = value);

    builder.addDoubleProperty(
        "Error",
        () -> targetRPM - shootMotor.getVelocity().getValue().in(RPM),
        null);

    builder.addBooleanProperty("Shooter Tuning Mode", 
    () -> shooterTuningModeEnable,
        value -> shooterTuningModeEnable = value);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Shooter/Index Stall", indexMotor.getStatorCurrent().getValue().in(Amps) > 60);
    if (targetRPM > 0) {
      setRPMShooter(targetRPM);
    } else {
      shootMotor.setControl(volt.withOutput(Volts.of(0)));
    }

    // Logs Shooter Data
    Logger.recordOutput("Shooter Volts", shootMotor.getMotorVoltage().getValue().in(Volts));
    Logger.recordOutput("Shooter Temp", shootMotor.getDeviceTemp().getValue());
    Logger.recordOutput("Shooter Stator Current", shootMotor.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput("Shooter Supply Current", shootMotor.getSupplyCurrent().getValue().in(Amps));
    Logger.recordOutput("Shooter Connection Status", shootMotor.isConnected());
    Logger.recordOutput("Shooter RPM", shootMotor.getVelocity().getValue().in(RPM));
    Logger.recordOutput("Shooter Target RPM", targetRPM);

    // Logs Agitator Data
    Logger.recordOutput("Agitator Volts", agitator.getMotorVoltage().getValue().in(Volts));
    Logger.recordOutput("Agitator Temp", agitator.getDeviceTemp().getValue());
    Logger.recordOutput("Agitator Stator Current", agitator.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput("Agitator Supply Current", agitator.getSupplyCurrent().getValue().in(Amps));
    Logger.recordOutput("Agitator Connection Status", agitator.isConnected());

    // Logs Indexer Data
    Logger.recordOutput("Indexer Volts", indexMotor.getMotorVoltage().getValue().in(Volts));
    Logger.recordOutput("Indexer Temp", indexMotor.getDeviceTemp().getValue());
    Logger.recordOutput("Indexer Stator Current", indexMotor.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput("Indexer Supply Current", indexMotor.getSupplyCurrent().getValue().in(Amps));
    Logger.recordOutput("Indexer Connection Status", indexMotor.isConnected());
  }
}