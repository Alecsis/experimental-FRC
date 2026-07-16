// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.Roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@SuppressWarnings("unused")
public class Intake extends SubsystemBase {
  public TalonFX intakePivot;
  public TalonFX roller;
  private boolean homed = false;
  private boolean deploy = false;

  private static final double reduction = 17.50;
  private final VoltageOut rollerWant = new VoltageOut(0);
  private final VelocityVoltage rollerRPM = new VelocityVoltage(0);
  private final VoltageOut homingvolt = new VoltageOut(0);
  private final MotionMagicVoltage mm = new MotionMagicVoltage(0);

    public enum Roller{
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

  /** Creates a new Intake. */
  public Intake() {
    intakePivot = new TalonFX(Constants.intakePivot);
    roller = new TalonFX(Constants.roller);
    configMotor();
    rollerConf();
    SmartDashboard.putBoolean("Intake/Zeroed", false);
  }

  private void rollerConf() {
    TalonFXConfiguration conf = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                // flip if needed lolz
                .withInverted(InvertedValue.Clockwise_Positive))
        .withSlot0(
            new Slot0Configs()
                .withKP(.35)
                .withKV(1.44057623))

        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(90))
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true));

    roller.getConfigurator().apply(conf);
  }

  private void configMotor() {
    TalonFXConfiguration cfg = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive))
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(reduction)
                .withFeedbackSensorSource(
                    FeedbackSensorSourceValue.RotorSensor))

        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(70))
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(
                    RPM.of(6000))
                .withMotionMagicAcceleration(
                    RPM.of(6000).div(reduction).per(Second)))
        .withSlot0(
            new Slot0Configs()
                .withKP(70.0)
                .withKD(0.0)
                .withKV(12 / RPM.of(6000).in(RotationsPerSecond))

        );

    intakePivot.getConfigurator().apply(cfg);
  }

  public void setRoller(Roller state) {
    if (state == Roller.STOP) {
      roller.setControl(rollerWant.withOutput(Volts.of(0)));
    } else {
      roller.setControl(rollerRPM.withVelocity(
          RotationsPerSecond.of(state.getRPM() / 60.0)));
    }
  }

  private boolean isJammed() {
    return roller.getStatorCurrent().getValue().in(Amps) > 80
        && roller.getVelocity().getValue().in(RotationsPerSecond) < 5;
  }

  public Command intakeJamReverse() {
    return Commands.sequence(
        Commands.runOnce(() -> setRoller(Roller.INTAKE)),
        Commands.waitUntil(() -> isJammed()),
        Commands.runOnce(() -> setRoller(Roller.EJECT)),
        Commands.waitSeconds(0.3),
        Commands.runOnce(() -> setRoller(Roller.INTAKE))).repeatedly().finallyDo(() -> setRoller(Roller.STOP));
  }

  private boolean sysIdActive = false;

  private final SysIdRoutine m_rollerSysId = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,
          Volts.of(4),
          null,
          state -> SignalLogger.writeString("roller", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> roller.setControl(rollerWant.withOutput(volts.in(Volts))),
          log -> {
            log.motor("Roller")
                .voltage(roller.getMotorVoltage().getValue())
                .angularVelocity(roller.getVelocity().getValue())
                .angularPosition(roller.getPosition().getValue());
          },
          this));

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
    intakePivot.setControl(mm.withPosition(pos.angle()));
    Logger.recordOutput("Target Pivot Angle", pos.angle());
  }

  public void goToDegrees(double deg) {
    intakePivot.setControl(mm.withPosition(Degrees.of(deg)));
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

  public boolean hardstop() {
    return intakePivot.getStatorCurrent().getValue().in(Amps) > 65;
  };

  public Command homing() {
    return Commands.sequence(
        Commands.run(() -> intakePivot.setControl(homingvolt.withOutput(Volts.of(3))), this)
            .until(() -> hardstop())
            .withTimeout(1),
        Commands.runOnce(() -> {
          intakePivot.setPosition(Degrees.of(0));
          intakePivot.setControl(homingvolt.withOutput(Volts.of(0)));
          homed = true;
          deploy = false;
          SmartDashboard.putBoolean("Intake/Zeroed", true);
        }, this),
        Commands.runOnce(() -> goTo(PivotState.DOWN), this));
  }

  public Command stopRoller() {
    return Commands.runOnce(
        () -> setRoller(Roller.STOP));

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake/Roller Stall", roller.getStatorCurrent().getValue().in(Amps) > 80);
    SmartDashboard.putNumber("Intake/Pivot Deg", intakePivot.getPosition().getValue().in(Degrees));

    // Logs Roller Data
    Logger.recordOutput("Roller Volts", roller.getMotorVoltage().getValue().in(Volts));
    Logger.recordOutput("Roller Temp", roller.getDeviceTemp().getValue());
    Logger.recordOutput("Roller Stator Current", roller.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput("Roller Supply Current", roller.getSupplyCurrent().getValue().in(Amps));
    Logger.recordOutput("Roller Connection Status", roller.isConnected());

    // Logs Pivot Data
    Logger.recordOutput("Pivot Angle", intakePivot.getPosition().getValue().in(Degrees));
    Logger.recordOutput("Pivot Volts", intakePivot.getMotorVoltage().getValue().in(Volts));
    Logger.recordOutput("Pivot Temp", intakePivot.getDeviceTemp().getValue());
    Logger.recordOutput("Pivot Stator Current", intakePivot.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput("Pivot Supply Current", intakePivot.getSupplyCurrent().getValue().in(Amps));
    Logger.recordOutput("Pivot Connection Status", intakePivot.isConnected());
  }
}