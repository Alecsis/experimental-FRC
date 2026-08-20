// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.utility.RobotMotor;

/** IntakeIO implementation for real TalonFX hardware. The only file allowed to touch CTRE APIs. */
public class IntakeIOReal implements IntakeIO {
  private final RobotMotor intakePivot = new RobotMotor(pivotConfig());
  private final RobotMotor roller = new RobotMotor(rollerConfig());

  private final VoltageOut pivotVoltageOut = new VoltageOut(0);
  private final MotionMagicVoltage pivotPositionOut = new MotionMagicVoltage(0);
  private final VoltageOut rollerVoltageOut = new VoltageOut(0);
  private final VelocityVoltage rollerVelocityOut = new VelocityVoltage(0);

  private static RobotMotor.MotorConfig pivotConfig() {
    RobotMotor.MotorConfig cfg = new RobotMotor.MotorConfig();
    cfg.name = "Intake/Pivot";
    cfg.canId = Constants.intakePivot;
    cfg.talonConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive))
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.kIntakePivotReduction)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(70))
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RPM.of(6000))
                .withMotionMagicAcceleration(
                    RPM.of(6000).div(Constants.kIntakePivotReduction).per(Second)))
        .withSlot0(
            new Slot0Configs()
                .withKP(70.0)
                .withKD(0.0)
                .withKV(12 / RPM.of(6000).in(RotationsPerSecond)));
    cfg.safetyTrip = RobotMotor.SafetyTripConfig.of(Amps.of(100), Second.of(0.25));
    return cfg;
  }

  private static RobotMotor.MotorConfig rollerConfig() {
    RobotMotor.MotorConfig cfg = new RobotMotor.MotorConfig();
    cfg.name = "Intake/Roller";
    cfg.canId = Constants.roller;
    cfg.talonConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
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
    // safetyTrip deliberately left null -- Intake.isJammed() already owns tuned jam-recovery on
    // this exact signal (kJamStatorCurrentAmps); a second independent detector risks the "two
    // writers, same tick" failure class documented in docs/claudex/architecture.md.
    return cfg;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakePivot.pollSafetyTrip();

    inputs.pivotConnected = intakePivot.isConnected();
    inputs.pivotPositionRads = intakePivot.getPosition().getValue().in(Radians);
    inputs.pivotVelocityRadsPerSec = intakePivot.getVelocity().getValue().in(RadiansPerSecond);
    inputs.pivotAppliedVolts = intakePivot.getMotorVoltage().getValue().in(Volts);
    inputs.pivotStatorCurrentAmps = intakePivot.getStatorCurrent().getValue().in(Amps);
    inputs.pivotSupplyCurrentAmps = intakePivot.getSupplyCurrent().getValue().in(Amps);
    inputs.pivotTempCelsius = intakePivot.getDeviceTemp().getValueAsDouble();

    inputs.rollerConnected = roller.isConnected();
    inputs.rollerPositionRads = roller.getPosition().getValue().in(Radians);
    inputs.rollerVelocityRadsPerSec = roller.getVelocity().getValue().in(RadiansPerSecond);
    inputs.rollerAppliedVolts = roller.getMotorVoltage().getValue().in(Volts);
    inputs.rollerStatorCurrentAmps = roller.getStatorCurrent().getValue().in(Amps);
    inputs.rollerSupplyCurrentAmps = roller.getSupplyCurrent().getValue().in(Amps);
    inputs.rollerTempCelsius = roller.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setPivotVoltage(double volts) {
    intakePivot.setControl(pivotVoltageOut.withOutput(Volts.of(volts)));
  }

  @Override
  public void setPivotPosition(double positionRads) {
    intakePivot.setControl(pivotPositionOut.withPosition(Radians.of(positionRads)));
  }

  @Override
  public void setPivotEncoderPosition(double positionRads) {
    intakePivot.setPosition(Radians.of(positionRads));
  }

  @Override
  public void setRollerVoltage(double volts) {
    roller.setControl(rollerVoltageOut.withOutput(Volts.of(volts)));
  }

  @Override
  public void setRollerVelocity(double velocityRadPerSec) {
    roller.setControl(rollerVelocityOut.withVelocity(RadiansPerSecond.of(velocityRadPerSec)));
  }
}
