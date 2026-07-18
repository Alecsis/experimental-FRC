// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

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

import frc.robot.Constants;

/** ShooterIO implementation for real TalonFX/TalonFXS hardware. The only file allowed to touch CTRE APIs. */
public class ShooterIOReal implements ShooterIO {
  private final TalonFX shootMotor = new TalonFX(Constants.shootMotor);
  private final TalonFX indexMotor = new TalonFX(Constants.indexMotor);
  private final TalonFXS agitator = new TalonFXS(Constants.agitator);

  private final VelocityTorqueCurrentFOC shootVelocityOut = new VelocityTorqueCurrentFOC(0);
  private final VoltageOut shootVoltageOut = new VoltageOut(0);
  private final VoltageOut indexVoltageOut = new VoltageOut(0);
  private final VoltageOut agitatorVoltageOut = new VoltageOut(0);

  public ShooterIOReal() {
    configureShoot();
    configureIndex();
    configureAgitator();
  }

  private void configureShoot() {
    TalonFXConfiguration conf = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(80))
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true))
        .withSlot0(
            new Slot0Configs()
                // kA is extreme relative to kV by design, not a tuning error: a single Kraken X60
                // drives all 4 flywheels (high reflected inertia) on an underhand shot trajectory.
                .withKP(13)
                .withKA(50)
                .withKV(0.215));
    shootMotor.getConfigurator().apply(conf);
  }

  private void configureIndex() {
    TalonFXConfiguration conf = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(70))
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true));
    indexMotor.getConfigurator().apply(conf);
  }

  private void configureAgitator() {
    TalonFXSConfiguration conf = new TalonFXSConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
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

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shootConnected = shootMotor.isConnected();
    inputs.shootPositionRads = shootMotor.getPosition().getValue().in(Radians);
    inputs.shootVelocityRadsPerSec = shootMotor.getVelocity().getValue().in(RadiansPerSecond);
    inputs.shootAppliedVolts = shootMotor.getMotorVoltage().getValue().in(Volts);
    inputs.shootStatorCurrentAmps = shootMotor.getStatorCurrent().getValue().in(Amps);
    inputs.shootSupplyCurrentAmps = shootMotor.getSupplyCurrent().getValue().in(Amps);
    inputs.shootTempCelsius = shootMotor.getDeviceTemp().getValueAsDouble();

    inputs.indexConnected = indexMotor.isConnected();
    inputs.indexPositionRads = indexMotor.getPosition().getValue().in(Radians);
    inputs.indexVelocityRadsPerSec = indexMotor.getVelocity().getValue().in(RadiansPerSecond);
    inputs.indexAppliedVolts = indexMotor.getMotorVoltage().getValue().in(Volts);
    inputs.indexStatorCurrentAmps = indexMotor.getStatorCurrent().getValue().in(Amps);
    inputs.indexSupplyCurrentAmps = indexMotor.getSupplyCurrent().getValue().in(Amps);
    inputs.indexTempCelsius = indexMotor.getDeviceTemp().getValueAsDouble();

    inputs.agitatorConnected = agitator.isConnected();
    inputs.agitatorPositionRads = agitator.getPosition().getValue().in(Radians);
    inputs.agitatorVelocityRadsPerSec = agitator.getVelocity().getValue().in(RadiansPerSecond);
    inputs.agitatorAppliedVolts = agitator.getMotorVoltage().getValue().in(Volts);
    inputs.agitatorStatorCurrentAmps = agitator.getStatorCurrent().getValue().in(Amps);
    inputs.agitatorSupplyCurrentAmps = agitator.getSupplyCurrent().getValue().in(Amps);
    inputs.agitatorTempCelsius = agitator.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setShootVelocity(double velocityRadPerSec) {
    shootMotor.setControl(shootVelocityOut.withVelocity(RadiansPerSecond.of(velocityRadPerSec)));
  }

  @Override
  public void setShootVoltage(double volts) {
    shootMotor.setControl(shootVoltageOut.withOutput(Volts.of(volts)));
  }

  @Override
  public void setIndexVoltage(double volts) {
    indexMotor.setControl(indexVoltageOut.withOutput(Volts.of(volts)));
  }

  @Override
  public void setAgitatorVoltage(double volts) {
    agitator.setControl(agitatorVoltageOut.withOutput(Volts.of(volts)));
  }
}
