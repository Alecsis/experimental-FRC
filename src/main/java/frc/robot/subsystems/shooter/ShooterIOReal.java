// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.utility.RobotMotor;

/** ShooterIO implementation for real TalonFX/TalonFXS hardware. The only file allowed to touch CTRE APIs. */
public class ShooterIOReal implements ShooterIO {
  private final RobotMotor shootMotor = new RobotMotor(shootConfig());
  private final RobotMotor indexMotor = new RobotMotor(indexConfig());
  private final TalonFXS agitator = new TalonFXS(Constants.agitator);

  private final VelocityTorqueCurrentFOC shootVelocityOut = new VelocityTorqueCurrentFOC(0);
  private final VoltageOut shootVoltageOut = new VoltageOut(0);
  private final VoltageOut indexVoltageOut = new VoltageOut(0);
  private final VoltageOut agitatorVoltageOut = new VoltageOut(0);

  public ShooterIOReal() {
    configureAgitator();
  }

  // Trip threshold sits 20A below each motor's firmware StatorCurrentLimit, same margin used for
  // Intake/Pivot in IntakeIOReal -- early/soft warning before the firmware clamp, not a new
  // protective action.
  private static RobotMotor.MotorConfig shootConfig() {
    RobotMotor.MotorConfig cfg = new RobotMotor.MotorConfig();
    cfg.name = "Shooter/Shoot";
    cfg.canId = Constants.shootMotor;
    cfg.talonConfig = new TalonFXConfiguration()
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
    cfg.safetyTrip = RobotMotor.SafetyTripConfig.of(Amps.of(100), Second.of(0.25));
    return cfg;
  }

  private static RobotMotor.MotorConfig indexConfig() {
    RobotMotor.MotorConfig cfg = new RobotMotor.MotorConfig();
    cfg.name = "Shooter/Index";
    cfg.canId = Constants.indexMotor;
    cfg.talonConfig = new TalonFXConfiguration()
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
    cfg.safetyTrip = RobotMotor.SafetyTripConfig.of(Amps.of(100), Second.of(0.25));
    return cfg;
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
    shootMotor.pollSafetyTrip();
    indexMotor.pollSafetyTrip();

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
