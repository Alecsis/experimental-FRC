// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/**
 * ShooterIO implementation backed by WPILib physics simulation. No vendor hardware
 * dependencies -- suitable for desktop/sim-only testing and replay.
 */
public class ShooterIOSim implements ShooterIO {
  // Confirmed real hardware: shoot/index are Kraken X60 (TalonFX). Agitator is a NEO 550 driven
  // by a TalonFXS (ShooterIOReal configures MotorArrangementValue.NEO550_JST).
  private static final DCMotor shootGearbox = DCMotor.getKrakenX60(1);
  private static final DCMotor indexGearbox = DCMotor.getKrakenX60(1);
  private static final DCMotor agitatorGearbox = DCMotor.getNeo550(1);

  private static final double kSimLoopPeriodSecs = 0.02;
  private static final double kShootKP = 13.0;

  private final FlywheelSim shootSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
          shootGearbox, Constants.kShooterFlywheelMOI, Constants.kShooterReduction),
      shootGearbox);

  private final DCMotorSim indexSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          indexGearbox, Constants.kIndexMOI, Constants.kIndexReduction),
      indexGearbox);

  private final DCMotorSim agitatorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          agitatorGearbox, Constants.kAgitatorMOI, Constants.kAgitatorReduction),
      agitatorGearbox);

  private double shootAppliedVolts = 0.0;
  private boolean shootClosedLoop = false;
  private double shootVelocitySetpointRadPerSec = 0.0;

  private double indexAppliedVolts = 0.0;
  private double agitatorAppliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double shootVelocityRadPerSec = RPM.of(shootSim.getAngularVelocityRPM()).in(RadiansPerSecond);

    if (DriverStation.isDisabled()) {
      shootAppliedVolts = 0.0;
      indexAppliedVolts = 0.0;
      agitatorAppliedVolts = 0.0;
    } else if (shootClosedLoop) {
      shootAppliedVolts = MathUtil.clamp(
          (shootVelocitySetpointRadPerSec - shootVelocityRadPerSec) * kShootKP, -12.0, 12.0);
    }

    shootSim.setInputVoltage(shootAppliedVolts);
    shootSim.update(kSimLoopPeriodSecs);

    indexSim.setInputVoltage(indexAppliedVolts);
    indexSim.update(kSimLoopPeriodSecs);

    agitatorSim.setInputVoltage(agitatorAppliedVolts);
    agitatorSim.update(kSimLoopPeriodSecs);

    inputs.shootConnected = true;
    inputs.shootPositionRads = 0.0; // flywheels don't simulate position
    inputs.shootVelocityRadsPerSec = RPM.of(shootSim.getAngularVelocityRPM()).in(RadiansPerSecond);
    inputs.shootAppliedVolts = shootAppliedVolts;
    inputs.shootStatorCurrentAmps = shootSim.getCurrentDrawAmps();
    inputs.shootSupplyCurrentAmps = shootSim.getCurrentDrawAmps();
    inputs.shootTempCelsius = 0.0;

    inputs.indexConnected = true;
    inputs.indexPositionRads = indexSim.getAngularPositionRad();
    inputs.indexVelocityRadsPerSec = indexSim.getAngularVelocityRadPerSec();
    inputs.indexAppliedVolts = indexAppliedVolts;
    inputs.indexStatorCurrentAmps = indexSim.getCurrentDrawAmps();
    inputs.indexSupplyCurrentAmps = indexSim.getCurrentDrawAmps();
    inputs.indexTempCelsius = 0.0;

    inputs.agitatorConnected = true;
    inputs.agitatorPositionRads = agitatorSim.getAngularPositionRad();
    inputs.agitatorVelocityRadsPerSec = agitatorSim.getAngularVelocityRadPerSec();
    inputs.agitatorAppliedVolts = agitatorAppliedVolts;
    inputs.agitatorStatorCurrentAmps = agitatorSim.getCurrentDrawAmps();
    inputs.agitatorSupplyCurrentAmps = agitatorSim.getCurrentDrawAmps();
    inputs.agitatorTempCelsius = 0.0;
  }

  @Override
  public void setShootVelocity(double velocityRadPerSec) {
    shootClosedLoop = true;
    shootVelocitySetpointRadPerSec = velocityRadPerSec;
  }

  @Override
  public void setShootVoltage(double volts) {
    shootClosedLoop = false;
    shootAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setIndexVoltage(double volts) {
    indexAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setAgitatorVoltage(double volts) {
    agitatorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
