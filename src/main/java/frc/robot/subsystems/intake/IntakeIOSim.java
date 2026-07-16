// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/**
 * IntakeIO implementation backed by WPILib physics simulation. No vendor hardware
 * dependencies -- suitable for desktop/sim-only testing and replay.
 */
public class IntakeIOSim implements IntakeIO {
  // TODO: confirm actual motors on the robot -- assumed Kraken X60 for both mechanisms
  private static final DCMotor pivotGearbox = DCMotor.getKrakenX60(1);
  private static final DCMotor rollerGearbox = DCMotor.getKrakenX60(1);

  private static final double kSimLoopPeriodSecs = 0.02;
  private static final double kStartingAngleDeg = -10.0; // matches Intake.PivotState.DOWN

  private static final double kPivotKP = 70.0;
  private static final double kPivotKD = 0.0;
  private static final double kRollerKP = 0.35;

  private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
      pivotGearbox,
      Constants.kIntakePivotReduction,
      Constants.kIntakePivotMassKg
          * Constants.kIntakePivotLengthMeters
          * Constants.kIntakePivotLengthMeters
          / 3.0,
      Constants.kIntakePivotLengthMeters,
      Degrees.of(Constants.kIntakePivotMinAngleDeg).in(Radians),
      Degrees.of(Constants.kIntakePivotMaxAngleDeg).in(Radians),
      true,
      Degrees.of(kStartingAngleDeg).in(Radians));

  private final DCMotorSim rollerSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          rollerGearbox, Constants.kIntakeRollerMOI, Constants.kIntakeRollerReduction),
      rollerGearbox);

  private double pivotAppliedVolts = 0.0;
  private boolean pivotClosedLoop = false;
  private double pivotPositionSetpointRads = 0.0;

  private double rollerAppliedVolts = 0.0;
  private boolean rollerClosedLoop = false;
  private double rollerVelocitySetpointRadPerSec = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      pivotAppliedVolts = 0.0;
      rollerAppliedVolts = 0.0;
    } else {
      if (pivotClosedLoop) {
        pivotAppliedVolts = MathUtil.clamp(
            (pivotPositionSetpointRads - pivotSim.getAngleRads()) * kPivotKP
                + (0.0 - pivotSim.getVelocityRadPerSec()) * kPivotKD,
            -12.0, 12.0);
      }
      if (rollerClosedLoop) {
        rollerAppliedVolts = MathUtil.clamp(
            (rollerVelocitySetpointRadPerSec - rollerSim.getAngularVelocityRadPerSec()) * kRollerKP,
            -12.0, 12.0);
      }
    }

    pivotSim.setInputVoltage(pivotAppliedVolts);
    pivotSim.update(kSimLoopPeriodSecs);

    rollerSim.setInputVoltage(rollerAppliedVolts);
    rollerSim.update(kSimLoopPeriodSecs);

    inputs.pivotConnected = true;
    inputs.pivotPositionRads = pivotSim.getAngleRads();
    inputs.pivotVelocityRadsPerSec = pivotSim.getVelocityRadPerSec();
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotStatorCurrentAmps = pivotSim.getCurrentDrawAmps();
    inputs.pivotSupplyCurrentAmps = pivotSim.getCurrentDrawAmps();
    inputs.pivotTempCelsius = 0.0;

    inputs.rollerConnected = true;
    inputs.rollerPositionRads = rollerSim.getAngularPositionRad();
    inputs.rollerVelocityRadsPerSec = rollerSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerStatorCurrentAmps = rollerSim.getCurrentDrawAmps();
    inputs.rollerSupplyCurrentAmps = rollerSim.getCurrentDrawAmps();
    inputs.rollerTempCelsius = 0.0;
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotClosedLoop = false;
    pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setPivotPosition(double positionRads) {
    pivotClosedLoop = true;
    pivotPositionSetpointRads = positionRads;
  }

  @Override
  public void setPivotEncoderPosition(double positionRads) {
    // No-op: the physics sim already tracks the arm's true position directly, so there is
    // no separate encoder reference to re-zero the way there is on real hardware.
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerClosedLoop = false;
    rollerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setRollerVelocity(double velocityRadPerSec) {
    rollerClosedLoop = true;
    rollerVelocitySetpointRadPerSec = velocityRadPerSec;
  }
}
