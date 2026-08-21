// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.slamtake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.frc2026.Constants;

public class SlamIOSim implements SlamIO {
  private static final double moi = 1.0;
  private static final double reduction = 1.0;
  private static final DCMotor motorModel = DCMotor.getKrakenX60(1);
  private static final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);

  private double currentOutput = 0.0;
  private double appliedVolts = 0.0;
  private boolean currentControl = false;

  public SlamIOSim() {}

  @Override
  public void updateInputs(SlamIOInputs inputs) {
    if (currentControl) {
      appliedVolts =
          MathUtil.clamp(
              motorModel.getVoltage(currentOutput, sim.getAngularVelocityRadPerSec()), -12.0, 12.0);
    } else {
      appliedVolts = 0.0;
    }
    // Update sim state
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.loopPeriodSecs);

    inputs.connected = true;
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = currentOutput;
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void applyOutputs(SlamIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE:
        currentControl = false;
        break;
      case COAST:
        currentOutput = 0.0;
        currentControl = true;
        break;
      case RUN_OPEN_LOOP:
        appliedVolts = outputs.appliedVolts;
        currentControl = false;
        break;
    }
  }
}
