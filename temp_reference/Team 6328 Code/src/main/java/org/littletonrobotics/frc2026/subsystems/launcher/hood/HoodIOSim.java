// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.launcher.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.frc2026.Constants;

public class HoodIOSim implements HoodIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX44(1);
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          motorModel,
          1.0,
          .004,
          .33,
          Units.degreesToRadians(19),
          Units.degreesToRadians(45),
          false,
          Units.degreesToRadians(19));

  private double currentOutput = 0.0;
  private double appliedVolts = 0.0;
  private boolean currentControl = false;

  private static final double kP = 30000;
  private static final double kD = 500;

  public HoodIOSim() {}

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    if (currentControl) {
      appliedVolts =
          MathUtil.clamp(
              motorModel.getVoltage(currentOutput, sim.getVelocityRadPerSec()), -12.0, 12.0);
    }

    // Update sim state
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.loopPeriodSecs);

    inputs.motorConnected = true;
    inputs.positionRads = sim.getAngleRads();
    inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = currentOutput;
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> {
        appliedVolts = 0.0;
        currentControl = false;
      }
      case COAST -> {
        currentOutput = 0.0;
        currentControl = true;
      }
      case CLOSED_LOOP -> {
        currentOutput =
            (outputs.positionRad - sim.getAngleRads()) * kP
                + (outputs.velocityRadsPerSec - sim.getVelocityRadPerSec()) * kD;
        currentControl = true;
      }
      case OPEN_LOOP -> {
        appliedVolts = outputs.appliedVolts;
        currentControl = false;
      }
    }
  }
}
