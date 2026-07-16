// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean shootConnected = false;
    public double shootPositionRads = 0.0;
    public double shootVelocityRadsPerSec = 0.0;
    public double shootAppliedVolts = 0.0;
    public double shootStatorCurrentAmps = 0.0;
    public double shootSupplyCurrentAmps = 0.0;
    public double shootTempCelsius = 0.0;

    public boolean indexConnected = false;
    public double indexPositionRads = 0.0;
    public double indexVelocityRadsPerSec = 0.0;
    public double indexAppliedVolts = 0.0;
    public double indexStatorCurrentAmps = 0.0;
    public double indexSupplyCurrentAmps = 0.0;
    public double indexTempCelsius = 0.0;

    public boolean agitatorConnected = false;
    public double agitatorPositionRads = 0.0;
    public double agitatorVelocityRadsPerSec = 0.0;
    public double agitatorAppliedVolts = 0.0;
    public double agitatorStatorCurrentAmps = 0.0;
    public double agitatorSupplyCurrentAmps = 0.0;
    public double agitatorTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ShooterIOInputs inputs) {}

  /** Runs the shooter flywheel motor closed-loop to the specified velocity. */
  default void setShootVelocity(double velocityRadPerSec) {}

  /** Runs the shooter flywheel motor at the specified open-loop voltage. */
  default void setShootVoltage(double volts) {}

  /** Runs the index motor at the specified open-loop voltage. */
  default void setIndexVoltage(double volts) {}

  /** Runs the agitator motor at the specified open-loop voltage. */
  default void setAgitatorVoltage(double volts) {}
}
