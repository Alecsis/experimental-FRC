// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean pivotConnected = false;
    public double pivotPositionRads = 0.0;
    public double pivotVelocityRadsPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotStatorCurrentAmps = 0.0;
    public double pivotSupplyCurrentAmps = 0.0;
    public double pivotTempCelsius = 0.0;

    public boolean rollerConnected = false;
    public double rollerPositionRads = 0.0;
    public double rollerVelocityRadsPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerStatorCurrentAmps = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;
    public double rollerTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(IntakeIOInputs inputs) {}

  /** Runs the pivot motor at the specified open-loop voltage. */
  default void setPivotVoltage(double volts) {}

  /** Runs the pivot motor closed-loop to the specified position. */
  default void setPivotPosition(double positionRads) {}

  /** Zeroes the pivot's position reference, used for homing against a hard stop. */
  default void setPivotEncoderPosition(double positionRads) {}

  /** Runs the roller motor at the specified open-loop voltage. */
  default void setRollerVoltage(double volts) {}

  /** Runs the roller motor closed-loop to the specified velocity. */
  default void setRollerVelocity(double velocityRadPerSec) {}
}
