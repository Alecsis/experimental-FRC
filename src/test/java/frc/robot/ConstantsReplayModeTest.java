// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.Constants.Mode;
import org.junit.jupiter.api.Test;

/**
 * Pure unit tests for Constants.resolveCurrentMode(), the logic that decides whether
 * Constants.currentMode is REAL/SIM/REPLAY. No HAL/robot boot needed -- this is a plain static
 * function. Exercises the exact three-way branch previously only expressible as one hardcoded
 * ternary (RobotBase.isReal() ? REAL : simMode), which made REPLAY permanently unreachable
 * regardless of any environment variable, per SKILLS/replay-testing-agent.md's prior finding.
 */
class ConstantsReplayModeTest {
  @Test
  void realHardwareAlwaysWinsRegardlessOfEnvVarOrSimMode() {
    assertEquals(Mode.REAL, Constants.resolveCurrentMode(true, "some/log/path.wpilog", Mode.SIM));
    assertEquals(Mode.REAL, Constants.resolveCurrentMode(true, null, Mode.REPLAY));
  }

  @Test
  void noReplayEnvVarFallsBackToSimMode() {
    assertEquals(Mode.SIM, Constants.resolveCurrentMode(false, null, Mode.SIM));
  }

  @Test
  void replayEnvVarSetSelectsReplayRegardlessOfSimMode() {
    assertEquals(
        Mode.REPLAY, Constants.resolveCurrentMode(false, "logs/akit_example.wpilog", Mode.SIM));
  }
}
