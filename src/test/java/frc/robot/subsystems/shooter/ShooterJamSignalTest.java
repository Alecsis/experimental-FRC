// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import frc.robot.Robot;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Autonomous Observability Phase 1, Stage A / F3: proves Shooter.isJammed() is a trustworthy
 * signal under a controlled, forced condition -- mirrors IntakeJamRecoveryTest's
 * forceJamConditionForTest()/clearJamOverrideForTest() pattern (Intake.java:168-177), ported to
 * Shooter. Deliberately in package frc.robot.subsystems.shooter for package-private access to
 * isJammed() and the new test hooks. Does not touch Superstructure's shooting-sequence
 * transitions and does not implement any jam recovery -- signal validation only.
 */
class ShooterJamSignalTest {
  private Robot robot;
  private Thread robotThread;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    robot = new Robot();
    robotThread = new Thread(robot::startCompetition, "ShooterJamSignalTest-competition");
    robotThread.setDaemon(true);
    robotThread.start();
    SimHooks.waitForProgramStart();

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void teardown() throws InterruptedException {
    try {
      robot.endCompetition();
      robotThread.join(1000);
      assertFalse(robotThread.isAlive(),
          "startCompetition() loop should have exited within 1s of endCompetition()");
      robot.close();
    } finally {
      SimHooks.resumeTiming();
    }
  }

  @Test
  @Timeout(30)
  void isJammedReflectsForcedConditionThenClears() {
    assertTrue(robotThread.isAlive(),
        "startCompetition() loop should still be running after waitForProgramStart()");

    Shooter shooter = Shooter.getInstance();
    SimHooks.stepTiming(0.1);

    assertFalse(shooter.isJammed(),
        "isJammed() should read false before any jam condition is forced");

    // Force a jam (high stator current, near-zero velocity -- isJammed()'s real condition).
    shooter.forceJamConditionForTest(100.0, 0.0);
    SimHooks.stepTiming(0.1);

    assertTrue(shooter.isJammed(),
        "forcing high index stator current + near-zero index velocity should trip isJammed()");

    shooter.clearJamOverrideForTest();
    SimHooks.stepTiming(0.1);

    assertFalse(shooter.isJammed(),
        "clearing the forced override should let isJammed() read false again");
  }
}
