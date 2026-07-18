// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Robot;
import frc.robot.subsystems.superstructure.Superstructure;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Third test class in this repo, deliberately in package frc.robot.subsystems.intake so it gets
 * package-private access to Intake's forceJamConditionForTest()/clearJamOverrideForTest() test
 * hooks. Same real-startCompetition()-loop harness and single-@Test-method-per-class reasoning as
 * RobotLifecycleTest/SuperstructureEjectingTest.
 *
 * Drives INTAKING through Superstructure rather than calling Intake.setRoller() directly:
 * Superstructure is registered and its periodic() runs every tick regardless (constructed as
 * part of the required Robot boot), and its OFF/STOWED cases unconditionally call
 * intake.setRoller(Roller.STOP) -- a direct-call test that never requests INTAKING would have
 * that call fought/overwritten by Superstructure on every intervening tick. Routing through
 * Superstructure.requestIntake()/requestStow() instead matches the only way this logic is
 * actually driven in production and avoids the race entirely.
 */
class IntakeJamRecoveryTest {
  private Robot robot;
  private Thread robotThread;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    robot = new Robot();
    robotThread = new Thread(robot::startCompetition, "IntakeJamRecoveryTest-competition");
    robotThread.setDaemon(true);
    robotThread.start();
    SimHooks.waitForProgramStart();
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
  void jamTriggersRecoveryPulseThenResumesAndAbortsOnStop() {
    assertTrue(robotThread.isAlive(),
        "startCompetition() loop should still be running after waitForProgramStart()");

    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.1);

    Intake intake = Intake.getInstance();
    Superstructure superstructure = Superstructure.getInstance(null);

    // Force a healthy running state (0 amps, high velocity) to get past the spin-up transient safely
    intake.forceJamConditionForTest(0.0, 1000.0);
    superstructure.requestIntake();
    SimHooks.stepTiming(0.5);

    System.out.println("[DEBUG] Roller state at baseline check: " + intake.getRollerState());

    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "INTAKING with no jam should just command the roller INTAKE");

    // Clear our healthy override right before we test the actual jam logic
    intake.clearJamOverrideForTest();

    // Force a jam (high stator current, near-zero velocity -- isJammed()'s real condition).
    // Superstructure.periodic() re-calls setRoller(Roller.INTAKE) every tick during INTAKING,
    // so the recovery pulse fires automatically, the same way it will in production.
    intake.forceJamConditionForTest(100.0, 0.0);
    SimHooks.stepTiming(0.1);

    assertEquals(Intake.Roller.EJECT, intake.getRollerState(),
        "a jam detected during INTAKING should trigger an immediate recovery pulse");

    // Clear the jam (simulates the obstruction clearing during the pulse) and step past the
    // pulse duration (0.3s) -- this is now a stable end state, since nothing will re-trigger a
    // pulse once isJammed() reads false again.
    intake.forceJamConditionForTest(0.0, 1000.0);
    SimHooks.stepTiming(0.4);

    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "after the pulse elapses and the jam clears, INTAKE should resume and stay resumed");

    // Force a second jam, confirm a new pulse fires, then abort mid-pulse via a real caller-level
    // stow request (matches a driver releasing the intake button / auto moving on).
    intake.forceJamConditionForTest(100.0, 0.0);
    SimHooks.stepTiming(0.1);
    assertEquals(Intake.Roller.EJECT, intake.getRollerState(),
        "second jam should trigger a new recovery pulse");

    superstructure.requestStow();
    SimHooks.stepTiming(0.1);

    assertEquals(Intake.Roller.STOP, intake.getRollerState(),
        "requestStow() mid-pulse must abort the recovery pulse immediately, not let it finish");

    intake.clearJamOverrideForTest();
  }
}