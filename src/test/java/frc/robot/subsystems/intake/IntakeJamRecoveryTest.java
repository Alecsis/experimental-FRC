// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
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
 * that call fought/overwritten by Superstructure on every intervening tick.
 *
 * <p>Specifically it schedules {@link Superstructure#intakeCmd()}, not a bare
 * {@code requestIntake()}. Superstructure's operator policy is its DEFAULT command, so in teleop
 * it re-applies the live board intent (STOWED, with nothing pressed) on every single tick; a raw
 * request that holds no Superstructure requirement is overwritten before it can take effect. That
 * is the intended authority model -- every production caller ("Intake Start Sequence", the
 * dashboard buttons, the operator board itself) drives the machine through a Command that
 * requires Superstructure, and this test now does the same.
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

    // Holds the Superstructure requirement for the whole test, exactly like the production
    // "Intake Start Sequence" NamedCommand and the operator board's own intake intent do.
    Command intaking = superstructure.intakeCmd();
    CommandScheduler.getInstance().schedule(intaking);
    // Longer settle window than the other stepTiming(0.1) calls in this test: IntakeIOSim's roller
    // is closed-loop velocity control over a real DCMotorSim, and a 0->800 RPM spin-up briefly
    // produces high current at low velocity -- the same electrical signature isJammed() looks for.
    // 0.1s wasn't enough settle time and this baseline assertion observed a real spin-up transient
    // (EJECT) rather than a broken test; widened per the known-risk note in the migration plan.
    SimHooks.stepTiming(0.5);

    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "INTAKING with no jam should just command the roller INTAKE");

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
    intake.clearJamOverrideForTest();
    SimHooks.stepTiming(0.4);

    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "after the pulse elapses and the jam clears, INTAKE should resume and stay resumed");

    // setRoller() holds off re-arming jam detection for kJamRecoveryCooldownSeconds after a pulse
    // ends (resuming INTAKE from EJECT's speed briefly re-triggers isJammed() otherwise -- see
    // Intake.java). Step past that cooldown before forcing a second, independent jam, or this
    // jam gets legitimately suppressed as still-in-cooldown rather than exercising a fresh pulse.
    SimHooks.stepTiming(0.3);

    // Force a second jam, confirm a new pulse fires, then abort mid-pulse via a real caller-level
    // stow request (matches a driver releasing the intake button / auto moving on).
    intake.forceJamConditionForTest(100.0, 0.0);
    SimHooks.stepTiming(0.1);
    assertEquals(Intake.Roller.EJECT, intake.getRollerState(),
        "second jam should trigger a new recovery pulse");

    // Ending the intake command stows, the same way releasing the operator toggle or the auto
    // sequence timing out does.
    CommandScheduler.getInstance().cancel(intaking);
    SimHooks.stepTiming(0.1);

    assertEquals(Intake.Roller.STOP, intake.getRollerState(),
        "stowing mid-pulse must abort the recovery pulse immediately, not let it finish");

    intake.clearJamOverrideForTest();
  }
}
