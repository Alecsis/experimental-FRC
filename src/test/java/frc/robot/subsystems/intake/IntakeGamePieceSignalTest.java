// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

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
 * Autonomous Observability Phase 1, Stage A / F1: proves hasGamePiece() is a trustworthy signal
 * under simulation, not a recovery-behavior test. Deliberately in package
 * frc.robot.subsystems.intake so it gets package-private access to Intake's
 * provideGamePieceForTest() test hook, same reasoning as IntakeJamRecoveryTest. No Superstructure
 * involvement -- this only proves the signal, it does not wire it into any decision.
 */
class IntakeGamePieceSignalTest {
  private Robot robot;
  private Thread robotThread;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    robot = new Robot();
    robotThread = new Thread(robot::startCompetition, "IntakeGamePieceSignalTest-competition");
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
  void hasGamePieceFlipsTrueOnlyAfterAPieceIsProvided() {
    assertTrue(robotThread.isAlive(),
        "startCompetition() loop should still be running after waitForProgramStart()");

    Intake intake = Intake.getInstance();

    // Let IntakeIOSim's lazily-constructed IntakeSimulation come up (it waits for
    // RobotContainer.drivetrain.getMapleSimDrive() to be non-null on its first updateInputs()).
    SimHooks.stepTiming(0.1);

    assertFalse(intake.hasGamePiece(),
        "hasGamePiece() must not read true before any game piece has been provided");

    boolean provided = intake.provideGamePieceForTest();
    assertTrue(provided,
        "test-only game piece injection should succeed once IntakeIOSim's IntakeSimulation "
            + "exists (capacity is 1, confirmed via Constants.kIntakeSimCapacity)");

    // Let IntakeIOSim.updateInputs() re-read the now-nonzero game piece count into inputs.hasGamePiece.
    SimHooks.stepTiming(0.1);

    assertTrue(intake.hasGamePiece(),
        "hasGamePiece() should flip true once a game piece has been provided to the intake");
  }
}
