// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.superstructure.Superstructure;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Separate test class from SuperstructureIntakeSequenceEarlyExitTest and
 * SuperstructureIntakeSequenceTimeoutFallbackTest deliberately -- build.gradle's forkEvery = 1
 * forks a fresh JVM per test CLASS, not per method. Superstructure/Intake/Shooter singletons have
 * no reset hook, so a second @Test method sharing this class (or JVM) would silently observe
 * state left behind by the first; keeping each conditional-sequence scenario in its own class is
 * what gives it its own JVM.
 */
class SuperstructureShootSequenceEarlyExitTest {
  private Robot robot;
  private Thread robotThread;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    robot = new Robot();
    robotThread = new Thread(robot::startCompetition, "SuperstructureShootSequenceEarlyExitTest-competition");
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
  void shootingSequenceFinishesEarlyAndStillRequestsStow() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.1);

    Superstructure superstructure = Superstructure.getInstance(null);
    boolean[] conditionMet = {false};

    Command sequence = superstructure.shootingSequence(() -> conditionMet[0], 5.0);
    CommandScheduler.getInstance().schedule(sequence);
    SimHooks.stepTiming(0.1);

    conditionMet[0] = true;
    SimHooks.stepTiming(0.1);

    assertFalse(CommandScheduler.getInstance().isScheduled(sequence),
        "shooting sequence should finish as soon as the condition is met");
    assertEquals(Superstructure.SuperstructureState.STOWED, superstructure.getSystemState(),
        "finishing early must still request STOWED -- this is the exact regression risk an "
            + "external .until() wrapper without finallyDo() cleanup would have introduced");
  }
}
