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
 * Second test class in this package deliberately -- build.gradle's forkEvery = 1 forks a fresh
 * JVM per test CLASS, matching SuperstructureEjectingTest's own stated reasoning for why its
 * Superstructure/Intake/Shooter singletons must not be shared with another class's test bodies.
 */
class SuperstructureConditionalSequenceTest {
  private Robot robot;
  private Thread robotThread;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    robot = new Robot();
    robotThread = new Thread(robot::startCompetition, "SuperstructureConditionalSequenceTest-competition");
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
  void intakeSequenceFinishesEarlyWhenConditionMetBeforeTimeout() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.1);

    Superstructure superstructure = Superstructure.getInstance(null);
    boolean[] conditionMet = {false};

    Command sequence = superstructure.intakeSequence(() -> conditionMet[0], 5.0);
    CommandScheduler.getInstance().schedule(sequence);
    SimHooks.stepTiming(0.1); // let it start

    assertTrue(CommandScheduler.getInstance().isScheduled(sequence),
        "sequence should still be running before the condition flips");

    conditionMet[0] = true;
    SimHooks.stepTiming(0.1); // one tick for the scheduler to observe the flipped condition

    assertFalse(CommandScheduler.getInstance().isScheduled(sequence),
        "sequence should finish as soon as the condition is met, well before the 5.0s timeout");
  }

  @Test
  @Timeout(30)
  void intakeSequenceFallsBackToTimeoutWhenConditionNeverMet() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.1);

    Superstructure superstructure = Superstructure.getInstance(null);

    Command sequence = superstructure.intakeSequence(() -> false, 0.2);
    CommandScheduler.getInstance().schedule(sequence);
    SimHooks.stepTiming(0.1);

    assertTrue(CommandScheduler.getInstance().isScheduled(sequence),
        "sequence should still be running before the 0.2s timeout elapses");

    SimHooks.stepTiming(0.3); // past the 0.2s timeout

    assertFalse(CommandScheduler.getInstance().isScheduled(sequence),
        "sequence should finish via the timeout fallback when the condition never fires");
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
