// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

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
 * SuperstructureShootSequenceEarlyExitTest deliberately -- build.gradle's forkEvery = 1 forks a
 * fresh JVM per test CLASS, not per method. Superstructure/Intake/Shooter singletons have no
 * reset hook, so a second @Test method sharing this class (or JVM) would silently observe state
 * left behind by the first; keeping each conditional-sequence scenario in its own class is what
 * gives it its own JVM.
 */
class SuperstructureIntakeSequenceTimeoutFallbackTest {
  private Robot robot;
  private Thread robotThread;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    robot = new Robot();
    robotThread = new Thread(robot::startCompetition, "SuperstructureIntakeSequenceTimeoutFallbackTest-competition");
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
}
