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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Second test class in this package deliberately -- build.gradle's forkEvery = 1 forks a fresh
 * JVM per test CLASS (not per method), which is what keeps this class's Superstructure/Intake/
 * Shooter singletons isolated from RobotLifecycleTest's. Kept to a single @Test method for the
 * same reason RobotLifecycleTest is: those singletons have no reset hook, so a second method in
 * this class would silently observe state left behind by the first.
 */
class SuperstructureEjectingTest {
  private Robot robot;
  private Thread robotThread;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    robot = new Robot();
    robotThread = new Thread(robot::startCompetition, "SuperstructureEjectingTest-competition");
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
  void ejectCmdDrivesEjectingThenCancelStopsRoller() {
    assertTrue(robotThread.isAlive(),
        "startCompetition() loop should still be running after waitForProgramStart()");

    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.1); // ~5 20ms cycles of teleopPeriodic

    Superstructure superstructure = Superstructure.getInstance(null);
    Intake intake = Intake.getInstance();
    Shooter shooter = Shooter.getInstance();

    assertEquals(Intake.Roller.STOP, intake.getRollerState(),
        "nothing scheduled yet: Superstructure's OFF state should hold the roller stopped");

    Command eject = superstructure.ejectCmd();
    CommandScheduler.getInstance().schedule(eject);
    SimHooks.stepTiming(0.1); // let EJECTING's periodic() commands land

    assertEquals(Intake.Roller.EJECT, intake.getRollerState(),
        "EJECTING should drive the intake roller in reverse");
    assertEquals(Shooter.indexing.EJECT, shooter.getIndexerState(),
        "EJECTING should drive the indexer in reverse");
    assertEquals(Shooter.Agitate.OUT, shooter.getAgitateState(),
        "EJECTING should drive the agitator outward");

    CommandScheduler.getInstance().cancel(eject);
    SimHooks.stepTiming(0.1); // let STOWED's periodic() commands land

    assertEquals(Intake.Roller.STOP, intake.getRollerState(),
        "cancelling ejectCmd() should request STOWED, which commands the roller to stop");
  }
}
