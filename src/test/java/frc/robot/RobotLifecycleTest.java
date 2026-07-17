// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.vision.Vision;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Drives the real {@link Robot#startCompetition()} loop (not direct disabledInit/autonomousInit
 * calls) so AdvantageKit's periodicBeforeUser/periodicAfterUser flush cycle actually runs and the
 * SIM-mode WPILOGWriter (Robot.java) actually receives the Vision/IMUMode records this test
 * asserts on. Kept to a single @Test method deliberately -- see the plan's Global Constraints on
 * forkEvery granularity and the Vision/Intake/Shooter/Superstructure singletons.
 */
class RobotLifecycleTest {
  private Robot robot;
  private Thread robotThread;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    robot = new Robot();
    robotThread = new Thread(robot::startCompetition, "RobotLifecycleTest-competition");
    robotThread.setDaemon(true);
    robotThread.start();
    SimHooks.waitForProgramStart();
  }

  @AfterEach
  void teardown() throws InterruptedException {
    robot.endCompetition();
    robotThread.join(1000);
    robot.close();
    SimHooks.resumeTiming();
  }

  @Test
  @Timeout(30)
  void autonomousInitSwitchesIMUModeFromSeedToFusion() {
    assertTrue(robotThread.isAlive(),
        "startCompetition() loop should still be running after waitForProgramStart()");

    // Disabled: matches a real boot sitting on the field before auto starts.
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.1); // ~5 20ms cycles of disabledPeriodic

    assertEquals(1, Vision.getInstance(null).getIMUMode(),
        "disabledInit/disabledPeriodic should hold IMUMode at 1 (EXTERNAL_SEED)");

    // Autonomous: the actual 2026-07-17 fix under test.
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.1); // ~5 20ms cycles of autonomousPeriodic

    assertEquals(4, Vision.getInstance(null).getIMUMode(),
        "autonomousInit should switch IMUMode to 4 (fusion) -- verifies the auto-snap fix");
  }
}
