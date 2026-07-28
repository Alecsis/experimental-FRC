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
import frc.robot.subsystems.superstructure.Superstructure;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Third test class in this package deliberately -- see {@code SuperstructureEjectingTest}'s own
 * javadoc for why forkEvery=1 means one @Test method per class here.
 *
 * <p>Guards a specific requirements gap: {@code Superstructure.shootCmd()}/{@code intakeCmd()}/
 * {@code ejectCmd()}/{@code stowCmd()} all declare {@code this} (Superstructure) as their
 * Command requirement via {@code Commands.startEnd(..., this)}/{@code Commands.runOnce(..., this)},
 * so the CommandScheduler will cancel one if another is scheduled concurrently -- normal
 * cancel-on-conflict behavior. {@code shootingSequence(double)} (registered as the "Shooting
 * Sequence"/"Quick Shooting" NamedCommands, used by every in-scope auto) composed
 * {@code Commands.runOnce(...)}/{@code Commands.waitUntil(...)} with no subsystem argument at
 * all, so the resulting composite had an empty requirement set -- it could run fully concurrently
 * with a teleop/dashboard {@code shootCmd()}/{@code ejectCmd()} without either being cancelled,
 * both writing {@code mWantedState} on alternating scheduler ticks. Concrete trigger: PathPlanner
 * runs "Shooting Sequence" during autonomous while a SmartDashboard "State: Eject" button (always
 * live over NetworkTables, including during auto) gets pressed.
 */
class SuperstructureCommandRequirementsTest {
  private Robot robot;
  private Thread robotThread;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    robot = new Robot();
    robotThread = new Thread(robot::startCompetition, "SuperstructureCommandRequirementsTest-competition");
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
  void shootingSequenceRequiresSuperstructure() {
    assertTrue(robotThread.isAlive(),
        "startCompetition() loop should still be running after waitForProgramStart()");

    Superstructure superstructure = Superstructure.getInstance(null);
    Command shootingSequence = superstructure.shootingSequence(1.0);

    assertTrue(shootingSequence.getRequirements().contains(superstructure),
        "shootingSequence(double) (the \"Shooting Sequence\"/\"Quick Shooting\" NamedCommands) "
            + "does not require Superstructure -- a concurrently-scheduled shootCmd()/ejectCmd() "
            + "(e.g. a SmartDashboard button pressed during autonomous) would not be cancelled and "
            + "would run alongside it, both writing mWantedState on alternating ticks");
  }
}
