// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.Timeout;

/**
 * The operator board -- and its simulation mirror -- must have exactly zero effect during
 * autonomous.
 *
 * <p>This is the failure mode that killed the previous default-command attempt: a default command
 * runs whenever nothing else requires the subsystem, which includes the whole of autonomous, and
 * it drove the state machine to STOWED (700 RPM flywheel, pivot commanded DOWN) from the instant
 * of enable. The isolation is now structural rather than incidental -- {@code operatorPolicyCmd}
 * returns without issuing any request unless {@code DriverStation.isTeleopEnabled()} -- so this
 * class mashes every control, on both HIDs, and asserts nothing moves.
 *
 * <p>Asserting "the state did not change" is deliberately stronger than asserting a particular
 * state: it holds regardless of what the selected auto (if any) happens to be doing.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class OperatorBoxAutonomousIsolationTest {
  private static final int kControlBoxPort = 1;
  private static final int kSimMirrorPort = 3;

  /** Xbox button indices on the sim-mirror controller: A=1, B=2, X=3, Y=4. */
  private static final int kSimA = 1;
  private static final int kSimB = 2;
  private static final int kSimX = 3;
  private static final int kSimY = 4;

  private static Robot robot;
  private static Thread robotThread;
  private static Superstructure superstructure;
  private static Intake intake;
  private static Shooter shooter;

  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    DriverStationSim.setJoystickButtonCount(kControlBoxPort, 12);
    DriverStationSim.setJoystickButtonCount(kSimMirrorPort, 12);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    robot = new Robot();
    robotThread = SimRobotLoop.start(robot, "OperatorBoxAutoIsolationTest-competition");
    SimRobotLoop.awaitProgramStart();

    superstructure = Superstructure.getInstance(null);
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();

    SimRobotLoop.registerDiagnostics(
        () -> SimRobotLoop.operatorBoardState(kControlBoxPort, 6));

    // AUTONOMOUS, not teleop.
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(0.5);
  }

  @AfterAll
  static void teardown() {
    for (int button = 1; button <= 6; button++) {
      DriverStationSim.setJoystickButton(kControlBoxPort, button, false);
      DriverStationSim.setJoystickButton(kSimMirrorPort, button, false);
    }
    SimRobotLoop.shutdown();
  }

  private static void step(double seconds) {
    SimRobotLoop.step(seconds);
  }

  private static void setBoard(int port, boolean pressed, int... buttons) {
    for (int button : buttons) {
      DriverStationSim.setJoystickButton(port, button, pressed);
    }
    DriverStationSim.notifyNewData();
  }

  @Test
  @Order(1)
  @Timeout(90)
  void controlBoxDuringAutonomousChangesNothing() {
    assertTrue(DriverStationSim.getAutonomous(), "precondition: still in autonomous");

    SuperstructureState stateBefore = superstructure.getSystemState();
    Intake.Roller rollerBefore = intake.getRollerState();
    Shooter.indexing indexerBefore = shooter.getIndexerState();
    Shooter.Agitate agitatorBefore = shooter.getAgitateState();
    double rpmBefore = shooter.getTargetRPM();
    Command superstructureOwnerBefore = CommandScheduler.getInstance().requiring(superstructure);

    // Every board control at once, including both maintained toggles and all three momentaries.
    setBoard(kControlBoxPort, true, 1, 2, 3, 4, 5, 6);
    step(1.0);

    assertEquals(stateBefore, superstructure.getSystemState(),
        "operator-box controls must not change the Superstructure state during autonomous");
    assertEquals(rollerBefore, intake.getRollerState(),
        "operator-box controls must not touch the intake roller during autonomous");
    assertEquals(indexerBefore, shooter.getIndexerState(),
        "operator-box controls must not touch the indexer during autonomous");
    assertEquals(agitatorBefore, shooter.getAgitateState(),
        "operator-box controls must not touch the agitator during autonomous");
    assertEquals(rpmBefore, shooter.getTargetRPM(), 1e-9,
        "operator-box controls must not change the shooter target during autonomous");
    assertSame(superstructureOwnerBefore, CommandScheduler.getInstance().requiring(superstructure),
        "no new Superstructure command may be scheduled by the board during autonomous -- the "
            + "operator policy is present but inert, and nothing else appears");

    setBoard(kControlBoxPort, false, 1, 2, 3, 4, 5, 6);
    step(0.2);
    assertEquals(stateBefore, superstructure.getSystemState(),
        "releasing the board during autonomous must be equally inert");
  }

  @Test
  @Order(2)
  @Timeout(90)
  void simulationMirrorDuringAutonomousChangesNothing() {
    assertTrue(DriverStationSim.getAutonomous(), "precondition: still in autonomous");

    SuperstructureState stateBefore = superstructure.getSystemState();
    Intake.Roller rollerBefore = intake.getRollerState();
    Shooter.indexing indexerBefore = shooter.getIndexerState();
    double rpmBefore = shooter.getTargetRPM();

    // The sim mirror is OR'd into the same suppliers, so it inherits the same teleop gate rather
    // than being a second, separately-bound control surface with its own lifecycle.
    setBoard(kSimMirrorPort, true, kSimA, kSimB, kSimX, kSimY);
    step(1.0);

    assertEquals(stateBefore, superstructure.getSystemState(),
        "simulation mirror controls must not change the Superstructure state during autonomous");
    assertEquals(rollerBefore, intake.getRollerState(),
        "simulation mirror controls must not touch the intake roller during autonomous");
    assertEquals(indexerBefore, shooter.getIndexerState(),
        "simulation mirror controls must not touch the indexer during autonomous");
    assertEquals(rpmBefore, shooter.getTargetRPM(), 1e-9,
        "simulation mirror controls must not change the shooter target during autonomous");

    setBoard(kSimMirrorPort, false, kSimA, kSimB, kSimX, kSimY);
    step(0.2);
  }

  @Test
  @Order(3)
  @Timeout(90)
  void theSameControlsDoTakeEffectOnceTeleopBegins() {
    // Proves the isolation above is the teleop gate doing its job, not the board being unwired.
    setBoard(kControlBoxPort, true, 6);
    step(0.5);
    assertEquals(SuperstructureState.OFF, superstructure.getSystemState(),
        "still autonomous: toggle 6 is inert");

    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    // Same physical switch position, no button edge -- only the mode changed.
    for (double elapsed = 0; elapsed < 4.0; elapsed += 0.02) {
      if (superstructure.getSystemState() == SuperstructureState.INTAKING) {
        break;
      }
      step(0.02);
    }
    assertEquals(SuperstructureState.INTAKING, superstructure.getSystemState(),
        "the identical switch position must take effect the moment teleop begins, with no button "
            + "edge -- confirming autonomous isolation came from the mode gate, not dead wiring");
  }
}
