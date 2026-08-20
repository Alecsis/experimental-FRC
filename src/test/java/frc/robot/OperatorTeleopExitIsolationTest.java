// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.Timeout;

/**
 * The teleop-exit boundary: a DIRECT teleop -&gt; autonomous lifecycle transition must not carry
 * operator-derived Superstructure state into autonomous.
 *
 * <p>This is deliberately not covered by the operator policy command's own end/interruption
 * cleanup, and cannot be. On this transition nothing is guaranteed to interrupt the default
 * command -- an auto whose selected command does not require Superstructure never takes it away --
 * so a fix that only hooks command interruption leaves the last teleop request latched and
 * {@code Superstructure.periodic()} happily keeps applying it for the whole autonomous period. The
 * invariant is therefore hung off the robot lifecycle instead: {@code Robot.teleopExit()} runs on
 * every exit from teleop, BEFORE the next mode's {@code xxxInit()}, and relinquishes the stored
 * operator request there.
 *
 * <p>Each test holds its control DOWN across the mode change rather than releasing it first. That
 * is both the harsher case and a simultaneous re-check of the autonomous isolation gate: the board
 * is physically active throughout autonomous and must still produce nothing.
 *
 * <p>The assertions are made after a deliberately tight 2-loop settle. Nothing but a cleanup that
 * had already run by the time autonomous's first {@code periodic()} executed can satisfy that
 * bound, which is what distinguishes "neutralized at the boundary" from "eventually neutralized".
 *
 * <p>One Robot per JVM (build.gradle {@code forkEvery = 1}); methods are ordered and each
 * re-establishes teleop for itself.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class OperatorTeleopExitIsolationTest {
  private static final int kControlBoxPort = 1;
  private static final int kSimMirrorPort = 3;

  private static final int kFallbackShot = 1;
  private static final int kUnifiedEject = 3;

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
    robotThread = SimRobotLoop.start(robot, "OperatorTeleopExitIsolationTest-competition");
    SimRobotLoop.awaitProgramStart();

    superstructure = Superstructure.getInstance(null);
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();

    SimRobotLoop.registerDiagnostics(
        () -> SimRobotLoop.operatorBoardState(kControlBoxPort, 6));
  }

  @AfterAll
  static void teardown() {
    releaseAllControls();
    SimRobotLoop.shutdown();
  }

  /* ------------------------------------------------------------------ helpers */

  private static void step(double seconds) {
    SimRobotLoop.step(seconds);
  }

  private static void setControl(int button, boolean pressed) {
    DriverStationSim.setJoystickButton(kControlBoxPort, button, pressed);
    DriverStationSim.notifyNewData();
  }

  private static void releaseAllControls() {
    for (int button = 1; button <= 6; button++) {
      DriverStationSim.setJoystickButton(kControlBoxPort, button, false);
    }
    DriverStationSim.notifyNewData();
  }

  private static void stepUntil(BooleanSupplier condition, double timeoutSeconds, String what) {
    SimRobotLoop.stepUntil(condition, timeoutSeconds, what);
  }

  /** Enabled teleop with an idle board and {@code intake.homing()} finished. */
  private static void settleIdleTeleop() {
    releaseAllControls();
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(0.1);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.STOWED, 4.0,
        "an idle board in teleop settles to STOWED");
  }

  /** The direct lifecycle transition under test: teleop straight into autonomous, no disable. */
  private static void enterAutonomousDirectlyFromTeleop() {
    assertFalse(DriverStationSim.getAutonomous(), "precondition: currently in teleop");
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    // Two 20ms loops only. teleopExit() runs before autonomousInit(), so a cleanup anchored to the
    // mode exit has already landed before autonomous's first periodic(); anything slower fails.
    step(0.04);
    assertTrue(DriverStationSim.getAutonomous(), "the robot should now be in autonomous");
  }

  /** Asserts the machine is neutral and no feed/eject output is live. */
  private static void assertNoOperatorDerivedStateLive(String context) {
    assertEquals(SuperstructureState.STOWED, superstructure.getSystemState(),
        context + " -- the stored wanted state must be the neutral STOWED posture");
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
        context + " -- indexer intent must be exactly STOP");
    assertEquals(Shooter.Agitate.STOP, shooter.getAgitateState(),
        context + " -- agitator intent must be exactly STOP");
    assertEquals(Intake.Roller.STOP, intake.getRollerState(),
        context + " -- intake roller intent must be exactly STOP");
  }

  /* -------------------------------------------------------------------- tests */

  /**
   * Required behavior 7: teleop EJECTING -&gt; teleopExit -&gt; autonomous. No stale teleop EJECT
   * request may appear in autonomous.
   */
  @Test
  @Order(1)
  @Timeout(90)
  void teleopEjectDoesNotSurviveIntoAutonomous() {
    settleIdleTeleop();

    setControl(kUnifiedEject, true);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.EJECTING, 3.0,
        "precondition: button 3 held drives EJECTING in teleop");
    assertEquals(Shooter.indexing.EJECT, shooter.getIndexerState(),
        "precondition: the indexer is genuinely running backwards before the mode change");
    assertEquals(Intake.Roller.EJECT, intake.getRollerState(),
        "precondition: the roller is genuinely running backwards before the mode change");

    // Button 3 stays DOWN across the transition.
    enterAutonomousDirectlyFromTeleop();

    assertNoOperatorDerivedStateLive(
        "a teleop EJECT must not survive into autonomous");

    // And it must stay gone: the board is still physically active, but the policy is teleop-gated,
    // so nothing re-establishes the request either.
    step(1.0);
    assertNoOperatorDerivedStateLive(
        "the eject must stay gone for the whole autonomous period, with button 3 still held");

    setControl(kUnifiedEject, false);
    step(0.1);
  }

  /**
   * Required behavior 8: teleop SHOOTING -&gt; teleopExit -&gt; autonomous. No stale feed may
   * appear in autonomous.
   */
  @Test
  @Order(2)
  @Timeout(90)
  void teleopShotFeedDoesNotSurviveIntoAutonomous() {
    // Back out of autonomous into teleop first.
    settleIdleTeleop();

    setControl(kFallbackShot, true);
    stepUntil(() -> shooter.getIndexerState() == Shooter.indexing.INDEX, 8.0,
        "precondition: the fallback shot spun up and is feeding in teleop");
    assertEquals(SuperstructureState.SHOOTING, superstructure.getSystemState(),
        "precondition: feeding happens in SHOOTING");
    assertEquals(Shooter.Agitate.IN, shooter.getAgitateState(),
        "precondition: the shot feed also runs the agitator inward");

    // Button 1 stays DOWN across the transition.
    enterAutonomousDirectlyFromTeleop();

    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
        "a teleop shot feed must not survive into autonomous");
    assertEquals(Shooter.Agitate.STOP, shooter.getAgitateState(),
        "a teleop shot agitator command must not survive into autonomous");
    assertNotEquals(SuperstructureState.SHOOTING, superstructure.getSystemState(),
        "autonomous must not begin in the teleop shot's feeding state");
    assertNotEquals(SuperstructureState.ALIGNING, superstructure.getSystemState(),
        "nor in its spin-up phase, which would feed as soon as the flywheel settled");

    step(1.0);
    assertNoOperatorDerivedStateLive(
        "the shot must stay gone for the whole autonomous period, with button 1 still held");

    setControl(kFallbackShot, false);
    step(0.1);
  }

  /**
   * The other half of the invariant: the cleanup must be a ONE-SHOT relinquish at the boundary,
   * never a continuous write. An autonomous Superstructure request made after the transition must
   * stand -- if teleop-exit cleanup were implemented as a periodic "stow unless teleop" it would
   * overwrite this on the very next tick.
   */
  @Test
  @Order(3)
  @Timeout(90)
  void anAutonomousRequestMadeAfterTheTransitionIsNotOverwritten() {
    settleIdleTeleop();

    setControl(kUnifiedEject, true);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.EJECTING, 3.0,
        "precondition: an operator request is live at the moment teleop ends");

    enterAutonomousDirectlyFromTeleop();
    setControl(kUnifiedEject, false);
    assertEquals(SuperstructureState.STOWED, superstructure.getSystemState(),
        "precondition: the operator request was relinquished at the boundary");

    // Stand in for autonomous Superstructure code: exactly the same call the "Intake Start
    // Sequence" NamedCommand's bridge makes.
    superstructure.requestIntake();
    step(0.2);

    assertEquals(SuperstructureState.INTAKING, superstructure.getSystemState(),
        "an autonomous Superstructure request must stand -- the teleop-exit cleanup is a one-shot "
            + "relinquish at the mode boundary, not a continuous STOWED write through autonomous");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "and it must actually reach the mechanism");

    step(1.0);
    assertEquals(SuperstructureState.INTAKING, superstructure.getSystemState(),
        "it must still stand a full second later, with the operator policy scheduled and running "
            + "its (inert) execute() on every one of those ticks");

    superstructure.requestStow();
    step(0.1);
  }
}
