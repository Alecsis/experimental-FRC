// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import frc.robot.subsystems.intake.Intake;
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
 * Lifecycle guarantees for the two MAINTAINED switches (5 = pivot bounce, 6 = persistent intake).
 *
 * <p>A maintained switch has no meaningful "press". Its physical position is the intent, and that
 * position can be established at any time -- including while the robot is disabled, when no edge
 * is observable at all. Every test here therefore manipulates the DriverStation enable state, not
 * just buttons, and several deliberately produce NO button transition whatsoever between the
 * setup and the assertion: the point is that the switch takes effect anyway.
 *
 * <p>These are the cases an edge-triggered {@code Trigger.whileTrue} binding cannot satisfy, and
 * they are why the board is wired as Superstructure's default command instead.
 *
 * <p>One Robot per JVM (build.gradle {@code forkEvery = 1} plus the static drivetrain closed by
 * {@code RobotContainer.close()}); methods are ordered and each establishes its own enable/switch
 * preconditions explicitly rather than inheriting them.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class OperatorBoxMaintainedSwitchLifecycleTest {
  private static final int kControlBoxPort = 1;
  private static final int kSimMirrorPort = 3;

  private static final int kUnifiedEject = 3;
  private static final int kBounceToggle = 5;
  private static final int kIntakeToggle = 6;

  private static Robot robot;
  private static Thread robotThread;
  private static Superstructure superstructure;
  private static Intake intake;

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
    robotThread = SimRobotLoop.start(robot, "OperatorBoxLifecycleTest-competition");
    SimRobotLoop.awaitProgramStart();

    superstructure = Superstructure.getInstance(null);
    intake = Intake.getInstance();

    SimRobotLoop.registerDiagnostics(
        () -> SimRobotLoop.operatorBoardState(kControlBoxPort, 6));
  }

  @AfterAll
  static void teardown() {
    setControl(kBounceToggle, false);
    setControl(kIntakeToggle, false);
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

  private static void enableTeleop() {
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(0.1);
  }

  private static void disableRobot() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    step(0.2);
  }

  private static void stepUntil(BooleanSupplier condition, double timeoutSeconds, String what) {
    SimRobotLoop.stepUntil(condition, timeoutSeconds, what);
  }

  /** The command currently holding Intake, or null. During teleop entry this is intake.homing(). */
  private static Command intakeOwner() {
    return CommandScheduler.getInstance().requiring(intake);
  }

  private static double pivotDegrees() {
    return SmartDashboard.getNumber("Intake/Pivot Deg", Double.NaN);
  }

  /**
   * Waits for teleop's {@code intake.homing()} to finish and release Intake. Homing is bounded by
   * {@code .withTimeout(1)} plus two instant steps, so this returns well inside its budget unless
   * something interrupted or stalled it.
   */
  private static void settleThroughHoming() {
    stepUntil(() -> intakeOwner() == null || superstructure.getSystemState() == SuperstructureState.BOUNCING,
        4.0, "intake.homing() should have completed and released Intake");
    step(0.1);
  }

  /* -------------------------------------------------------------------- tests */

  @Test
  @Order(1)
  @Timeout(90)
  void intakeToggleAlreadyOnBeforeTeleopEnableTakesEffectWithoutCycling() {
    // The switch is flipped while the robot is DISABLED. There is no enabled false->true button
    // edge anywhere in this test -- the only edge is the DriverStation enable.
    setControl(kIntakeToggle, true);
    step(0.2);
    assertNotEquals(SuperstructureState.INTAKING, superstructure.getSystemState(),
        "a disabled robot must not act on the board at all");

    enableTeleop();
    settleThroughHoming();

    assertEquals(SuperstructureState.INTAKING, superstructure.getSystemState(),
        "toggle 6 left ON across the enable must take effect on its own -- the operator must not "
            + "have to cycle the physical switch");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "the resumed intent must actually drive the roller");
  }

  @Test
  @Order(2)
  @Timeout(90)
  void intakeToggleSurvivesDisableThenReEnableWithNoButtonEdge() {
    // Entering with toggle 6 still ON from the previous test. Deliberately no setControl() call
    // anywhere in this method: the switch never moves, only the enable state does.
    assertEquals(SuperstructureState.INTAKING, superstructure.getSystemState(),
        "precondition: intaking with toggle 6 physically ON");

    disableRobot();
    assertNull(CommandScheduler.getInstance().requiring(superstructure),
        "while disabled the scheduler drops the operator policy, so the board drives nothing -- "
            + "and there is no edge for it to miss when it comes back");

    enableTeleop();
    settleThroughHoming();

    assertEquals(SuperstructureState.INTAKING, superstructure.getSystemState(),
        "toggle 6 still physically ON across a disable/enable cycle must resume intaking with no "
            + "switch edge of any kind");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "resumed intake must drive the roller again");
  }

  @Test
  @Order(3)
  @Timeout(90)
  void bounceToggleAlreadyOnBeforeTeleopEnableTakesEffectAfterHomingReleasesIntake() {
    disableRobot();
    setControl(kIntakeToggle, false);
    setControl(kBounceToggle, true); // established while disabled
    step(0.2);

    enableTeleop();

    // While homing owns the pivot, the bounce routine must NOT be scheduled on top of it, but the
    // roller must already reflect the live operator intent rather than a frozen stale value.
    Command ownerDuringHoming = intakeOwner();
    assertNotNull(ownerDuringHoming, "intake.homing() should own Intake right after teleop enable");
    double pivotAtHomingStart = pivotDegrees();

    boolean sawBounceDuringHoming = false;
    double maxPivotDuringHoming = pivotAtHomingStart;
    for (int i = 0; i < 60 && intakeOwner() == ownerDuringHoming; i++) {
      if (intakeOwner() != ownerDuringHoming) {
        sawBounceDuringHoming = true;
      }
      maxPivotDuringHoming = Math.max(maxPivotDuringHoming, pivotDegrees());
      step(0.02);
    }
    assertFalse(sawBounceDuringHoming,
        "the bounce routine must never take the pivot away from intake.homing()");
    // Homing drives the pivot with +3V toward the hardstop. Superstructure yields the pivot for
    // the whole window, so that voltage is unopposed and the arm rises off its start angle. A
    // competing goTo(DOWN)/goTo(AGITATE) position request would hold or pull it the other way.
    assertTrue(maxPivotDuringHoming > pivotAtHomingStart + 1.0,
        "the homing voltage must have been free to drive the pivot toward the hardstop rather "
            + "than being fought by a competing position request (start " + pivotAtHomingStart
            + " deg, max seen " + maxPivotDuringHoming + " deg)");
    // The roller is a SEPARATE ownership axis: homing drives the pivot only, so the roller must
    // still be following live operator intent here, not frozen at some earlier stale request.
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "toggle 5 keeps the roller running even while homing owns the pivot -- suppressing roller "
            + "writes for the whole homing window is what let a stale roller request survive");

    settleThroughHoming();
    assertEquals(SuperstructureState.BOUNCING, superstructure.getSystemState(),
        "toggle 5 left ON across the enable must begin bouncing once homing releases Intake");
    assertNotNull(intakeOwner(), "the bounce routine should own the pivot once homing is done");
    assertNotSame(ownerDuringHoming, intakeOwner(),
        "the bounce routine is a different command from homing");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "legacy toggle 5 keeps the intake roller running while the pivot bounces");
  }

  @Test
  @Order(4)
  @Timeout(90)
  void bounceToggleSurvivesDisableThenReEnableWithNoButtonEdge() {
    assertEquals(SuperstructureState.BOUNCING, superstructure.getSystemState(),
        "precondition: bouncing with toggle 5 physically ON");

    disableRobot();
    enableTeleop();
    settleThroughHoming();

    assertEquals(SuperstructureState.BOUNCING, superstructure.getSystemState(),
        "toggle 5 still physically ON across a disable/enable cycle must resume bouncing after "
            + "homing, with no switch edge");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "resumed bounce keeps the roller running");
  }

  @Test
  @Order(5)
  @Timeout(90)
  void bounceToggleReassertsAfterAnInterruptingSuperstructureCommandFinishes() {
    assertEquals(SuperstructureState.BOUNCING, superstructure.getSystemState(),
        "precondition: bouncing with toggle 5 physically ON");

    // Any command requiring Superstructure evicts the operator policy -- a NamedCommand, a
    // SmartDashboard button, anything. Toggle 5 never moves during this.
    Command interrupter = Commands.run(() -> {
    }, superstructure).withTimeout(0.3);
    CommandScheduler.getInstance().schedule(interrupter);
    step(0.06);
    assertSame(interrupter, CommandScheduler.getInstance().requiring(superstructure),
        "precondition: the interrupter has taken Superstructure from the operator policy");

    stepUntil(() -> !interrupter.isScheduled(), 2.0, "the interrupter should finish on its own");

    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.BOUNCING, 3.0,
        "once the interrupting command releases Superstructure, a still-ON toggle 5 must reassert "
            + "automatically -- no whileTrue rescheduling, no latch, no operator action");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "the reasserted bounce must drive the roller again");
  }

  @Test
  @Order(6)
  @Timeout(90)
  void intakeToggleReassertsAfterAnInterruptingSuperstructureCommandFinishes() {
    setControl(kBounceToggle, false);
    setControl(kIntakeToggle, true);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.INTAKING, 3.0,
        "precondition: intaking with toggle 6 physically ON");

    Command interrupter = Commands.run(() -> {
    }, superstructure).withTimeout(0.3);
    CommandScheduler.getInstance().schedule(interrupter);
    step(0.06);
    assertSame(interrupter, CommandScheduler.getInstance().requiring(superstructure),
        "precondition: the interrupter has taken Superstructure from the operator policy");

    stepUntil(() -> !interrupter.isScheduled(), 2.0, "the interrupter should finish on its own");

    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.INTAKING, 3.0,
        "once the interrupting command releases Superstructure, a still-ON toggle 6 must reassert "
            + "automatically");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "the reasserted intake must drive the roller again");
  }

  @Test
  @Order(7)
  @Timeout(90)
  void momentaryEjectStillOverridesAndReleasesBackToTheMaintainedSwitch() {
    // The maintained-switch machinery must not have cost the momentary overrides their priority.
    assertEquals(SuperstructureState.INTAKING, superstructure.getSystemState(),
        "precondition: intaking with toggle 6 physically ON");

    setControl(kUnifiedEject, true);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.EJECTING, 2.0,
        "button 3 must override the maintained intake toggle");

    setControl(kUnifiedEject, false);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.INTAKING, 2.0,
        "releasing button 3 with toggle 6 still ON must resume intaking");
  }

  @Test
  @Order(8)
  @Timeout(90)
  void operatorPolicyIsTheOnlySuperstructureCommandAndNeedsNoRegistrationOrder() {
    // STOWED is the zero-active-control teleop intent, produced by the same single command that
    // produces every other intent. There is no separate teleop-start stow command, so nothing can
    // race the operator policy for Superstructure at the teleop edge and correctness cannot depend
    // on the order RobotModeTriggers were registered in.
    setControl(kIntakeToggle, false);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.STOWED, 3.0,
        "releasing every control must settle to STOWED via the operator policy itself");

    Command policy = CommandScheduler.getInstance().requiring(superstructure);
    assertNotNull(policy, "the operator policy runs as Superstructure's default command");

    disableRobot();
    enableTeleop();
    settleThroughHoming();

    Command afterCycle = CommandScheduler.getInstance().requiring(superstructure);
    assertNotNull(afterCycle, "the default command must be rescheduled after a disable/enable");
    assertSame(policy, afterCycle,
        "the same default command instance is rescheduled -- exactly one Superstructure command "
            + "exists across the teleop edge, so there is nothing for it to race");
    assertEquals(SuperstructureState.STOWED, superstructure.getSystemState(),
        "with no control active the teleop intent is STOWED, established by the policy itself");
    assertNull(intakeOwner(), "homing released Intake and nothing else claimed it");
  }

  /**
   * Finding 8: a roller request that is already stale when {@code intake.homing()} starts must be
   * actively driven to the safe value DURING homing, not frozen for its whole duration.
   *
   * <p>The stale request is deliberately made by a NON-OPERATOR writer -- the same direct
   * {@code requestIntake()} call the "Intake Start Sequence" NamedCommand's bridge makes. It has to
   * be. An operator-derived request cannot reach the far side of a disable at all any more: the
   * operator policy's {@code finallyDo(clearOperatorRequest)} neutralizes it at exactly that
   * boundary, which {@code OperatorStateCleanupTest} asserts as required behavior. Rebuilding this
   * scenario on a non-operator writer is not a workaround for that cleanup, it is the case the
   * cleanup deliberately does not cover: {@code clearOperatorRequest()} is a no-op unless the
   * operator policy is the current writer, so a non-operator request survives the disable and is
   * genuinely still live when homing begins.
   *
   * <p>What must hold is unchanged from the original scenario. Homing owns the PIVOT and never the
   * roller, so Superstructure has to keep writing the roller through the homing window. If it stood
   * down from roller writes for the duration -- inferring ownership from "some Command requires
   * Intake" rather than declaring it per mechanism -- the roller would simply hold whatever it was
   * last told and spin for the entire home with nothing asking it to.
   */
  @Test
  @Order(9)
  @Timeout(90)
  void aStaleNonOperatorRollerRequestIsCancelledDuringHomingNotFrozenThroughIt() {
    // Start from a disabled robot with an idle board, so nothing operator-derived is in play and
    // the only live request is the one this test makes itself.
    setControl(kBounceToggle, false);
    setControl(kIntakeToggle, false);
    disableRobot();

    // ---- before homing begins: a deliberately stale, non-STOP roller request.
    assertNull(intakeOwner(),
        "precondition: no Command owns Intake while disabled -- homing has not started yet");
    superstructure.requestIntake();
    stepUntil(() -> intake.getRollerState() == Intake.Roller.INTAKE, 1.0,
        "precondition: a non-operator INTAKE request actually drives the roller");
    assertEquals(SuperstructureState.INTAKING, superstructure.getSystemState(),
        "precondition: the non-operator request is the live wanted state");

    // ---- enabling teleop is the rising edge that schedules intake.homing().
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(0.02);

    Command homingOwner = intakeOwner();
    assertNotNull(homingOwner, "intake.homing() should own the pivot right after teleop enable");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "precondition: the stale roller request was still live at the instant homing took the "
            + "pivot -- if it had already been cleared before homing started, the rest of this "
            + "test would prove nothing about the homing window");

    // ---- the roller must be driven to the safe value while homing STILL owns the pivot.
    boolean clearedDuringHoming = false;
    for (int i = 0; i < 60 && intakeOwner() == homingOwner; i++) {
      if (intake.getRollerState() == Intake.Roller.STOP) {
        clearedDuringHoming = true;
        break;
      }
      step(0.02);
    }
    assertTrue(clearedDuringHoming,
        "a stale intake-roller request must be driven to STOP while homing is still running -- "
            + "homing owns the pivot, never the roller, so freezing roller writes for its duration "
            + "leaves the roller spinning with nothing asking for it");
    assertSame(homingOwner, intakeOwner(),
        "and it must have happened WHILE homing still owned the pivot. Observing STOP after homing "
            + "released Intake would be satisfied by the frozen-roller bug too, since the roller "
            + "would resume tracking the wanted state the moment the window ended");
    assertEquals(SuperstructureState.STOWED, superstructure.getSystemState(),
        "the roller followed the CURRENT wanted state rather than the stale one -- with an idle "
            + "board the teleop intent is STOWED, and STOWED is what commands Roller.STOP");

    // ---- homing was never interrupted or fought: it finishes on its own and releases the pivot.
    stepUntil(() -> intakeOwner() == null, 4.0,
        "intake.homing() must complete under its own timeout and release Intake -- Superstructure "
            + "keeping roller authority must not have cost homing the pivot");
    assertEquals(Intake.Roller.STOP, intake.getRollerState(),
        "and the roller stays at the safe value once homing is done");
  }
}
