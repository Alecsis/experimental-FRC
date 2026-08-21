// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
 * Lifecycle cleanup of the operator policy's STORED intent.
 *
 * <p>The operator policy is level-triggered and teleop-gated, but the intent it produces is not
 * momentary: it writes a persistent {@code mWantedState} that {@code Superstructure.periodic()}
 * keeps applying in every robot mode, including modes in which the policy's own body returns
 * immediately without reading a single control. That asymmetry -- a stored request whose only
 * author has gone inert -- is the entire subject of this class. Two boundaries close it:
 *
 * <ul>
 *   <li>the policy command's own {@code finallyDo}, covering disabled cancellation, interruption
 *       by another Superstructure-requiring command, and explicit cancellation;
 *   <li>{@code Robot.teleopExit()}, covering mode exit without depending on the command being
 *       interrupted at all (see {@code OperatorTeleopExitIsolationTest}).
 * </ul>
 *
 * <p>Every interruption test here uses a REQUIREMENT-ONLY interrupter -- {@code Commands.run(() ->
 * {}, superstructure)}, which takes the subsystem and then does nothing with it. That is the exact
 * shape that used to leave the previous operator state running for the interrupter's whole
 * duration, and it is the reason these tests assert the observable outputs went to STOP rather
 * than merely that some state changed. They also assert the ORIGINAL default command instance is
 * back in ownership afterwards, so "the toggle resumed" cannot be satisfied by a state that simply
 * never moved.
 *
 * <p>One Robot per JVM (build.gradle {@code forkEvery = 1} plus the static drivetrain closed by
 * {@code RobotContainer.close()}); methods are ordered and each establishes its own
 * enable/switch preconditions explicitly.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class OperatorStateCleanupTest {
  private static final int kControlBoxPort = 1;
  private static final int kSimMirrorPort = 3;

  private static final int kFallbackShot = 1;
  private static final int kUnifiedEject = 3;
  private static final int kBounceToggle = 5;
  private static final int kIntakeToggle = 6;

  /** The fixed fallback target, in Kraken rotor RPM. Used because it reaches SHOOTING without vision. */
  private static final double kFallbackShotRPM = 1600;

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
    robotThread = SimRobotLoop.start(robot, "OperatorStateCleanupTest-competition");
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

  /** The Command currently holding Superstructure. In steady teleop this is the operator policy. */
  private static Command superstructureOwner() {
    return CommandScheduler.getInstance().requiring(superstructure);
  }

  /**
   * Brings the robot to enabled teleop with an idle board and {@code intake.homing()} finished, so
   * each test starts from the same settled precondition rather than inheriting the previous one.
   */
  private static void settleIdleTeleop() {
    releaseAllControls();
    enableTeleop();
    stepUntil(() -> CommandScheduler.getInstance().requiring(intake) == null, 4.0,
        "intake.homing() should have completed and released Intake");
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.STOWED, 3.0,
        "an idle board in teleop settles to STOWED");
  }

  /**
   * Schedules a Superstructure-requiring Command that issues NO request of its own, and asserts it
   * actually took ownership. The caller must cancel it.
   */
  private static Command scheduleRequirementOnlyInterrupter() {
    Command policyBefore = superstructureOwner();
    assertNotNull(policyBefore, "precondition: the operator policy owns Superstructure");

    Command interrupter = Commands.run(() -> {
    }, superstructure).withName("RequirementOnlyInterrupter");
    CommandScheduler.getInstance().schedule(interrupter);
    step(0.04);
    assertSame(interrupter, superstructureOwner(),
        "precondition: the interrupter has taken Superstructure from the operator policy");
    return interrupter;
  }

  /** Cancels the interrupter and asserts the ORIGINAL default command instance is back in charge. */
  private static void endInterrupterAndAssertDefaultRestored(Command interrupter,
      Command expectedDefault) {
    CommandScheduler.getInstance().cancel(interrupter);
    stepUntil(() -> superstructureOwner() != null && superstructureOwner() != interrupter, 2.0,
        "the scheduler must re-schedule Superstructure's default command once the interrupter "
            + "releases the requirement");
    assertSame(expectedDefault, superstructureOwner(),
        "the SAME operator-policy default command instance must be back in ownership -- an "
            + "assertion that only watched the state could be satisfied by a state that never "
            + "moved, which is exactly the bug this file exists for");
  }

  /* -------------------------------------------------------------------- tests */

  /**
   * Required behavior 1: EJECTING -> disable -> button released while disabled -> the stored
   * operator intent is already neutral BEFORE re-enable, and the first enabled cycle cannot
   * re-issue EJECT.
   */
  @Test
  @Order(1)
  @Timeout(90)
  void ejectingIsNeutralizedWhileDisabledSoReEnableCannotReissueEject() {
    settleIdleTeleop();

    setControl(kUnifiedEject, true);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.EJECTING, 2.0,
        "precondition: button 3 held drives EJECTING");
    assertEquals(Shooter.indexing.EJECT, shooter.getIndexerState(),
        "precondition: EJECTING is actually running the indexer backwards");

    // Disable with the button STILL HELD -- the operator's hand is on it when the field disables
    // the robot, which is precisely how the stale request used to get latched.
    disableRobot();
    setControl(kUnifiedEject, false); // released while disabled: no enabled tick ever sees the edge
    step(0.2); // advance disabled scheduler loops

    assertEquals(SuperstructureState.STOWED, superstructure.getSystemState(),
        "the stored operator intent must already be neutral before re-enable -- the policy's "
            + "execute() is inert while disabled, so nothing else would ever revise it");
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
        "indexer intent must be exactly STOP while disabled, not merely 'not INDEX'");
    assertEquals(Shooter.Agitate.STOP, shooter.getAgitateState(),
        "agitator intent must be exactly STOP while disabled");
    assertEquals(Intake.Roller.STOP, intake.getRollerState(),
        "the eject roller request must not survive the disable either");

    // Re-enable and inspect the FIRST enabled scheduler cycle. Superstructure.periodic() runs
    // before a newly-available default command gets to execute, so "the default will fix it on
    // its next tick" would not be good enough here.
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(0.02);

    assertNotEquals(SuperstructureState.EJECTING, superstructure.getSystemState(),
        "the first enabled scheduler cycle must not apply a stale EJECT");
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
        "no stale EJECT may reach the indexer on the first enabled cycle");
    assertEquals(Intake.Roller.STOP, intake.getRollerState(),
        "no stale EJECT may reach the intake roller on the first enabled cycle");
  }

  /**
   * Required behaviors 2 and 9: SHOOTING -> disable -> button released while disabled -> the feed
   * is neutralized before re-enable, and the FIRST enabled scheduler cycle cannot emit a stale
   * teleop feed-path state.
   */
  @Test
  @Order(2)
  @Timeout(90)
  void shootingFeedIsNeutralizedWhileDisabledSoTheFirstEnabledCycleCannotFeed() {
    settleIdleTeleop();

    setControl(kFallbackShot, true);
    stepUntil(() -> shooter.getIndexerState() == Shooter.indexing.INDEX, 8.0,
        "precondition: the fallback shot spun up and is feeding");
    assertEquals(SuperstructureState.SHOOTING, superstructure.getSystemState(),
        "precondition: feeding happens in SHOOTING");
    assertEquals(Shooter.Agitate.IN, shooter.getAgitateState(),
        "precondition: the shot feed also runs the agitator inward");

    disableRobot();
    setControl(kFallbackShot, false); // released while disabled
    step(0.2); // advance disabled scheduler loops

    assertEquals(SuperstructureState.STOWED, superstructure.getSystemState(),
        "a shot taken into a disable must not leave SHOOTING latched as the stored intent");
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
        "feed intent must be exactly STOP before re-enable");
    assertEquals(Shooter.Agitate.STOP, shooter.getAgitateState(),
        "agitator intent must be exactly STOP before re-enable");

    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(0.02);

    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
        "the first enabled scheduler cycle must not emit a stale teleop feed");
    assertEquals(Shooter.Agitate.STOP, shooter.getAgitateState(),
        "the first enabled scheduler cycle must not emit a stale teleop agitator command");
    assertNotEquals(SuperstructureState.SHOOTING, superstructure.getSystemState(),
        "the first enabled scheduler cycle must not re-enter the feed path");
    assertNotEquals(SuperstructureState.ALIGNING, superstructure.getSystemState(),
        "nor its spin-up phase -- no shot may be resumed without the operator asking again");
  }

  /**
   * Required behaviors 3 and 10: a maintained toggle's INTAKING request must be dropped for the
   * WHOLE of a requirement-only interruption, then resume from the still-ON physical switch once
   * the interrupter releases Superstructure.
   */
  @Test
  @Order(3)
  @Timeout(90)
  void intakingIsDroppedThroughARequirementOnlyInterruptionThenResumesFromTheLiveToggle() {
    settleIdleTeleop();

    setControl(kIntakeToggle, true);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.INTAKING, 3.0,
        "precondition: toggle 6 drives INTAKING");
    stepUntil(() -> intake.getRollerState() == Intake.Roller.INTAKE, 2.0,
        "precondition: INTAKING is actually running the roller");
    Command policy = superstructureOwner();

    Command interrupter = scheduleRequirementOnlyInterrupter();

    // The interrupter issues no request. Without end-of-command cleanup the previous INTAKING
    // request would simply keep being applied by periodic() for the interrupter's whole duration.
    stepUntil(() -> intake.getRollerState() == Intake.Roller.STOP, 1.0,
        "the operator's INTAKING request must be relinquished when the default command is "
            + "interrupted, not carried through the interrupter");
    assertSame(interrupter, superstructureOwner(),
        "the roller must have stopped WHILE the interrupter still owns Superstructure -- an "
            + "assertion made after it released would prove nothing");
    assertEquals(SuperstructureState.STOWED, superstructure.getSystemState(),
        "the relinquished intent lands on STOWED, the neutral posture");

    // Hold the interruption for a while and confirm the old request does not creep back.
    step(0.5);
    assertEquals(Intake.Roller.STOP, intake.getRollerState(),
        "the stale INTAKING request must stay gone for the whole interruption");
    assertSame(interrupter, superstructureOwner(), "precondition: still interrupted");

    // Toggle 6 never moved. The physical switch is still ON.
    endInterrupterAndAssertDefaultRestored(interrupter, policy);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.INTAKING, 3.0,
        "a still-ON toggle 6 must reassert automatically once ownership frees -- the cleanup must "
            + "not have cost the maintained switch its resume behavior");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "the resumed intent must actually drive the roller again");

    setControl(kIntakeToggle, false);
  }

  /** Required behavior 4: the same drop-then-resume contract for the BOUNCING maintained toggle. */
  @Test
  @Order(4)
  @Timeout(90)
  void bouncingIsDroppedThroughARequirementOnlyInterruptionThenResumesFromTheLiveToggle() {
    settleIdleTeleop();

    setControl(kBounceToggle, true);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.BOUNCING, 4.0,
        "precondition: toggle 5 drives BOUNCING");
    stepUntil(() -> intake.getRollerState() == Intake.Roller.INTAKE, 2.0,
        "precondition: BOUNCING keeps the roller running");
    Command policy = superstructureOwner();

    Command interrupter = scheduleRequirementOnlyInterrupter();

    stepUntil(() -> intake.getRollerState() == Intake.Roller.STOP, 1.0,
        "the operator's BOUNCING request must be relinquished on interruption");
    assertSame(interrupter, superstructureOwner(),
        "the roller must have stopped while the interrupter still owns Superstructure");
    assertEquals(SuperstructureState.STOWED, superstructure.getSystemState(),
        "BOUNCING must not survive the interruption");
    assertEquals(Shooter.Agitate.STOP, shooter.getAgitateState(),
        "BOUNCING's outward agitator command must not survive either");

    endInterrupterAndAssertDefaultRestored(interrupter, policy);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.BOUNCING, 4.0,
        "a still-ON toggle 5 must reassert automatically once ownership frees");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "the resumed bounce must drive the roller again");

    setControl(kBounceToggle, false);
  }

  /**
   * Required behavior 5: EJECTING is the worst stale request to leak, because it drives the
   * indexer and roller BACKWARDS. Assert the exact STOP outputs during the interruption.
   */
  @Test
  @Order(5)
  @Timeout(90)
  void ejectingOutputsGoToStopForTheWholeRequirementOnlyInterruption() {
    settleIdleTeleop();

    setControl(kUnifiedEject, true);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.EJECTING, 2.0,
        "precondition: button 3 held drives EJECTING");
    assertEquals(Shooter.indexing.EJECT, shooter.getIndexerState(),
        "precondition: the indexer is genuinely running backwards");
    assertEquals(Intake.Roller.EJECT, intake.getRollerState(),
        "precondition: the roller is genuinely running backwards");
    Command policy = superstructureOwner();

    Command interrupter = scheduleRequirementOnlyInterrupter();

    stepUntil(() -> shooter.getIndexerState() == Shooter.indexing.STOP, 1.0,
        "the indexer must be driven to exactly STOP during the interruption");
    assertSame(interrupter, superstructureOwner(),
        "the indexer must have stopped while the interrupter still owns Superstructure");
    assertEquals(Shooter.Agitate.STOP, shooter.getAgitateState(),
        "the agitator must be driven to exactly STOP during the interruption");
    assertEquals(Intake.Roller.STOP, intake.getRollerState(),
        "the reversed roller must be driven to exactly STOP during the interruption");

    // Button 3 is still held. The momentary must still resume once ownership frees -- cleanup is a
    // relinquish of the STORED request, not a suppression of the live control.
    endInterrupterAndAssertDefaultRestored(interrupter, policy);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.EJECTING, 2.0,
        "a still-held button 3 must reassert once the interrupter releases Superstructure");

    setControl(kUnifiedEject, false);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.STOWED, 2.0,
        "releasing button 3 returns to STOWED");
  }

  /** Required behavior 6: the shot feed goes to exactly STOP during a requirement-only interruption. */
  @Test
  @Order(6)
  @Timeout(90)
  void shootingFeedGoesToStopForTheWholeRequirementOnlyInterruption() {
    settleIdleTeleop();

    setControl(kFallbackShot, true);
    stepUntil(() -> shooter.getIndexerState() == Shooter.indexing.INDEX, 8.0,
        "precondition: the fallback shot spun up and is feeding");
    assertEquals(SuperstructureState.SHOOTING, superstructure.getSystemState(),
        "precondition: feeding happens in SHOOTING");
    assertEquals(kFallbackShotRPM, shooter.getTargetRPM(), 1e-9,
        "precondition: the fixed fallback target is what is commanded");
    Command policy = superstructureOwner();

    Command interrupter = scheduleRequirementOnlyInterrupter();

    stepUntil(() -> shooter.getIndexerState() == Shooter.indexing.STOP, 1.0,
        "the feed must be driven to exactly STOP during the interruption -- an interrupter that "
            + "issues no request of its own must not inherit an in-progress shot");
    assertSame(interrupter, superstructureOwner(),
        "the feed must have stopped while the interrupter still owns Superstructure");
    assertEquals(Shooter.Agitate.STOP, shooter.getAgitateState(),
        "the agitator must be driven to exactly STOP during the interruption");
    assertNotEquals(SuperstructureState.SHOOTING, superstructure.getSystemState(),
        "the shot must not still be in its feeding state during the interruption");
    assertNotEquals(SuperstructureState.ALIGNING, superstructure.getSystemState(),
        "nor parked in spin-up waiting to feed the moment the interrupter ends");

    endInterrupterAndAssertDefaultRestored(interrupter, policy);

    setControl(kFallbackShot, false);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.STOWED, 3.0,
        "releasing button 1 returns to STOWED");
  }
}
