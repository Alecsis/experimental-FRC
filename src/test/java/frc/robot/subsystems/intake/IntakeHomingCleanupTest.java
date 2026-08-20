// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.SimRobotLoop;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Covers {@link Intake#homing()}'s exit paths, especially the interruption path that previously had
 * no cleanup.
 *
 * <p><b>The defect.</b> {@code homing()} is a three-stage sequence: a {@code Commands.run} seek that
 * drives the pivot at {@code kHomingVoltage} until {@code hardstop()} (bounded by a 1 s timeout),
 * then a stage that zeroes the output and the encoder, then {@code goTo(DOWN)}. Cleanup lived
 * exclusively in stage two. Interrupting the sequence while stage one was running skipped it, and
 * the last {@code setPivotVoltage(kHomingVoltage)} stayed LATCHED -- CTRE control requests persist
 * until superseded, and {@code IntakeIOSim} models that faithfully ({@code setPivotVoltage} leaves
 * {@code pivotClosedLoop} false, so {@code updateInputs} keeps re-applying the stored volts).
 *
 * <p><b>Why it mattered.</b> "Home Intake" is the first command in all 15 autos, so every cancel in
 * the first second of autonomous -- {@code teleopInit()}, a disable, a test cancel -- landed inside
 * that window. Nothing downstream rescued it: {@code Superstructure} boots in {@code OFF}, and
 * {@code OFF} is the one state whose {@code periodic()} never calls {@code setIntakePivot()}.
 *
 * <p>Package {@code frc.robot.subsystems.intake} for package-private access to
 * {@link Intake#pivotAppliedVoltsForTest()}, same seam style as
 * {@code IntakeJamRecoveryTest}'s use of {@code forceJamConditionForTest}. Boots once in
 * {@code @BeforeAll} like {@code OperatorBoxArbitrationTest}: {@code Intake} is a singleton, so a
 * per-method boot would buy nothing, and every method here re-homes from a settled state.
 */
class IntakeHomingCleanupTest {
  /** Matches Intake.kHomingVoltage; re-declared because that constant is private. */
  private static final double kHomingVoltage = 3.0;

  private static final double kTick = 0.02;

  private static Robot robot;
  private static Thread robotThread;
  private static Intake intake;

  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    robot = new Robot();
    robotThread = SimRobotLoop.start(robot, "IntakeHomingCleanupTest-competition");
    SimRobotLoop.awaitProgramStart();

    // AUTONOMOUS, deliberately, for two reasons.
    //
    // 1. It is the environment the defect actually lives in. Superstructure boots in OFF and OFF is
    //    the only state whose periodic() never calls setIntakePivot(), so nothing re-commands the
    //    pivot after an interrupted home -- which is exactly why a latched voltage would persist.
    //    In TELEOP the operator-policy default command settles the machine into STOWED, whose
    //    periodic() issues goTo(DOWN) on every tick the intake is unowned; that masks the latch by
    //    superseding it one tick later, and would have made these tests pass for the wrong reason.
    // 2. Entering teleop at all would fire RobotModeTriggers.teleop().onTrue(intake.homing()) and
    //    schedule a homing command this class did not ask for.
    //
    // Enabled is required either way: IntakeIOSim zeroes every applied-voltage output while the
    // DriverStation reports disabled, exactly as a real motor controller does, so nothing about a
    // latched request is observable until the robot is enabled.
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();

    intake = Intake.getInstance();
    SimRobotLoop.step(kTick);
  }

  @AfterAll
  static void teardown() {
    SimRobotLoop.shutdown();
  }

  /**
   * Every method must start with the pivot AWAY from the hardstop, or its homing command finds the
   * stop on its first tick and finishes before the test can interrupt anything -- which silently
   * turns an interruption test into a no-op. The timeout test in particular leaves the arm pinned
   * against the stop (it drives the full 1 s seek with hardstop detection suppressed), and JUnit
   * does not guarantee method order, so this cannot be left to luck.
   *
   * <p>Verified by a bite-check: with the production
   * finalizer disabled, the interruption tests fail. Without this reset, one of them passed anyway.
   */
  @BeforeEach
  void settle() {
    assertTrue(robotThread.isAlive(), "startCompetition() loop should still be running");
    CommandScheduler.getInstance().cancelAll();
    intake.clearPivotCurrentOverrideForTest();
    SimRobotLoop.step(kTick);

    intake.goTo(Intake.PivotState.DOWN);
    SimRobotLoop.stepUntil(() -> !intake.hardstop(), 2.0, "the pivot to come off the hardstop");
    for (int i = 0; i < 15; i++) {
      SimRobotLoop.step(kTick);
    }
    assertFalse(intake.hardstop(),
        "each test must start with the pivot clear of the hardstop, or its seek stage is skipped");
  }

  /**
   * Schedules a command and advances until the seek stage's voltage request is OBSERVABLE.
   *
   * <p>Two ticks, not one. CommandScheduler.run() calls every subsystem's periodic() BEFORE it
   * executes scheduled commands, so Intake.periodic()'s io.updateInputs(inputs) snapshot always
   * lags this tick's setPivotVoltage() by one loop. One step would read the previous tick's value.
   */
  private static void scheduleAndEnterSeek(Command command) {
    CommandScheduler.getInstance().schedule(command);
    SimRobotLoop.step(kTick);
    SimRobotLoop.step(kTick);
  }

  // ---- 1. Normal successful homing ----

  @Test
  @Timeout(30)
  void successfulHomingSeeksZeroesAndLeavesThePivotHeldAtDown() {
    Command homing = intake.homing();

    // Stage one drives toward the hardstop.
    scheduleAndEnterSeek(homing);
    assertEquals(kHomingVoltage, intake.pivotAppliedVoltsForTest(), 1e-9,
        "the seek stage must actually command the homing voltage");

    // IntakeIOSim's arm reaches the hardstop current within a few ticks (measured 91.5 A at
    // t=0.123s in logs/akit_26-08-20_14-52-37.wpilog, against a 65 A threshold), so this finishes
    // on the hardstop branch rather than the timeout.
    SimRobotLoop.stepUntil(() -> !CommandScheduler.getInstance().isScheduled(homing), 3.0,
        "homing to finish");

    assertFalse(CommandScheduler.getInstance().isScheduled(homing));
    assertTrue(intake.homedForTest(), "a detected hardstop must mark the intake homed");
    // Stage three hands the pivot to setPivotPosition(DOWN), i.e. closed loop -- NOT 0 V. Asserting
    // "not the homing voltage" rather than a specific number keeps this about the handover.
    assertNotEquals(kHomingVoltage, intake.pivotAppliedVoltsForTest(), 1e-9,
        "the homing voltage must not survive normal completion");
  }

  // ---- 2. Timeout path ----

  @Test
  @Timeout(30)
  void timingOutZeroesTheOutputAndDoesNotFalselyMarkHomed() {
    // IntakeIOSim's arm always reaches the hardstop current within a few ticks, so the timeout
    // branch is only reachable by holding the sensed current below the threshold.
    intake.forcePivotStatorCurrentForTest(0.0);
    try {
      boolean homedBefore = intake.homedForTest();
      assertFalse(intake.hardstop(), "precondition: the override must suppress hardstop detection");

      Command homing = intake.homing();
      scheduleAndEnterSeek(homing);
      assertEquals(kHomingVoltage, intake.pivotAppliedVoltsForTest(), 1e-9,
          "precondition: the seek stage is driving");

      SimRobotLoop.stepUntil(() -> !CommandScheduler.getInstance().isScheduled(homing), 3.0,
          "homing to time out and finish");

      assertNotEquals(kHomingVoltage, intake.pivotAppliedVoltsForTest(), 1e-9,
          "timing out must not leave the homing voltage applied");
      assertEquals(homedBefore, intake.homedForTest(),
          "a seek that never found the hardstop must not assert a home -- re-zeroing the encoder"
              + " at an arbitrary angle would misdirect every later setPivotPosition()");
    } finally {
      intake.clearPivotCurrentOverrideForTest();
      CommandScheduler.getInstance().cancelAll();
      SimRobotLoop.step(kTick);
    }
  }

  // ---- 3. Interruption during voltage-seeking ----

  @Test
  @Timeout(30)
  void interruptionDuringTheSeekNeutralisesThePivotAndDoesNotMarkHomed() {
    boolean homedBefore = intake.homedForTest();

    Command homing = intake.homing();
    scheduleAndEnterSeek(homing);
    assertEquals(kHomingVoltage, intake.pivotAppliedVoltsForTest(), 1e-9,
        "precondition: the seek stage is live and the homing voltage is latched");

    homing.cancel();
    SimRobotLoop.step(kTick);

    assertFalse(CommandScheduler.getInstance().isScheduled(homing));
    assertEquals(0.0, intake.pivotAppliedVoltsForTest(), 1e-9,
        "interrupting the seek must leave the pivot output neutral, not latched at "
            + kHomingVoltage + " V");
    assertEquals(homedBefore, intake.homedForTest(),
        "an interrupted home must not change the homed assertion");
  }

  @Test
  @Timeout(30)
  void interruptedHomingLeavesNoLatchedRequestAcrossManyIdleTicks() {
    // The latch is the hazard, not the single tick: a stale VoltageOut keeps driving forever. If
    // cleanup only appeared to work for one tick, this would catch it.
    Command homing = intake.homing();
    scheduleAndEnterSeek(homing);
    assertTrue(CommandScheduler.getInstance().isScheduled(homing),
        "precondition: homing must still be running, otherwise this cancels nothing");
    assertEquals(kHomingVoltage, intake.pivotAppliedVoltsForTest(), 1e-9,
        "precondition: the seek stage is live and the homing voltage is latched");

    homing.cancel();

    for (int i = 0; i < 25; i++) {
      SimRobotLoop.step(kTick);
      assertEquals(0.0, intake.pivotAppliedVoltsForTest(), 1e-9,
          "pivot output must stay neutral on idle tick " + i + " after an interrupted home");
    }
  }

  // ---- 4. Repeated schedule / cancel / retry ----

  @Test
  @Timeout(60)
  void repeatedScheduleCancelRetryStaysSafeAndStillHomesOnTheFinalAttempt() {
    for (int attempt = 0; attempt < 3; attempt++) {
      Command aborted = intake.homing();
      scheduleAndEnterSeek(aborted);
      aborted.cancel();
      SimRobotLoop.step(kTick);
      assertEquals(0.0, intake.pivotAppliedVoltsForTest(), 1e-9,
          "pivot must be neutral after aborted attempt " + attempt);
    }

    Command completed = intake.homing();
    CommandScheduler.getInstance().schedule(completed);
    SimRobotLoop.stepUntil(() -> !CommandScheduler.getInstance().isScheduled(completed), 3.0,
        "the retried homing to finish");

    assertTrue(intake.homedForTest(),
        "a clean retry after repeated aborts must still home successfully");
  }

  // ---- 5. Autonomous cancellation during Home Intake ----

  @Test
  @Timeout(30)
  void autonomousCancellationDuringHomeIntakeLeavesThePivotNeutral() {
    // "Home Intake" runs inside a composed autonomous command, so a real cancel arrives as an
    // interruption of the enclosing group rather than a direct cancel() on homing() itself. That
    // is a different code path -- SequentialCommandGroup.end(true) -- and it must clean up too.
    try {
      Command auto = Commands.sequence(intake.homing(), Commands.idle());
      scheduleAndEnterSeek(auto);
      assertEquals(kHomingVoltage, intake.pivotAppliedVoltsForTest(), 1e-9,
          "precondition: the composed auto is in its Home Intake seek stage");

      auto.cancel(); // what teleopInit()'s m_autonomousCommand.cancel() does
      SimRobotLoop.step(kTick);

      assertEquals(0.0, intake.pivotAppliedVoltsForTest(), 1e-9,
          "cancelling the enclosing autonomous command must neutralise the pivot too");
    } finally {
      CommandScheduler.getInstance().cancelAll();
      SimRobotLoop.step(kTick);
    }
  }

  // ---- Negative control ----

  /**
   * Negative control for the interruption tests above. Without it they could pass for a reason
   * unrelated to the fix -- anything that happens to leave the pivot at 0 V would satisfy them.
   *
   * <p>This builds BOTH command shapes over a recording {@link IntakeIO} and interrupts each in its
   * seek stage: the pre-fix shape (cleanup only in the middle stage) leaves the homing voltage
   * latched, the post-fix shape (whole-sequence finalizer) does not. No HAL, no singleton, no
   * physics -- it isolates the composition semantics that are the actual root cause.
   */
  @Test
  void negativeControlOldCompositionLeavesTheHomingVoltageLatchedAfterInterruption() {
    RecordingPivotIo oldIo = new RecordingPivotIo();
    Command oldShape = Commands.sequence(
        Commands.run(() -> oldIo.setPivotVoltage(kHomingVoltage)).until(() -> false).withTimeout(1),
        Commands.runOnce(() -> oldIo.setPivotVoltage(0.0)),
        Commands.runOnce(() -> oldIo.setPivotPosition(0.0)));

    interruptDuringSeek(oldShape);

    assertEquals(kHomingVoltage, oldIo.lastPivotVolts, 1e-9,
        "negative control is vacuous unless the OLD shape really does latch the homing voltage");

    RecordingPivotIo fixedIo = new RecordingPivotIo();
    Command fixedShape = Commands.sequence(
        Commands.run(() -> fixedIo.setPivotVoltage(kHomingVoltage)).until(() -> false)
            .withTimeout(1),
        Commands.runOnce(() -> fixedIo.setPivotVoltage(0.0)),
        Commands.runOnce(() -> fixedIo.setPivotPosition(0.0)))
        .finallyDo(interrupted -> {
          if (interrupted) {
            fixedIo.setPivotVoltage(0.0);
          }
        });

    interruptDuringSeek(fixedShape);

    assertEquals(0.0, fixedIo.lastPivotVolts, 1e-9,
        "the whole-sequence finalizer is what removes the latch");
  }

  /** Drives a command through initialize/execute (still in stage one), then interrupts it. */
  private static void interruptDuringSeek(Command command) {
    command.initialize();
    command.execute();
    command.end(true);
  }

  /** Records the last pivot request without needing HAL, physics, or the singleton. */
  private static final class RecordingPivotIo implements IntakeIO {
    private double lastPivotVolts = Double.NaN;

    @Override
    public void setPivotVoltage(double volts) {
      lastPivotVolts = volts;
    }

    @Override
    public void setPivotPosition(double positionRads) {
      // A closed-loop request supersedes any latched voltage; recorded as "not a voltage request"
      // by leaving lastPivotVolts alone, which is what makes the old-shape assertion meaningful.
    }
  }
}
