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
import frc.robot.Robot;
import frc.robot.SimRobotLoop;
import frc.robot.subsystems.intake.Intake.HomingState;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * The hardened hardstop detector and the failed-home position guard.
 *
 * <p><b>What changed and why these tests exist.</b> Homing used to end the seek on the FIRST sample
 * where pivot stator current exceeded 65 A. On real hardware that sample is not distinctive: the
 * pivot is stationary at seek start, so breaking the arm away draws stall-level current while the
 * arm is not yet moving -- electrically identical to sitting against the bumper hardstop. The
 * detector now requires the current AND near-stall motion together, held continuously for
 * {@code kHardstopQualifiedSeconds}, after a {@code kHomingStartupGraceSeconds} dead window. And a
 * seek that never qualifies no longer merely declines to zero: {@code commandPivotPosition()}
 * refuses every closed-loop pivot request while no home reference exists, so a failed home cannot
 * be followed by a MotionMagic DOWN aimed at the meaningless boot zero.
 *
 * <p><b>Why the sensor overrides.</b> {@code IntakeIOSim}'s arm is near-massless: measured over a
 * real seek it draws 3-6 A while free-running at ~8.7 rad/s and only reaches 91.5 A once pinned at
 * the travel limit with velocity exactly 0. It therefore cannot produce a startup acceleration
 * spike at all, which is precisely the case that most needs covering. The overrides let each half
 * of the hardstop signature be presented independently -- same seam style, and same package-private
 * visibility, as the pre-existing {@code forceJamConditionForTest}.
 *
 * <p>Boots one {@code Robot} in {@code @BeforeAll} like {@code IntakeHomingCleanupTest}: the
 * subsystems are singletons, so a per-method boot would buy nothing.
 */
class IntakeHomingQualificationTest {
  private static final double kTick = 0.02;
  /** Matches Intake.kHomingVoltage; re-declared because that constant is private. */
  private static final double kHomingVoltage = 3.0;
  /** Comfortably longer than the 1.0 s seek timeout plus its stage-transition ticks. */
  private static final double kSeekOutcomeTimeout = 4.0;

  /** Well above the 65 A threshold -- "the motor is pushing hard on something". */
  private static final double kPushingAmps = 95.0;
  /** Well below it -- "the motor is barely loaded". */
  private static final double kUnloadedAmps = 5.0;
  /** Below the 30 deg/s near-stall threshold. */
  private static final double kStalledRadPerSec = 0.0;
  /** Roughly free speed (~8.7 rad/s = ~500 deg/s) -- unambiguously still accelerating/moving. */
  private static final double kMovingRadPerSec = 8.0;

  private static Robot robot;
  private static Thread robotThread;
  private static Intake intake;

  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    robot = new Robot();
    robotThread = SimRobotLoop.start(robot, "IntakeHomingQualificationTest-competition");
    SimRobotLoop.awaitProgramStart();

    // AUTONOMOUS for the same two reasons IntakeHomingCleanupTest documents: Superstructure boots
    // in OFF, the one state that never re-commands the pivot (so nothing masks what homing leaves
    // behind), and entering teleop would fire RobotModeTriggers.teleop().onTrue(intake.homing())
    // and schedule a homing command these tests did not ask for.
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

  @BeforeEach
  void reset() {
    assertTrue(robotThread.isAlive(), "startCompetition() loop should still be running");
    CommandScheduler.getInstance().cancelAll();
    clearOverrides();
    SimRobotLoop.step(kTick);
  }

  @AfterEach
  void cleanUp() {
    CommandScheduler.getInstance().cancelAll();
    clearOverrides();
    SimRobotLoop.step(kTick);
  }

  /* ------------------------------------------------------------------------ 1. the detector -- */

  /**
   * The headline case. A stationary arm being broken away draws stall-level current while its
   * velocity is still ~0 -- the exact electrical signature of the hardstop. The startup grace
   * window is what separates them, so a spike confined to the start of the seek must not home even
   * though, sample for sample, it looks like a stop.
   */
  @Test
  @Timeout(60)
  void startupAccelerationSpikeAloneDoesNotHome() {
    clearHome();

    // Full hardstop signature -- but only during the grace window.
    intake.forcePivotStatorCurrentForTest(kPushingAmps);
    intake.forcePivotVelocityForTest(kStalledRadPerSec);

    Command homing = intake.homing();
    CommandScheduler.getInstance().schedule(homing);

    // 0.10 s of spike: inside the 0.15 s grace, and far longer than the 0.06 s persistence the
    // detector would otherwise need, so this fails ONLY because of the grace window.
    for (int i = 0; i < 5; i++) {
      SimRobotLoop.step(kTick);
    }
    assertFalse(intake.homedForTest(),
        "a current spike inside the startup grace window must not declare a home");

    // The arm then breaks away and runs free for the rest of the seek: current drops, so nothing
    // ever qualifies and the seek must run out its timeout.
    intake.forcePivotStatorCurrentForTest(kUnloadedAmps);
    intake.forcePivotVelocityForTest(kMovingRadPerSec);

    SimRobotLoop.stepUntil(() -> !CommandScheduler.getInstance().isScheduled(homing),
        kSeekOutcomeTimeout, "the seek to time out");

    assertFalse(intake.homedForTest(), "the seek never found a real stop, so it must not be homed");
    assertEquals(HomingState.FAILED_TIMEOUT, intake.getHomingState());
  }

  /**
   * The other half of the same discrimination: high current for the WHOLE seek, but with the arm
   * still moving. Current alone must never be enough, however long it lasts.
   */
  @Test
  @Timeout(60)
  void sustainedHighCurrentWhileStillMovingDoesNotHome() {
    clearHome();

    intake.forcePivotStatorCurrentForTest(kPushingAmps);
    intake.forcePivotVelocityForTest(kMovingRadPerSec);

    Command homing = intake.homing();
    CommandScheduler.getInstance().schedule(homing);
    SimRobotLoop.stepUntil(() -> !CommandScheduler.getInstance().isScheduled(homing),
        kSeekOutcomeTimeout, "the seek to time out");

    assertFalse(intake.homedForTest(),
        "current above threshold with the mechanism still moving is an accelerating arm, not a stop");
    assertEquals(HomingState.FAILED_TIMEOUT, intake.getHomingState());
  }

  /** Both halves of the signature, held past the grace window and the persistence window: a home. */
  @Test
  @Timeout(60)
  void sustainedCurrentWithLowVelocityHomes() {
    clearHome();

    intake.forcePivotStatorCurrentForTest(kPushingAmps);
    intake.forcePivotVelocityForTest(kStalledRadPerSec);

    Command homing = intake.homing();
    CommandScheduler.getInstance().schedule(homing);
    SimRobotLoop.stepUntil(() -> !CommandScheduler.getInstance().isScheduled(homing),
        kSeekOutcomeTimeout, "the seek to qualify a hardstop");

    assertTrue(intake.homedForTest(),
        "a sustained stall against a load, past the grace window, is exactly what a hardstop is");
    assertEquals(HomingState.HOMED, intake.getHomingState());
  }

  /** A successful home establishes the reference exactly once, never repeatedly. */
  @Test
  @Timeout(60)
  void successfulHomeZeroesTheReferenceExactlyOnce() {
    clearHome();
    long zeroesBefore = intake.pivotZeroCountForTest();

    homeSuccessfully();

    assertEquals(zeroesBefore + 1, intake.pivotZeroCountForTest(),
        "one successful seek must zero the encoder reference exactly once");

    // ... and holding the stop for a further second must not re-zero, since the seek has ended.
    for (int i = 0; i < 50; i++) {
      SimRobotLoop.step(kTick);
    }
    assertEquals(zeroesBefore + 1, intake.pivotZeroCountForTest(),
        "nothing outside a seek may re-zero the reference");
  }

  /* ------------------------------------------------------------ 2. the failed-home guard ----- */

  /**
   * The central safety property. After a failed seek the pivot must be unhomed AND electrically
   * neutral, and the DOWN request that the homing sequence itself issues as its last stage must be
   * refused rather than driving MotionMagic against the boot zero.
   */
  @Test
  @Timeout(60)
  void timedOutHomeStaysUnhomedNeutralAndRefusesDown() {
    clearHome();
    long blockedBefore = intake.blockedPivotPositionRequestsForTest();

    timeOutASeek();

    assertFalse(intake.homedForTest(), "a timed-out seek must leave the intake unhomed");
    assertEquals(HomingState.FAILED_TIMEOUT, intake.getHomingState());
    assertNotEquals(kHomingVoltage, intake.pivotAppliedVoltsForTest(), 1e-9,
        "timing out must not leave the seek voltage applied");
    assertEquals(0.0, intake.pivotAppliedVoltsForTest(), 1e-9,
        "a timed-out seek must leave the pivot electrically neutral");
    assertTrue(intake.blockedPivotPositionRequestsForTest() > blockedBefore,
        "the homing sequence's own final goTo(DOWN) must have been refused");
  }

  /**
   * And the refusal must hold for every later caller, tick after tick -- this is what
   * {@code Superstructure.STOWED} does on every single loop while the intake is unowned. Proved
   * from the OUTPUT, not just the counter: if any MotionMagic request had actually reached the IO,
   * {@code IntakeIOSim} would leave closed-loop mode on and start driving the arm, and the applied
   * voltage would move off zero.
   */
  @Test
  @Timeout(60)
  void unhomedPositionRequestsAreBlockedEveryTickAndDriveNothing() {
    clearHome();
    timeOutASeek();

    for (int i = 0; i < 30; i++) {
      intake.goTo(Intake.PivotState.DOWN);
      intake.goToDegrees(-95);
      SimRobotLoop.step(kTick);
      assertEquals(0.0, intake.pivotAppliedVoltsForTest(), 1e-9,
          "an unhomed pivot must stay neutral on tick " + i + " despite repeated DOWN requests");
    }
    assertFalse(intake.homedForTest(), "nothing here may establish a home reference");
  }

  /** A successful home lifts the guard: DOWN is honoured and actually drives the mechanism. */
  @Test
  @Timeout(60)
  void successfulHomeAllowsDown() {
    clearHome();
    homeSuccessfully();
    clearOverrides();

    long blockedBefore = intake.blockedPivotPositionRequestsForTest();
    intake.goTo(Intake.PivotState.DOWN);
    SimRobotLoop.step(kTick);
    SimRobotLoop.step(kTick);

    assertEquals(blockedBefore, intake.blockedPivotPositionRequestsForTest(),
        "once homed, a DOWN request must not be refused");
    assertNotEquals(0.0, intake.pivotAppliedVoltsForTest(), 1e-9,
        "the honoured MotionMagic request must actually be driving the pivot");
  }

  /**
   * A failed home must not poison the mechanism for the rest of the match: teleop re-arms homing on
   * every entry, and the retry has to be able to succeed.
   */
  @Test
  @Timeout(90)
  void aFailedHomeCanBeRetriedSuccessfully() {
    clearHome();

    timeOutASeek();
    assertFalse(intake.homedForTest(), "precondition: the first attempt failed");
    assertEquals(HomingState.FAILED_TIMEOUT, intake.getHomingState());

    homeSuccessfully();

    assertTrue(intake.homedForTest(), "a retry after a failed home must still be able to succeed");
    assertEquals(HomingState.HOMED, intake.getHomingState());
  }

  /* --------------------------------------------------------------------- 3. state reporting -- */

  /** An interrupted seek is reported as INTERRUPTED, distinctly from a timeout. */
  @Test
  @Timeout(60)
  void interruptedSeekIsReportedDistinctlyFromATimeout() {
    clearHome();

    intake.forcePivotStatorCurrentForTest(kUnloadedAmps);
    intake.forcePivotVelocityForTest(kMovingRadPerSec);

    Command homing = intake.homing();
    CommandScheduler.getInstance().schedule(homing);
    SimRobotLoop.step(kTick);
    SimRobotLoop.step(kTick);
    assertEquals(HomingState.SEEKING, intake.getHomingState(),
        "precondition: the seek must actually be running");
    assertEquals(kHomingVoltage, intake.pivotAppliedVoltsForTest(), 1e-9,
        "precondition: the seek must be driving the homing voltage");

    homing.cancel();
    SimRobotLoop.step(kTick);

    assertEquals(HomingState.INTERRUPTED, intake.getHomingState());
    assertEquals(0.0, intake.pivotAppliedVoltsForTest(), 1e-9,
        "an interrupted seek must leave the pivot neutral");
    assertFalse(intake.homedForTest(), "an interrupted seek must not assert a home");
  }

  /* -------------------------------------------------------------------------------- helpers -- */

  private static void clearOverrides() {
    intake.clearPivotCurrentOverrideForTest();
    intake.clearPivotVelocityOverrideForTest();
  }

  /**
   * Returns the singleton to its power-up posture. {@link Intake} has no reset hook and other tests
   * in this JVM home it, so without this the never-homed guard would be unreachable.
   */
  private static void clearHome() {
    intake.clearHomeReferenceForTest();
    assertFalse(intake.homedForTest(), "precondition: no home reference");
  }

  /** Runs a seek that cannot qualify, to completion. */
  private static void timeOutASeek() {
    intake.forcePivotStatorCurrentForTest(kUnloadedAmps);
    intake.forcePivotVelocityForTest(kMovingRadPerSec);
    Command homing = intake.homing();
    CommandScheduler.getInstance().schedule(homing);
    SimRobotLoop.stepUntil(() -> !CommandScheduler.getInstance().isScheduled(homing),
        kSeekOutcomeTimeout, "the seek to time out");
    clearOverrides();
    SimRobotLoop.step(kTick);
  }

  /** Runs a seek that presents a clean, sustained hardstop, to completion. */
  private static void homeSuccessfully() {
    intake.forcePivotStatorCurrentForTest(kPushingAmps);
    intake.forcePivotVelocityForTest(kStalledRadPerSec);
    Command homing = intake.homing();
    CommandScheduler.getInstance().schedule(homing);
    SimRobotLoop.stepUntil(() -> !CommandScheduler.getInstance().isScheduled(homing),
        kSeekOutcomeTimeout, "the seek to qualify a hardstop");
    assertTrue(intake.homedForTest(), "the helper must actually have homed");
  }
}
