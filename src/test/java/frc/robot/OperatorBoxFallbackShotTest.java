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
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.Timeout;

/**
 * Button 1, the fixed-RPM fallback shot -- the capability the refactor had dropped, restored
 * through the Superstructure rather than by re-adding the legacy direct Shooter calls.
 *
 * <p>1600 is a Kraken <b>rotor</b> RPM, matching the shooter's existing control convention (there
 * is no SensorToMechanismRatio on the shooter and the empirical LUT is calibrated against rotor
 * RPM). Nothing here touches the LUT or the vision RPM path: the fallback simply bypasses them.
 *
 * <p>Split from {@code OperatorBoxArbitrationTest} because ALIGNING/SHOOTING are sticky states that
 * hold the machine until the shot is released; keeping them in their own Robot boot means neither
 * file's timing depends on the other's. See that class's javadoc for why one Robot is booted per
 * class rather than per method.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class OperatorBoxFallbackShotTest {
  private static final int kControlBoxPort = 1;

  private static final int kFallbackShot = 1;
  private static final int kVisionShot = 2;
  private static final int kUnifiedEject = 3;
  private static final int kBounceToggle = 5;
  private static final int kIntakeToggle = 6;

  /** The legacy fixed-RPM fallback target, in Kraken rotor RPM. */
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
    DriverStationSim.setJoystickButtonCount(3, 12);
    DriverStationSim.notifyNewData();

    robot = new Robot();
    robotThread = SimRobotLoop.start(robot, "OperatorBoxFallbackShotTest-competition");
    SimRobotLoop.awaitProgramStart();

    superstructure = Superstructure.getInstance(null);
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();

    SimRobotLoop.registerDiagnostics(
        () -> SimRobotLoop.operatorBoardState(kControlBoxPort, 6));

    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    step(2.0); // let teleop's intake.homing() finish and release Intake
  }

  @AfterAll
  static void teardown() {
    releaseAllControls();
    SimRobotLoop.shutdown();
  }

  @BeforeEach
  void resetBoard() {
    assertTrue(robotThread.isAlive(), "startCompetition() loop should still be running");
    releaseAllControls();
    assertIdleBoard("each test must start from a settled, idle board");
  }

  /**
   * The idle posture is OFF before any operator control has ever been used in this JVM and STOWED
   * afterwards (releasing the last control ends the intent command, which stows). Both mean "the
   * board is not asking for anything", which is the only precondition these tests need.
   */
  private static void assertIdleBoard(String context) {
    SuperstructureState state = superstructure.getSystemState();
    assertTrue(state == SuperstructureState.OFF || state == SuperstructureState.STOWED,
        context + " -- expected an idle state (OFF or STOWED) but was " + state);
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
    step(0.5);
  }

  private static void stepUntil(BooleanSupplier condition, double timeoutSeconds, String what) {
    SimRobotLoop.stepUntil(condition, timeoutSeconds, what);
  }

  /* -------------------------------------------------------------------- tests */

  @Test
  @Order(1)
  @Timeout(60)
  void fallbackShotTargetsFixed1600RotorRpm() {
    setControl(kFallbackShot, true);
    step(0.2);

    // ALIGNING or SHOOTING -- the simulated flywheel can cross the tolerance band within a couple
    // of ticks, so which half of the shot path we land in is a sim-dynamics detail. What matters
    // is that button 1 entered the shot path with the fixed target commanded.
    SuperstructureState state = superstructure.getSystemState();
    assertTrue(state == SuperstructureState.ALIGNING || state == SuperstructureState.SHOOTING,
        "button 1 must enter the normal shot path, but was " + state);
    assertEquals(kFallbackShotRPM, shooter.getTargetRPM(), 1e-9,
        "button 1 must command the fixed 1600 rotor-RPM fallback target, bypassing the vision LUT");
  }

  @Test
  @Order(2)
  @Timeout(60)
  void fallbackShotDoesNotFeedUntilShooterIsAtSpeed() {
    setControl(kFallbackShot, true);
    step(0.06); // a few scheduler ticks: target is set, flywheel has not spun up yet

    assertFalse(shooter.shooterAtSpeed(kFallbackShotRPM),
        "precondition: the flywheel cannot already be at 1600 RPM this soon after the press");
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
        "the indexer must be positively STOPPED while the shooter spins up -- not merely 'not "
            + "INDEX', which a leftover EJECT would also satisfy");
    assertEquals(Shooter.Agitate.STOP, shooter.getAgitateState(),
        "the agitator must be positively STOPPED while the shooter spins up");
  }

  @Test
  @Order(3)
  @Timeout(60)
  void fallbackShotFeedsIndexerAndAgitatorOnceAtSpeed() {
    setControl(kFallbackShot, true);

    // Watch the ordering directly rather than sampling the flywheel speed at the instant the feed
    // starts: the closed-loop sim rings around the setpoint, so an instantaneous at-speed read is
    // a property of the sim's damping, not of the gate. The gate is that at-speed happened FIRST.
    boolean reachedSpeedBeforeFeeding = false;
    boolean feeding = false;
    for (double elapsed = 0; elapsed < 6.0; elapsed += 0.02) {
      if (shooter.getIndexerState() == Shooter.indexing.INDEX) {
        feeding = true;
        break;
      }
      if (shooter.shooterAtSpeed(kFallbackShotRPM)) {
        reachedSpeedBeforeFeeding = true;
      }
      step(0.02);
    }

    assertTrue(feeding, "once the shooter reaches the fixed target the indexer must feed");
    assertTrue(reachedSpeedBeforeFeeding,
        "the feed must be gated on the shooter reaching the fixed 1600 RPM target first -- this "
            + "is the legacy waitUntil(shooterAtSpeed(...)) behavior");
    assertEquals(Shooter.Agitate.IN, shooter.getAgitateState(),
        "feeding uses the normal shot semantics: indexer INDEX plus agitator IN");
    assertEquals(SuperstructureState.SHOOTING, superstructure.getSystemState(),
        "feeding happens in the SHOOTING state, reusing the existing shot path");
  }

  @Test
  @Order(4)
  @Timeout(60)
  void releasingFallbackShotStopsTheFeed() {
    setControl(kFallbackShot, true);
    stepUntil(() -> shooter.getIndexerState() == Shooter.indexing.INDEX, 6.0,
        "precondition: the fallback shot is feeding");

    setControl(kFallbackShot, false);
    step(0.3);

    assertEquals(SuperstructureState.STOWED, superstructure.getSystemState(),
        "releasing button 1 with no toggle on returns to the idle/stowed state");
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
        "releasing button 1 must stop feeding the indexer");
    assertEquals(Shooter.Agitate.STOP, shooter.getAgitateState(),
        "releasing button 1 must stop the agitator");
  }

  @Test
  @Order(5)
  @Timeout(60)
  void releasingFallbackShotDropsBackToTheCurrentIdleRpmNotTheLegacy1000() {
    setControl(kFallbackShot, true);
    stepUntil(() -> shooter.getTargetRPM() == kFallbackShotRPM, 3.0,
        "precondition: the fallback target is commanded");

    setControl(kFallbackShot, false);
    step(0.3);

    // The legacy binding hard-coded 1000 RPM on release. The current Superstructure owns idle RPM
    // itself (kIdleRPM = 700, Superstructure.java), and that policy is preserved rather than
    // reintroducing the legacy number.
    assertEquals(700.0, shooter.getTargetRPM(), 1e-9,
        "release must hand the shooter back to the Superstructure's own idle RPM policy");
  }

  @Test
  @Order(6)
  @Timeout(60)
  void releasingFallbackShotResumesIntakeToggle() {
    setControl(kIntakeToggle, true);
    step(0.3);
    assertEquals(SuperstructureState.INTAKING, superstructure.getSystemState(),
        "precondition: toggle 6 has the robot intaking");

    setControl(kFallbackShot, true);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.ALIGNING
        || superstructure.getSystemState() == SuperstructureState.SHOOTING,
        3.0, "button 1 must temporarily override the intake toggle");

    setControl(kFallbackShot, false);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.INTAKING,
        3.0, "releasing button 1 with toggle 6 still on must resume intaking");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "resumed intake must drive the roller again");
    assertEquals(700.0, shooter.getTargetRPM(), 1e-9,
        "resuming a maintained mode must also spin the flywheel back down to idle -- otherwise it "
            + "would stay at the shot target for as long as the toggle stayed on");
  }

  @Test
  @Order(7)
  @Timeout(60)
  void releasingFallbackShotResumesBounceToggle() {
    setControl(kBounceToggle, true);
    step(0.5);
    assertEquals(SuperstructureState.BOUNCING, superstructure.getSystemState(),
        "precondition: toggle 5 has the robot bouncing");

    setControl(kFallbackShot, true);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.ALIGNING
        || superstructure.getSystemState() == SuperstructureState.SHOOTING,
        3.0, "button 1 must temporarily override the bounce toggle");

    setControl(kFallbackShot, false);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.BOUNCING,
        3.0, "releasing button 1 with toggle 5 still on must resume the bounce mode");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "resumed bounce keeps the roller running");
  }

  /** Applies several control changes in ONE DriverStation update, so the scheduler never observes
   *  an intermediate board position (e.g. eject released before the shot button is seen). */
  private static void setControls(int[] buttons, boolean[] pressed) {
    for (int i = 0; i < buttons.length; i++) {
      DriverStationSim.setJoystickButton(kControlBoxPort, buttons[i], pressed[i]);
    }
    DriverStationSim.notifyNewData();
  }

  /**
   * Steps forward from a shot-target change, asserting on EVERY tick that fuel is never indexed
   * while the new target is the one in effect, and that the feed does stop promptly.
   */
  private static void assertFeedStopsWithoutEverFeedingAtTheNewTarget(
      double newTarget, String context) {
    for (int tick = 0; tick < 8; tick++) {
      step(0.02);
      assertFalse(
          shooter.getIndexerState() == Shooter.indexing.INDEX
              && shooter.getTargetRPM() == newTarget,
          context + " -- fuel was indexed while the flywheel was already commanded to the new "
              + "target; the feed must stop before the target changes");
      if (shooter.getIndexerState() == Shooter.indexing.STOP) {
        assertEquals(Shooter.Agitate.STOP, shooter.getAgitateState(),
            context + " -- the agitator must stop together with the indexer");
        return;
      }
    }
    org.junit.jupiter.api.Assertions.fail(
        context + " -- the feed never stopped after the shot target changed");
  }

  /** As above, for the fallback -> vision direction where the new target is not known up front. */
  private static void assertFeedStopsWithoutEverFeedingAtANonFallbackTarget(String context) {
    for (int tick = 0; tick < 8; tick++) {
      step(0.02);
      assertFalse(
          shooter.getIndexerState() == Shooter.indexing.INDEX
              && shooter.getTargetRPM() != kFallbackShotRPM,
          context + " -- fuel was indexed while the flywheel was already commanded off the "
              + "fallback target");
      if (shooter.getIndexerState() == Shooter.indexing.STOP) {
        assertEquals(Shooter.Agitate.STOP, shooter.getAgitateState(),
            context + " -- the agitator must stop together with the indexer");
        return;
      }
    }
    org.junit.jupiter.api.Assertions.fail(
        context + " -- the feed never stopped after the shot target changed");
  }

  /** Steps until the shooter has been observed at speed for its CURRENT target, then returns the
   *  number of ticks it took. Fails if the feed starts before that happens. */
  private static void assertFeedOnlyAfterFreshAtSpeed(String context) {
    boolean reachedSpeed = false;
    for (double elapsed = 0; elapsed < 6.0; elapsed += 0.02) {
      if (shooter.getIndexerState() == Shooter.indexing.INDEX) {
        assertTrue(reachedSpeed,
            context + " -- the feed restarted before the shooter reached the NEW target");
        return;
      }
      if (shooter.shooterAtSpeed(shooter.getTargetRPM())) {
        reachedSpeed = true;
      }
      step(0.02);
    }
    org.junit.jupiter.api.Assertions.fail(context + " -- the feed never restarted");
  }

  @Test
  @Order(8)
  @Timeout(90)
  void ejectingIntoVisionShotEstablishesSafeFeedOutputsBeforeSpinUp() {
    setControl(kUnifiedEject, true);
    stepUntil(() -> shooter.getIndexerState() == Shooter.indexing.EJECT, 3.0,
        "precondition: the unified eject is running the indexer backwards");
    assertEquals(Shooter.Agitate.OUT, shooter.getAgitateState(),
        "precondition: the eject is running the agitator outward");

    // Release eject and request the vision shot in a SINGLE DriverStation update, which is the
    // worst case: the state machine goes EJECTING -> ALIGNING with no idle tick in between.
    setControls(new int[] {kUnifiedEject, kVisionShot}, new boolean[] {false, true});
    step(0.06);

    assertTrue(superstructure.getSystemState() == SuperstructureState.ALIGNING
        || superstructure.getSystemState() == SuperstructureState.SHOOTING,
        "the shot must have taken over from the eject");
    if (superstructure.getSystemState() == SuperstructureState.ALIGNING) {
      assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
          "no stale indexing.EJECT may survive into shot spin-up");
      assertEquals(Shooter.Agitate.STOP, shooter.getAgitateState(),
          "no stale Agitate.OUT may survive into shot spin-up");
    }
    setControl(kVisionShot, false);
  }

  @Test
  @Order(9)
  @Timeout(90)
  void ejectingIntoFallbackShotEstablishesSafeFeedOutputsBeforeSpinUp() {
    setControl(kUnifiedEject, true);
    stepUntil(() -> shooter.getIndexerState() == Shooter.indexing.EJECT, 3.0,
        "precondition: the unified eject is running the indexer backwards");

    setControls(new int[] {kUnifiedEject, kFallbackShot}, new boolean[] {false, true});
    step(0.04);

    assertEquals(kFallbackShotRPM, shooter.getTargetRPM(), 1e-9,
        "the fallback target must be commanded immediately");
    if (superstructure.getSystemState() == SuperstructureState.ALIGNING) {
      assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
          "no stale indexing.EJECT may survive into fallback spin-up");
      assertEquals(Shooter.Agitate.STOP, shooter.getAgitateState(),
          "no stale Agitate.OUT may survive into fallback spin-up");
    }
    setControl(kFallbackShot, false);
  }

  @Test
  @Order(10)
  @Timeout(90)
  void switchingVisionShotToFallbackWhileFeedingStopsFeedAndReacquires() {
    setControl(kVisionShot, true);
    stepUntil(() -> shooter.getIndexerState() == Shooter.indexing.INDEX, 6.0,
        "precondition: the vision shot is feeding");
    double visionTarget = shooter.getTargetRPM();

    // Swap to the fixed fallback in one update while the feed is live.
    org.junit.jupiter.api.Assertions.assertNotEquals(kFallbackShotRPM, visionTarget,
        "precondition check: the two shot modes must command different speeds, or this test "
            + "would prove nothing");

    setControls(new int[] {kVisionShot, kFallbackShot}, new boolean[] {false, true});

    // The board is sampled once per scheduler run, so the state machine legitimately spends one
    // further tick feeding at the OLD, still-commanded target before it observes the new intent.
    // The invariant that actually matters is stronger and is checked on EVERY tick: the new
    // target is never in effect while fuel is being indexed.
    assertFeedStopsWithoutEverFeedingAtTheNewTarget(kFallbackShotRPM, "vision -> fallback");

    assertEquals(SuperstructureState.ALIGNING, superstructure.getSystemState(),
        "a target change must drop back through the pre-shot phase, not stay in SHOOTING");
    stepUntil(() -> shooter.getTargetRPM() == kFallbackShotRPM, 1.0,
        "the new fixed fallback target must be applied once the feed has stopped");

    assertFeedOnlyAfterFreshAtSpeed("vision -> fallback");
    setControl(kFallbackShot, false);
  }

  @Test
  @Order(11)
  @Timeout(90)
  void switchingFallbackToVisionShotWhileFeedingStopsFeedAndReacquires() {
    setControl(kFallbackShot, true);
    stepUntil(() -> shooter.getIndexerState() == Shooter.indexing.INDEX, 6.0,
        "precondition: the fallback shot is feeding");
    assertEquals(kFallbackShotRPM, shooter.getTargetRPM(), 1e-9,
        "precondition: feeding at the fixed fallback target");

    setControls(new int[] {kFallbackShot, kVisionShot}, new boolean[] {false, true});

    // Same invariant in the other direction. The incoming vision target is whatever the LUT gives
    // for the current pose, so it is identified as "no longer the fixed fallback target".
    assertFeedStopsWithoutEverFeedingAtANonFallbackTarget("fallback -> vision");

    assertEquals(SuperstructureState.ALIGNING, superstructure.getSystemState(),
        "a target change must drop back through the pre-shot phase in this direction too");
    stepUntil(() -> shooter.getTargetRPM() != kFallbackShotRPM, 1.0,
        "the vision/LUT target must replace the fixed fallback target once the feed has stopped");

    assertFeedOnlyAfterFreshAtSpeed("fallback -> vision");
    setControl(kVisionShot, false);
  }
}
