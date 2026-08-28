// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
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
import frc.robot.subsystems.vision.Vision;
import java.util.OptionalDouble;
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
 * Shooter tuning mode: an explicit, bounded bring-up mode that owns the SHOOTER without seizing the
 * rest of the robot.
 *
 * <p><b>What the mode is, mechanically.</b> Two ownership rules, both enforced inside
 * {@link Shooter} rather than by a competing set of commands: while tuning is armed every automatic
 * flywheel writer ({@code targetRPMShooter}) is a no-op, and while a deliberate hold-to-feed action
 * is running every automatic feed-path writer ({@code indexControl}, {@code setAgitator}) is a
 * no-op. {@code Superstructure.requestShot()} additionally refuses to start a shot while tuning is
 * armed. Everything else -- intake, drivetrain, eject -- is untouched, which is the property the
 * "intake remains usable" tests below pin.
 *
 * <p>Boots one {@code Robot} per class, like every other operator-board test here; see
 * {@code OperatorBoxArbitrationTest}'s javadoc for why.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class ShooterTuningModeTest {
  private static final int kControlBoxPort = 1;
  private static final int kFallbackShot = 1;
  private static final int kVisionShot = 2;
  private static final int kIntakeToggle = 6;

  /** The legacy fixed-RPM fallback target, in Kraken rotor RPM. */
  private static final double kFallbackShotRPM = 1600;
  /** Superstructure's STOWED idle flywheel target. */
  private static final double kIdleRPM = 700;
  /** An arbitrary manual target, chosen to collide with nothing the automatic policy produces. */
  private static final double kManualRPM = 2345;

  private static Robot robot;
  private static Thread robotThread;
  private static Superstructure superstructure;
  private static Intake intake;
  private static Shooter shooter;
  private static Vision vision;

  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    DriverStationSim.setJoystickButtonCount(kControlBoxPort, 12);
    DriverStationSim.setJoystickButtonCount(3, 12);
    DriverStationSim.notifyNewData();

    robot = new Robot();
    robotThread = SimRobotLoop.start(robot, "ShooterTuningModeTest-competition");
    SimRobotLoop.awaitProgramStart();

    superstructure = Superstructure.getInstance(null);
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();
    vision = Vision.getInstance(null);

    SimRobotLoop.registerDiagnostics(() -> SimRobotLoop.operatorBoardState(kControlBoxPort, 6));

    enterTeleop();
    step(2.0); // let teleop's intake.homing() finish and release Intake
  }

  @AfterAll
  static void teardown() {
    releaseAllControls();
    shooter.exitTuningMode();
    SimRobotLoop.shutdown();
  }

  @BeforeEach
  void resetBoard() {
    assertTrue(robotThread.isAlive(), "startCompetition() loop should still be running");
    CommandScheduler.getInstance().cancelAll();
    releaseAllControls();
    shooter.exitTuningMode();
    enterTeleop();
    step(0.4);
    assertFalse(shooter.isTuningMode(), "each test must start with tuning disarmed");
  }

  /* --------------------------------------------- 1. tuning owns the flywheel outright -------- */

  /**
   * The direct proof, at the exact seam the vision path uses. {@code updateShooterRPM()}'s vision
   * branch is literally {@code shooter.targetRPMShooter(rpm)}, so calling it with a LUT-shaped value
   * and observing that the manual target survives is the strongest statement available -- stronger
   * than only observing that the shot sequence never starts, which would pass vacuously.
   */
  @Test
  @Order(1)
  @Timeout(60)
  void visionLutCannotOverwriteTheManualTargetInTuningMode() {
    armTuning(kManualRPM);

    double lutValue = vision.calculateRPM().orElse(1500.0);
    shooter.targetRPMShooter(lutValue); // exactly what Superstructure's VISION branch does
    step(0.1);

    assertEquals(kManualRPM, shooter.getTargetRPM(), 1e-9,
        "the vision LUT must not be able to move a manually-dialled tuning target");
  }

  /** Same, for the fixed fallback branch -- a different literal, the same single seam. */
  @Test
  @Order(2)
  @Timeout(60)
  void fixedFallbackCannotOverwriteTheManualTargetInTuningMode() {
    armTuning(kManualRPM);

    shooter.targetRPMShooter(kFallbackShotRPM); // exactly what the FIXED_FALLBACK branch does
    step(0.1);

    assertEquals(kManualRPM, shooter.getTargetRPM(), 1e-9,
        "the fixed fallback must not be able to move a manually-dialled tuning target");
  }

  /**
   * And the same must hold against the LIVE board: holding either shot control must neither start a
   * shot nor move the flywheel target, because the request is refused before the state machine ever
   * reaches ALIGNING.
   */
  @Test
  @Order(3)
  @Timeout(90)
  void holdingEitherShotControlInTuningModeStartsNoShotAndMovesNoTarget() {
    armTuning(kManualRPM);

    for (int button : new int[] {kVisionShot, kFallbackShot}) {
      setControl(button, true);
      step(1.0);

      assertNotEquals(SuperstructureState.ALIGNING, superstructure.getSystemState(),
          "button " + button + " must not start an automatic shot while tuning is armed");
      assertNotEquals(SuperstructureState.SHOOTING, superstructure.getSystemState(),
          "button " + button + " must not start an automatic shot while tuning is armed");
      assertEquals(kManualRPM, shooter.getTargetRPM(), 1e-9,
          "button " + button + " must not move the manual target");

      setControl(button, false);
      step(0.3);
    }
  }

  /**
   * STOWED's idle RPM is an automatic writer too, and runs on EVERY tick -- so a mode that only
   * blocked the vision and fallback branches would still lose the manual target within one loop.
   */
  @Test
  @Order(4)
  @Timeout(60)
  void theIdleRpmPolicyCannotOverwriteTheManualTargetEither() {
    armTuning(kManualRPM);
    step(1.0); // ~50 STOWED ticks, each of which calls targetRPMShooter(kIdleRPM)

    assertEquals(SuperstructureState.STOWED, superstructure.getSystemState(),
        "precondition: an idle board settles the machine into STOWED, which idles the flywheel");
    assertEquals(kManualRPM, shooter.getTargetRPM(), 1e-9,
        "STOWED's idle RPM must not be able to move a manually-dialled tuning target");
  }

  /* ---------------------------------------------------- 2. no automatic feed while tuning ---- */

  /** With tuning armed and a quiet board, the feed path must sit at STOP indefinitely. */
  @Test
  @Order(5)
  @Timeout(60)
  void noAutomaticIndexingOrAgitationInTuningMode() {
    armTuning(kManualRPM);

    for (int i = 0; i < 25; i++) {
      step(0.02);
      assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
          "the indexer must default to STOP throughout tuning (tick " + i + ")");
      assertEquals(Shooter.Agitate.STOP, shooter.getAgitateState(),
          "the agitator must default to STOP throughout tuning (tick " + i + ")");
    }
    assertFalse(shooter.isTuningFeedActive(), "nothing may arm the feed on its own");
  }

  /**
   * The deliberate hold-to-feed is the ONLY thing that feeds, it wins against
   * {@code Superstructure.periodic()}'s per-tick {@code indexControl(STOP)}, and releasing it stops
   * the feed immediately rather than latching.
   */
  @Test
  @Order(6)
  @Timeout(60)
  void theDeliberateHoldToFeedOwnsTheFeedPathAndReleasesImmediately() {
    armTuning(kManualRPM);

    Command feed = shooter.tuningFeedCmd();
    CommandScheduler.getInstance().schedule(feed);
    step(0.2); // ~10 ticks of STOWED writing indexControl(STOP) underneath

    assertTrue(shooter.isTuningFeedActive(), "the hold must take feed-path ownership");
    assertEquals(Shooter.indexing.INDEX, shooter.getIndexerState(),
        "the deliberate feed must beat the state machine's per-tick STOP");
    assertEquals(Shooter.Agitate.IN, shooter.getAgitateState());

    feed.cancel();
    step(0.06);

    assertFalse(shooter.isTuningFeedActive(), "releasing must drop feed-path ownership");
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
        "releasing the hold must stop the feed immediately, not latch it");
    assertEquals(Shooter.Agitate.STOP, shooter.getAgitateState());
  }

  /** And the feed binding is inert unless tuning is armed, so it cannot feed during a match. */
  @Test
  @Order(7)
  @Timeout(60)
  void theHoldToFeedIsInertOutsideTuningMode() {
    assertFalse(shooter.isTuningMode(), "precondition: tuning disarmed");

    Command feed = shooter.tuningFeedCmd();
    CommandScheduler.getInstance().schedule(feed);
    step(0.2);

    assertFalse(shooter.isTuningFeedActive(),
        "the tuning feed must not take ownership when tuning is not armed");
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
        "and it must not have fed anything");

    feed.cancel();
    step(0.06);
  }

  /** The +/- RPM controls adjust a number and nothing else -- they must never be able to feed. */
  @Test
  @Order(8)
  @Timeout(60)
  void theStepControlsAdjustRpmWithoutEverFeeding() {
    armTuning(1000);
    double stepRpm = shooter.getTuningStepRPM();

    shooter.stepTuningTargetRPM(1);
    shooter.stepTuningTargetRPM(1);
    step(0.1);
    assertEquals(1000 + 2 * stepRpm, shooter.getTargetRPM(), 1e-9, "+StepRPM must raise the target");

    shooter.stepTuningTargetRPM(-1);
    step(0.1);
    assertEquals(1000 + stepRpm, shooter.getTargetRPM(), 1e-9, "-StepRPM must lower the target");

    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
        "adjusting RPM must never touch the indexer");
    assertEquals(Shooter.Agitate.STOP, shooter.getAgitateState(),
        "adjusting RPM must never touch the agitator");
    assertFalse(shooter.isTuningFeedActive());
  }

  /* ------------------------------------------------------- 3. the rest of the robot is free -- */

  /**
   * Tuning owns the SHOOTER, not the Superstructure. The intake must keep following the live board
   * while the flywheel stays on the manual target -- if tuning had been built as a command that
   * seized Superstructure, this is the test that would fail.
   */
  @Test
  @Order(9)
  @Timeout(90)
  void theIntakeRemainsFullyUsableWhileTuningOwnsTheShooter() {
    armTuning(kManualRPM);

    setControl(kIntakeToggle, true);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.INTAKING, 3.0,
        "the intake toggle to take effect while tuning is armed");

    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "the intake roller must still follow the operator board during shooter tuning");
    assertEquals(kManualRPM, shooter.getTargetRPM(), 1e-9,
        "and intaking must not have disturbed the manual flywheel target");
    assertTrue(intake.isHomed(),
        "precondition: the pivot homed at teleop entry, so INTAKING's DOWN request is honoured");

    setControl(kIntakeToggle, false);
    stepUntil(() -> intake.getRollerState() == Intake.Roller.STOP, 3.0,
        "releasing the toggle to stop the roller");
    assertEquals(kManualRPM, shooter.getTargetRPM(), 1e-9,
        "and releasing it must not have disturbed the manual target either");
  }

  /* ----------------------------------------------------------------- 4. leaving the mode ----- */

  /** Disarming hands the flywheel straight back to the normal policy, with no explicit hand-off. */
  @Test
  @Order(10)
  @Timeout(60)
  void exitingTuningReturnsTheFlywheelToNormalPolicy() {
    armTuning(kManualRPM);
    assertEquals(kManualRPM, shooter.getTargetRPM(), 1e-9, "precondition: tuning owns the flywheel");

    shooter.setTuningMode(false);
    stepUntil(() -> shooter.getTargetRPM() == kIdleRPM, 2.0,
        "the normal STOWED idle policy to reclaim the flywheel");

    assertFalse(shooter.isTuningMode());
    assertEquals(kIdleRPM, shooter.getTargetRPM(), 1e-9);
  }

  /**
   * A mode left armed before a disable must NOT come back on the next enable. The operator
   * re-enabling may be expecting match behaviour, and a resurrected tuning mode would silently
   * suppress the vision LUT, the fallback shot and the whole shooting sequence.
   */
  @Test
  @Order(11)
  @Timeout(90)
  void disabledThenEnabledDoesNotResurrectTuning() {
    armTuning(kManualRPM);
    assertTrue(shooter.isTuningMode(), "precondition: tuning is armed");

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    step(0.5);

    assertFalse(shooter.isTuningMode(),
        "the disabled transition must disarm tuning rather than leaving it latched");

    enterTeleop();
    step(2.0); // teleop entry re-runs intake.homing(); let it finish

    assertFalse(shooter.isTuningMode(), "and re-enabling must not resurrect it");
    stepUntil(() -> shooter.getTargetRPM() == kIdleRPM, 3.0,
        "normal policy to own the flywheel again after the enable");
  }

  /* ------------------------------------------- 5. production behaviour outside tuning mode --- */

  /** With tuning disarmed, the fallback shot must behave exactly as it always has. */
  @Test
  @Order(12)
  @Timeout(90)
  void theFallbackShotIsUnchangedOutsideTuningMode() {
    assertFalse(shooter.isTuningMode(), "precondition: tuning disarmed");

    setControl(kFallbackShot, true);
    stepUntil(() -> shooter.getTargetRPM() == kFallbackShotRPM, 3.0,
        "the fallback shot to command its fixed 1600 rotor RPM");
    assertEquals(kFallbackShotRPM, shooter.getTargetRPM(), 1e-9);

    setControl(kFallbackShot, false);
    stepUntil(() -> shooter.getTargetRPM() == kIdleRPM, 3.0, "releasing to return to idle");
  }

  /** With tuning disarmed, the vision shot must still be driven by the LUT. */
  @Test
  @Order(13)
  @Timeout(90)
  void theVisionShotIsUnchangedOutsideTuningMode() {
    assertFalse(shooter.isTuningMode(), "precondition: tuning disarmed");

    setControl(kVisionShot, true);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.ALIGNING
        || superstructure.getSystemState() == SuperstructureState.SHOOTING, 3.0,
        "the vision shot to enter the shot phases");

    OptionalDouble lut = vision.calculateRPM();
    assertTrue(lut.isPresent(), "precondition: the LUT resolves a target from the AprilTag layout");
    assertEquals(lut.getAsDouble(), shooter.getTargetRPM(), 5.0,
        "outside tuning mode the vision shot must still be sized by the LUT");

    setControl(kVisionShot, false);
    step(0.5);
  }

  /**
   * Autonomous and teleop must size a shot from the SAME geometry. Both request paths funnel
   * through {@code updateShooterRPM()}'s single VISION branch, which reads
   * {@code Vision.calculateRPM()}, which measures from the shooter origin -- so at one pose the two
   * modes must agree.
   *
   * <p>The tolerance is not slack in the claim: the sim keeps stepping between the two
   * measurements, so the pose drifts a few millimetres and a continuous LUT maps that to a few
   * tenths of an RPM. Anything structurally different -- a robot-centre distance on one path, a
   * different table, a stale target -- moves the answer by far more than this.
   *
   * <p>Ordered last because it leaves and re-enters teleop.
   */
  @Test
  @Order(14)
  @Timeout(120)
  void autonomousAndTeleopVisionShotsUseIdenticalShotGeometry() {
    assertFalse(shooter.isTuningMode(), "precondition: tuning disarmed");

    // ---- autonomous: the bounded NamedCommand-shaped request.
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    step(0.2);

    superstructure.requestShoot(5.0);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.ALIGNING
        || superstructure.getSystemState() == SuperstructureState.SHOOTING, 3.0,
        "the autonomous shot to enter the shot phases");
    double autoRpm = shooter.getTargetRPM();
    double autoLut = vision.calculateRPM().orElseThrow();
    assertEquals(autoLut, autoRpm, 5.0, "the autonomous shot must be sized by the shooter-origin LUT");

    superstructure.requestStow();
    step(0.5);

    // ---- teleop: the hold-to-shoot request off the live board.
    enterTeleop();
    step(2.0); // teleop entry re-runs intake.homing()
    setControl(kVisionShot, true);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.ALIGNING
        || superstructure.getSystemState() == SuperstructureState.SHOOTING, 3.0,
        "the teleop shot to enter the shot phases");
    double teleopRpm = shooter.getTargetRPM();
    double teleopLut = vision.calculateRPM().orElseThrow();
    assertEquals(teleopLut, teleopRpm, 5.0, "the teleop shot must be sized by the same LUT");

    setControl(kVisionShot, false);
    step(0.5);

    assertEquals(autoRpm, teleopRpm, 5.0,
        "autonomous and teleop must size a shot from identical geometry at the same pose");
  }

  /* ------------------------------------------- 6. tuning can never leak into autonomous ------ */

  /**
   * The blocker this test exists for: shooter tuning left armed when autonomous starts silently
   * disables EVERY autonomous shot.
   *
   * <p><b>Why {@code disabledInit()} was not enough.</b> That hook fires at the DISABLE TRANSITION.
   * Arming tuning from the dashboard while the robot then sits disabled happens strictly after it,
   * so the armed mode survives into the next enable. If that enable is autonomous,
   * {@code Superstructure.requestShot()} refuses and stows, "Shooting Sequence" satisfies its
   * {@code waitUntil(!mShotInProgress)} on the spot, and the auto completes its path having fired
   * nothing -- with no error anywhere. This test reproduces exactly that sequence: arm while
   * DISABLED, then enable autonomous.
   *
   * <p>Driven through the real DriverStation lifecycle rather than by calling
   * {@code autonomousInit()} directly, because a direct call would bypass the robot loop that
   * actually invokes it -- see the note in {@code build.gradle}'s test block.
   *
   * <p>Ordered last: it leaves and re-enters teleop.
   */
  @Test
  @Order(15)
  @Timeout(120)
  void enteringAutonomousAlwaysDisarmsTuningAndRestoresNormalShots() {
    // ---- arm tuning while DISABLED, i.e. after disabledInit() has already run.
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    step(0.5);

    shooter.setTuningMode(true);
    shooter.setTuningTargetRPM(kManualRPM);
    step(0.2);
    assertTrue(shooter.isTuningMode(),
        "precondition: tuning is armed while disabled -- disabledInit() cannot have cleared this");

    // ---- enable AUTONOMOUS through the real lifecycle.
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
    step(0.5);

    assertFalse(shooter.isTuningMode(),
        "entering autonomous must disarm shooter tuning, however it was armed");
    assertFalse(shooter.isTuningFeedActive(),
        "and must leave no tuning feed ownership behind");

    // ---- and a normal autonomous shot must now be ACCEPTED, not refused into a stow.
    superstructure.requestShoot(5.0);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.ALIGNING
        || superstructure.getSystemState() == SuperstructureState.SHOOTING, 3.0,
        "the autonomous shot request to be accepted after tuning was cleared");

    assertTrue(shooter.getTargetRPM() > 0,
        "the accepted shot must actually command the flywheel rather than leaving it at the "
            + "manual tuning target");
    assertNotEquals(kManualRPM, shooter.getTargetRPM(), 1e-9,
        "and the automatic policy -- not the stale manual target -- must own the flywheel again");

    superstructure.requestStow();
    step(0.5);
    enterTeleop();
    step(2.0); // teleop entry re-runs intake.homing(); let it finish before the class tears down
  }

  /* -------------------------------------------------------------------------------- helpers -- */

  private static void armTuning(double manualRpm) {
    shooter.setTuningMode(true);
    shooter.setTuningTargetRPM(manualRpm);
    step(0.1);
    assertTrue(shooter.isTuningMode(), "precondition: tuning armed");
    assertEquals(manualRpm, shooter.getTargetRPM(), 1e-9,
        "precondition: the manual target owns the flywheel");
  }

  private static void enterTeleop() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
  }

  private static void step(double seconds) {
    SimRobotLoop.step(seconds);
  }

  private static void stepUntil(BooleanSupplier condition, double timeoutSeconds, String what) {
    SimRobotLoop.stepUntil(condition, timeoutSeconds, what);
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
}
