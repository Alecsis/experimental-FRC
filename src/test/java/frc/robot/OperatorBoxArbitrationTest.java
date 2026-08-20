// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
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
 * End-to-end operator control board behavior, driven through the real DriverStation joystick data
 * so the CommandScheduler's button loop, the operator-intent command, and the Superstructure state
 * machine all run exactly as they do on the robot.
 *
 * <p><b>Why one Robot for the whole class rather than one @Test method per class.</b> build.gradle's
 * {@code forkEvery = 1} gives this class its own JVM, and {@code RobotContainer.close()} closes a
 * {@code static} drivetrain, so a Robot may be booted only ONCE per JVM -- that is what forces the
 * one-method-per-class rule on the existing {@code RobotLifecycleTest}/{@code
 * SuperstructureEjectingTest}. Booting once in {@code @BeforeAll} honors that same constraint while
 * still allowing several assertions, which matters here: the behavior under test is precisely the
 * transitions BETWEEN control states, and a dozen separate JVM boots would buy nothing. Each method
 * starts from {@link #releaseAllControls()}, which drives the board back to "everything off" and
 * settles the state machine into STOWED, so methods do not inherit each other's control state.
 *
 * <p>Ordering is pinned only so a failure reads in the same sequence as the physical workflow; no
 * method depends on a previous one's leftovers.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class OperatorBoxArbitrationTest {
  /** The physical operator box is CommandGenericHID(1). */
  private static final int kControlBoxPort = 1;

  private static final int kFallbackShot = 1;
  private static final int kVisionShot = 2;
  private static final int kUnifiedEject = 3;
  private static final int kReservedUnbound = 4;
  private static final int kBounceToggle = 5;
  private static final int kIntakeToggle = 6;

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

    // Both HIDs OperatorControls reads must report enough buttons, or DriverStation treats every
    // read as an unplugged-joystick error and silently returns false for all of them.
    DriverStationSim.setJoystickButtonCount(kControlBoxPort, 12);
    DriverStationSim.setJoystickButtonCount(3, 12); // sim-mirror CommandXboxController
    DriverStationSim.notifyNewData();

    robot = new Robot();
    robotThread = SimRobotLoop.start(robot, "OperatorBoxArbitrationTest-competition");
    SimRobotLoop.awaitProgramStart();

    superstructure = Superstructure.getInstance(null);
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();

    SimRobotLoop.registerDiagnostics(
        () -> SimRobotLoop.operatorBoardState(kControlBoxPort, 6));

    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();

    // Teleop's rising edge schedules intake.homing(), which owns Intake for up to ~1s. Superstructure
    // deliberately yields the pivot and roller for that whole window, so settle past it before any
    // roller assertion -- that yielding is itself asserted by homingOwnsTheIntakeUninterrupted().
    step(2.0);
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

  /**
   * Releases every board control and lets the state machine settle. Two scheduler ticks are the
   * structural minimum (one to poll the button loop and run the intent command, one for the
   * Superstructure periodic() that acts on the resulting wanted-state); this steps well past that.
   */
  private static void releaseAllControls() {
    for (int button = 1; button <= 6; button++) {
      DriverStationSim.setJoystickButton(kControlBoxPort, button, false);
    }
    DriverStationSim.notifyNewData();
    step(0.3);
  }

  /** Steps in scheduler-sized increments until {@code condition} holds, or fails after a bound. */
  private static void stepUntil(BooleanSupplier condition, double timeoutSeconds, String what) {
    SimRobotLoop.stepUntil(condition, timeoutSeconds, what);
  }

  private static double pivotDegrees() {
    return SmartDashboard.getNumber("Intake/Pivot Deg", Double.NaN);
  }

  /* -------------------------------------------------------------------- tests */

  @Test
  @Order(1)
  @Timeout(60)
  void homingOwnsTheIntakeUninterrupted() {
    // The bounce routine now requires Intake, so it could in principle be scheduled on top of
    // intake.homing() and abort the hardstop seek. Superstructure yields instead. By the time
    // setup()'s settle finished, homing must have completed and released the intake.
    assertNull(CommandScheduler.getInstance().requiring(intake),
        "nothing should still own Intake once homing has finished");
    assertEquals(Intake.Roller.STOP, intake.getRollerState(),
        "an idle board leaves the roller stopped");
  }

  @Test
  @Order(2)
  @Timeout(60)
  void intakeToggleOnDrivesIntaking() {
    setControl(kIntakeToggle, true);
    step(0.3);

    assertEquals(SuperstructureState.INTAKING, superstructure.getSystemState(),
        "toggle 6 on is the persistent intake mode");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "INTAKING must actually run the intake roller");
    assertEquals(Shooter.Agitate.OUT, shooter.getAgitateState(),
        "INTAKING runs the agitator outward so incoming fuel doesn't pack against the indexer");
  }

  @Test
  @Order(3)
  @Timeout(60)
  void intakeToggleOffStopsIntaking() {
    setControl(kIntakeToggle, true);
    step(0.3);
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(), "precondition: intaking");

    setControl(kIntakeToggle, false);
    step(0.3);

    assertEquals(SuperstructureState.STOWED, superstructure.getSystemState(),
        "toggle 6 off returns to the normal idle state");
    assertEquals(Intake.Roller.STOP, intake.getRollerState(),
        "toggle 6 off must stop the intake roller");
  }

  @Test
  @Order(4)
  @Timeout(60)
  void visionShotWhileIntakeToggleHeldResumesIntakeOnRelease() {
    setControl(kIntakeToggle, true);
    step(0.3);
    assertEquals(SuperstructureState.INTAKING, superstructure.getSystemState(),
        "precondition: toggle 6 has the robot intaking");

    setControl(kVisionShot, true);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.ALIGNING
        || superstructure.getSystemState() == SuperstructureState.SHOOTING,
        3.0, "holding button 2 must take over from the intake toggle and start the shot sequence");

    // Toggle 6 is still physically ON the whole time; the operator must not have to cycle it.
    setControl(kVisionShot, false);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.INTAKING,
        3.0, "releasing button 2 with toggle 6 still on must resume intaking, not idle");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "resumed intake must actually be driving the roller again");
  }

  @Test
  @Order(5)
  @Timeout(60)
  void ejectRunsWholeFeedPathThenResumesIntakeOnRelease() {
    setControl(kIntakeToggle, true);
    step(0.3);
    assertEquals(SuperstructureState.INTAKING, superstructure.getSystemState(),
        "precondition: toggle 6 has the robot intaking");

    setControl(kUnifiedEject, true);
    step(0.3);

    // Button 3 is the unified eject: intake roller AND indexer AND agitator, not just the roller
    // the legacy button 3 drove.
    assertEquals(SuperstructureState.EJECTING, superstructure.getSystemState(),
        "button 3 must override the intake toggle");
    assertEquals(Intake.Roller.EJECT, intake.getRollerState(),
        "unified eject reverses the intake roller");
    assertEquals(Shooter.indexing.EJECT, shooter.getIndexerState(),
        "unified eject reverses the indexer -- this is what the legacy button 4 used to do");
    assertEquals(Shooter.Agitate.OUT, shooter.getAgitateState(),
        "unified eject runs the agitator outward");

    setControl(kUnifiedEject, false);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.INTAKING,
        3.0, "releasing button 3 with toggle 6 still on must resume intaking");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "resumed intake must drive the roller forward again");
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
        "resuming intake must stop the indexer -- leaving it in EJECT would spit fuel back out "
            + "for as long as the intake toggle stayed on");
  }

  @Test
  @Order(6)
  @Timeout(60)
  void bounceToggleBouncesPivotAndKeepsRollerRunning() {
    setControl(kBounceToggle, true);
    step(0.3);

    assertEquals(SuperstructureState.BOUNCING, superstructure.getSystemState(),
        "toggle 5 on is the pivot bounce/agitate mode");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "legacy toggle 5 kept the intake roller running while bouncing the pivot");

    Command owner = CommandScheduler.getInstance().requiring(intake);
    assertNotNull(owner,
        "the bounce routine must own Intake -- a requirement-free command could not be arbitrated "
            + "against homing or SysId at all");

    // The pivot must physically sweep, not merely be "in bounce mode". Measured as the range
    // covered over more than two full 1.0s bounce cycles, so the check does not depend on which
    // phase of the swing the sampling happens to start in.
    double min = Double.POSITIVE_INFINITY;
    double max = Double.NEGATIVE_INFINITY;
    for (int sample = 0; sample < 130; sample++) {
      step(0.02);
      double deg = pivotDegrees();
      min = Math.min(min, deg);
      max = Math.max(max, deg);
    }
    assertTrue(max - min > 30.0,
        "the pivot must actually sweep between the AGITATE and DOWN angles while bouncing, but "
            + "it only covered " + (max - min) + " degrees");
  }

  @Test
  @Order(7)
  @Timeout(60)
  void bounceToggleOffWithIntakeToggleOffStowsAndEndsBounce() {
    setControl(kBounceToggle, true);
    step(0.5);
    assertEquals(SuperstructureState.BOUNCING, superstructure.getSystemState(),
        "precondition: bouncing");

    setControl(kBounceToggle, false);
    step(0.3);

    assertEquals(SuperstructureState.STOWED, superstructure.getSystemState(),
        "toggle 5 off with toggle 6 off returns to the normal idle/stowed behavior");
    assertNull(CommandScheduler.getInstance().requiring(intake),
        "the bounce routine must not survive its toggle going off");
    assertEquals(Intake.Roller.STOP, intake.getRollerState(),
        "stowed leaves the roller stopped");
    stepUntil(() -> pivotDegrees() > -30.0, 3.0,
        "ending the bounce must bring the pivot back down, not leave it parked mid-swing");
  }

  @Test
  @Order(8)
  @Timeout(60)
  void bounceToggleOffWithIntakeToggleOnResumesNormalIntake() {
    setControl(kIntakeToggle, true);
    setControl(kBounceToggle, true);
    step(0.5);
    assertEquals(SuperstructureState.BOUNCING, superstructure.getSystemState(),
        "precondition: with both toggles on, toggle 5 decides the pivot behavior");

    setControl(kBounceToggle, false);
    step(0.3);

    assertEquals(SuperstructureState.INTAKING, superstructure.getSystemState(),
        "toggle 5 off while toggle 6 is still on falls through to normal persistent intake");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "normal intake keeps the roller running");
    assertNull(CommandScheduler.getInstance().requiring(intake),
        "the bounce routine must have released Intake");
  }

  @Test
  @Order(9)
  @Timeout(60)
  void bothTogglesOnDoNotFightOverTheIntake() {
    setControl(kIntakeToggle, true);
    setControl(kBounceToggle, true);
    step(0.5);

    Command owner = CommandScheduler.getInstance().requiring(intake);
    assertNotNull(owner, "precondition: the bounce routine owns Intake");

    // A scheduling storm -- two commands cancelling and rescheduling each other every tick, which
    // is exactly what independent whileTrue bindings for toggles 5 and 6 used to risk -- would
    // show up as the owning Command object changing identity across ticks.
    for (int sample = 0; sample < 50; sample++) {
      step(0.02);
      assertSame(owner, CommandScheduler.getInstance().requiring(intake),
          "the same bounce Command must stay scheduled across ticks with both toggles on; a new "
              + "instance means something cancelled and rescheduled it");
      assertEquals(SuperstructureState.BOUNCING, superstructure.getSystemState(),
          "the arbitrated state must stay put rather than alternating between the two toggles");
    }
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "the intake must remain active with both toggles on");
  }

  @Test
  @Order(10)
  @Timeout(60)
  void visionShotWhileBounceToggleHeldResumesBounceOnRelease() {
    setControl(kBounceToggle, true);
    step(0.5);
    assertEquals(SuperstructureState.BOUNCING, superstructure.getSystemState(),
        "precondition: bouncing");

    setControl(kVisionShot, true);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.ALIGNING
        || superstructure.getSystemState() == SuperstructureState.SHOOTING,
        3.0, "button 2 must temporarily override the bounce toggle");

    setControl(kVisionShot, false);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.BOUNCING,
        3.0, "releasing button 2 with toggle 5 still on must resume the bounce mode");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "resumed bounce keeps the roller running");
  }

  @Test
  @Order(11)
  @Timeout(60)
  void ejectWhileBounceToggleHeldResumesBounceOnRelease() {
    setControl(kBounceToggle, true);
    step(0.5);
    assertEquals(SuperstructureState.BOUNCING, superstructure.getSystemState(),
        "precondition: bouncing");

    setControl(kUnifiedEject, true);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.EJECTING,
        3.0, "button 3 must temporarily override the bounce toggle");
    assertEquals(Intake.Roller.EJECT, intake.getRollerState(),
        "the eject must actually reverse the roller even though the bounce toggle is still on");

    setControl(kUnifiedEject, false);
    stepUntil(() -> superstructure.getSystemState() == SuperstructureState.BOUNCING,
        3.0, "releasing button 3 with toggle 5 still on must resume the bounce mode");
  }

  @Test
  @Order(12)
  @Timeout(60)
  void reservedButton4DoesNothing() {
    setControl(kReservedUnbound, true);
    step(0.5);

    assertEquals(SuperstructureState.STOWED, superstructure.getSystemState(),
        "button 4 is reserved and intentionally unbound -- pressing it must not change state");
    assertEquals(Intake.Roller.STOP, intake.getRollerState(),
        "button 4 must not touch the intake");
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
        "button 4 must not touch the indexer");
    // The operator policy is Superstructure's default command, so it is always the owner in
    // teleop. What button 4 must not do is change the resolved intent or bring up any OTHER
    // command -- so assert the owner is still that same policy instance, unchanged.
    Command ownerBefore = CommandScheduler.getInstance().requiring(superstructure);
    step(0.3);
    assertSame(ownerBefore, CommandScheduler.getInstance().requiring(superstructure),
        "button 4 must not schedule or swap any Superstructure command -- it is reserved and "
            + "wired to nothing");

    setControl(kReservedUnbound, false);
  }
}
