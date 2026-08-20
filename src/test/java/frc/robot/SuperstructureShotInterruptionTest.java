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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Covers {@link Superstructure#shootingSequence(double)}'s interruption path.
 *
 * <p><b>The defect.</b> {@code shootingSequence} is
 * {@code sequence(runOnce(requestShoot), waitUntil(!mShotInProgress))}. Only the state machine ever
 * cleared that state -- on the normal feed timeout -- so cancelling the command cleaned up nothing:
 * {@code mWantedState} stayed ALIGNING, {@code mSystemState} stayed SHOOTING, and
 * {@code mShotInProgress} stayed true. {@code Superstructure.periodic()} is a subsystem callback
 * rather than a command, so it kept feeding regardless of who owned the subsystem.
 *
 * <p><b>Run in AUTONOMOUS, deliberately.</b> In teleop the operator-policy default command writes
 * {@code requestStow()} on its first tick, which incidentally cleans this up -- that is what
 * {@code OperatorStateCleanupTest} observes, and it would make every test here pass for a reason
 * that has nothing to do with the finalizer. In autonomous the policy body returns immediately
 * ({@code if (!DriverStation.isTeleopEnabled()) return;}), nothing else writes the wanted state, and
 * the stale shot is visible. That is also the mode the autos actually run in.
 *
 * <p>Boots once in {@code @BeforeAll} like {@code OperatorBoxArbitrationTest}: Superstructure,
 * Shooter and Intake are singletons, so a per-method boot would buy nothing.
 */
class SuperstructureShotInterruptionTest {
  private static final double kTick = 0.02;
  /** Feed window used throughout; matches the "Shooting Sequence" NamedCommand. */
  private static final double kFeedSeconds = 5.0;

  private static Robot robot;
  private static Thread robotThread;
  private static Superstructure superstructure;
  private static Shooter shooter;

  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    robot = new Robot();
    robotThread = SimRobotLoop.start(robot, "SuperstructureShotInterruptionTest-competition");
    SimRobotLoop.awaitProgramStart();

    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();

    superstructure = Superstructure.getInstance(null);
    shooter = Shooter.getInstance();
    SimRobotLoop.step(kTick);
  }

  @AfterAll
  static void teardown() {
    SimRobotLoop.shutdown();
  }

  @BeforeEach
  void settle() {
    assertTrue(robotThread.isAlive(), "startCompetition() loop should still be running");
    CommandScheduler.getInstance().cancelAll();
    superstructure.requestStow();
    stepUntilStowedAndQuiet("the superstructure to settle into STOWED between tests");
  }

  /**
   * Waits for a real stow, meaning the STOWED body has actually run.
   *
   * <p>SHOOTING's exit guard does {@code setState(mWantedState); break;} -- it changes the state and
   * returns WITHOUT executing the STOWED branch on that same tick. So {@code getSystemState()} reads
   * STOWED one full tick before {@code indexControl(STOP)} is issued. Asserting on outputs the
   * instant the state flips is a test bug, not a production one; wait on the outputs themselves.
   */
  private static void stepUntilStowedAndQuiet(String what) {
    SimRobotLoop.stepUntil(
        () -> superstructure.getSystemState() == SuperstructureState.STOWED
            && shooter.getIndexerState() == Shooter.indexing.STOP
            && shooter.getAgitateState() == Shooter.Agitate.STOP,
        4.0, what);
  }

  // ---- Normal completion is unchanged ----

  @Test
  @Timeout(60)
  void normalShotStillRunsToCompletionAndStowsItself() {
    Command shot = superstructure.shootingSequence(kFeedSeconds);
    CommandScheduler.getInstance().schedule(shot);

    SimRobotLoop.stepUntil(() -> shooter.getIndexerState() == Shooter.indexing.INDEX, 4.0,
        "the shot to spin up, settle and begin feeding");
    assertEquals(SuperstructureState.SHOOTING, superstructure.getSystemState(),
        "precondition: feeding happens in SHOOTING");

    // Must not stow early: the whole point of the conditional finalizer.
    for (int i = 0; i < 50; i++) {
      SimRobotLoop.step(kTick);
      assertEquals(Shooter.indexing.INDEX, shooter.getIndexerState(),
          "the feed must keep running mid-shot -- normal completion must not be cut short");
    }

    SimRobotLoop.stepUntil(() -> !CommandScheduler.getInstance().isScheduled(shot), 8.0,
        "the shot to finish on its own");
    stepUntilStowedAndQuiet("a completed shot to stow through the state machine, as before");
    assertEquals(SuperstructureState.STOWED, superstructure.getSystemState());
  }

  // ---- Interruption ----

  @Test
  @Timeout(60)
  void interruptionDuringAligningStowsInsteadOfLeavingTheShotArmed() {
    Command shot = superstructure.shootingSequence(kFeedSeconds);
    CommandScheduler.getInstance().schedule(shot);

    SimRobotLoop.stepUntil(() -> superstructure.getSystemState() == SuperstructureState.ALIGNING,
        2.0, "the shot to enter its spin-up phase");
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
        "precondition: ALIGNING is pre-feed");

    shot.cancel();
    stepUntilStowedAndQuiet("the cancelled shot to stow");

    assertNotEquals(SuperstructureState.ALIGNING, superstructure.getSystemState(),
        "a cancelled shot must not stay parked in spin-up, ready to feed later");
  }

  @Test
  @Timeout(60)
  void interruptionDuringShootingStowsAndStopsTheFeed() {
    Command shot = superstructure.shootingSequence(kFeedSeconds);
    CommandScheduler.getInstance().schedule(shot);

    SimRobotLoop.stepUntil(() -> shooter.getIndexerState() == Shooter.indexing.INDEX, 4.0,
        "the shot to begin feeding");
    assertEquals(SuperstructureState.SHOOTING, superstructure.getSystemState());

    shot.cancel();
    stepUntilStowedAndQuiet("the feed to stop once its owning command is cancelled");

    assertEquals(SuperstructureState.STOWED, superstructure.getSystemState());
  }

  @Test
  @Timeout(60)
  void deadlineInterruptionLeavesNoStaleShootingState() {
    // The autos wrap this in deadline[wait(N) || Orbit, Shooting Sequence]. When the deadline
    // expires first, the shot is interrupted by its enclosing group rather than cancelled directly
    // -- a different path (ParallelDeadlineGroup.end(true)) that must clean up too.
    Command group = Commands.deadline(
        Commands.waitSeconds(1.5), superstructure.shootingSequence(kFeedSeconds));
    CommandScheduler.getInstance().schedule(group);

    SimRobotLoop.stepUntil(() -> superstructure.getSystemState() == SuperstructureState.ALIGNING
        || superstructure.getSystemState() == SuperstructureState.SHOOTING, 2.0,
        "the shot inside the deadline group to become active");

    SimRobotLoop.stepUntil(() -> !CommandScheduler.getInstance().isScheduled(group), 4.0,
        "the deadline to expire and tear the group down");

    stepUntilStowedAndQuiet("a deadline-truncated shot to stow rather than keep feeding");
  }

  @Test
  @Timeout(60)
  void aCancelledShotCannotResumeAcrossDisableAndReEnable() {
    // mStateStartTimestamp is wall-clock, and disabling neutralises outputs without touching the
    // state machine. Before the finalizer, a shot cancelled at the end of autonomous stayed armed
    // and could pick straight back up on re-enable.
    Command shot = superstructure.shootingSequence(kFeedSeconds);
    CommandScheduler.getInstance().schedule(shot);
    SimRobotLoop.stepUntil(() -> shooter.getIndexerState() == Shooter.indexing.INDEX, 4.0,
        "the shot to begin feeding");

    shot.cancel(); // what teleopInit()'s m_autonomousCommand.cancel() does
    SimRobotLoop.step(kTick);

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    for (int i = 0; i < 10; i++) {
      SimRobotLoop.step(kTick);
    }

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    for (int i = 0; i < 15; i++) {
      SimRobotLoop.step(kTick);
      assertNotEquals(SuperstructureState.SHOOTING, superstructure.getSystemState(),
          "a cancelled shot must not resurrect itself after a disable/re-enable cycle");
      assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
          "and must not feed on re-enable");
    }
  }

  @Test
  @Timeout(90)
  void repeatedScheduleAndCancelStaysSafeAndAFinalShotStillRuns() {
    for (int attempt = 0; attempt < 3; attempt++) {
      Command aborted = superstructure.shootingSequence(kFeedSeconds);
      CommandScheduler.getInstance().schedule(aborted);
      SimRobotLoop.stepUntil(
          () -> superstructure.getSystemState() == SuperstructureState.ALIGNING
              || superstructure.getSystemState() == SuperstructureState.SHOOTING,
          3.0, "attempt " + attempt + " to become active");
      aborted.cancel();
      stepUntilStowedAndQuiet("attempt " + attempt + " to stow after cancel");
    }

    Command completed = superstructure.shootingSequence(kFeedSeconds);
    CommandScheduler.getInstance().schedule(completed);
    SimRobotLoop.stepUntil(() -> shooter.getIndexerState() == Shooter.indexing.INDEX, 4.0,
        "a clean shot after repeated aborts must still spin up and feed");
    SimRobotLoop.stepUntil(() -> !CommandScheduler.getInstance().isScheduled(completed), 8.0,
        "and must still finish on its own");
    stepUntilStowedAndQuiet("the final shot to stow");
  }

  // ---- Negative control ----

  /**
   * Negative control. Without it the tests above could pass for reasons unrelated to the finalizer
   * -- anything that ends with the machine STOWED would satisfy them.
   *
   * <p>This rebuilds the PRE-FIX command shape over the REAL Superstructure singleton -- a
   * {@code requestShoot} with no whole-command cleanup -- cancels it mid-feed, and shows the state
   * machine sails straight on: still SHOOTING, still commanding INDEX, with nobody owning the
   * subsystem. That is the behaviour the finalizer removes.
   */
  @Test
  @Timeout(60)
  void negativeControlPreFixShapeSurvivesCancellationAsActiveShootingState() {
    Command preFixShape = Commands.sequence(
        Commands.runOnce(() -> superstructure.requestShoot(kFeedSeconds), superstructure),
        Commands.idle(superstructure)); // stands in for waitUntil(!mShotInProgress)
    CommandScheduler.getInstance().schedule(preFixShape);

    SimRobotLoop.stepUntil(() -> shooter.getIndexerState() == Shooter.indexing.INDEX, 4.0,
        "the pre-fix shape to begin feeding");
    assertEquals(SuperstructureState.SHOOTING, superstructure.getSystemState());

    preFixShape.cancel();
    SimRobotLoop.step(kTick);
    SimRobotLoop.step(kTick);

    assertFalse(CommandScheduler.getInstance().isScheduled(preFixShape),
        "precondition: the command really was cancelled");
    assertEquals(SuperstructureState.SHOOTING, superstructure.getSystemState(),
        "negative control is vacuous unless the un-finalised shape really does leave the state"
            + " machine feeding after its owner is gone");
    assertEquals(Shooter.indexing.INDEX, shooter.getIndexerState(),
        "and really does keep the indexer running");

    // Leave the rig clean for the next method.
    superstructure.requestStow();
    stepUntilStowedAndQuiet("manual cleanup after the negative control");
  }
}
