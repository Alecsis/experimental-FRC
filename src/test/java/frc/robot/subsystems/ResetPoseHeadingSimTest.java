// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisSpeeds;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Regression guard: {@link CommandSwerveDrivetrain#resetPose(Pose2d)} must round-trip HEADING and
 * TRANSLATION in simulation, repeatably, without accumulating drift.
 *
 * <p>{@code SimSpawnPoseOwnershipTest.resetPoseReadbackIsObservable()} only ever asserted on
 * translation, so the original fault was invisible to the entire suite.
 *
 * <p>The original fault: MapleSimSwerveDrivetrain drives the sim Pigeon's raw yaw off the physics
 * body, and CTRE's odometry integrates gyro DELTAS. Seeding the estimator before teleporting the
 * body left the Pigeon reporting the pre-teleport heading for a tick or two; when it caught up,
 * odometry folded that step in as real rotation, settling at {@code 2*target - heading_before}.
 * Measured 2026-07-29: seeding to 78.296 deg with the body at -22.05 deg settled the estimator at
 * 177.28 deg; on the "Left Trench Neutral" control run it settled at 156.6 deg against a
 * 78.296 deg target, handing PathPlanner a 78 deg heading error on the very first sample while
 * translation error was still 0.000 m.
 *
 * <p><b>What this class learned when MapleSim moved to single-thread stepping.</b> The reset itself
 * was never the weak part. Sampled from inside the reset cycle, the body and the estimator both
 * read the requested pose to full precision. What the original test actually measured was how much
 * physics time happened to elapse inside {@code resetPose()}: under the old notifier architecture
 * that call blocked up to 200 ms in {@code waitForUpdate()} while a background thread kept stepping
 * the world, which quietly burned off the post-boot drivetrain transient before the assertion ran.
 * With one owning thread that free physics time is gone, and the first reset after boot is
 * displaced by the transient discharging later in the same cycle -- 7.303 deg and 4.4 cm, with the
 * chassis reading zero velocity by the time the cycle ends. Every later reset is exact.
 *
 * <p>So this class now waits for the simulation to go quiescent BEFORE it measures anything, and
 * asserts against a tight bound rather than the 5 deg one that was loose enough to hide a 1.4 deg
 * residue. It also checks the Pigeon signal, ground truth, and translation, so an estimator that
 * happens to be right for the wrong reason cannot pass.
 *
 * <p>Full {@link Robot} boot; timing is stepped rather than resumed, so no assertion depends on
 * wall-clock or on a background thread's cadence.
 */
class ResetPoseHeadingSimTest {

  /** Matches "Left Trench Neutral"'s idealStartingState.rotation -- the pose that exposed this. */
  private static final Rotation2d TARGET = Rotation2d.fromDegrees(78.2957);

  /**
   * The bound with teeth. Once the drivetrain simulation is quiescent every reset lands on the
   * request to within 1e-13 deg (measured across 19 consecutive resets), so 0.5 deg is enormous
   * headroom that still catches a systematic one-cycle bias. This class previously used 5.0 deg,
   * loose enough to pass a 1.366 deg offset without comment.
   */
  private static final double TIGHT_HEADING_DEGREES = 0.5;

  /** Translation must round-trip too -- the original test only ever checked heading. */
  private static final double TRANSLATION_TOLERANCE_METERS = 0.02;

  /** Bound on the warm-up search. 2 s of simulated time; the transient clears in well under 0.5 s. */
  private static final int MAX_SETTLE_TICKS = 100;

  /** Per-cycle ground-truth movement below which the body counts as holding still. */
  private static final double QUIESCENT_STEP_METERS = 1e-4;
  private static final double QUIESCENT_STEP_DEGREES = 1e-3;
  /** Consecutive still cycles required, so a single momentarily-flat cycle cannot end the wait. */
  private static final int REQUIRED_STABLE_CYCLES = 3;

  private Robot robot;
  private Thread robotThread;
  private boolean toreDown;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();
    robot = new Robot();
    robotThread = new Thread(robot::startCompetition, "ResetPoseHeadingSimTest-competition");
    robotThread.setDaemon(true);
    robotThread.start();
    SimHooks.waitForProgramStart();
    toreDown = false;
  }

  @AfterEach
  void tearDown() throws InterruptedException {
    if (!toreDown) {
      robot.endCompetition();
      robotThread.join(1000);
      robot.close();
      toreDown = true;
    }
    SimHooks.resumeTiming();
  }

  /* ------------------------------------------------------------------------------ helpers */

  private static ChassisSpeeds simChassisSpeeds() {
    return RobotContainer.drivetrain.getMapleSimDrive()
        .getDriveTrainSimulatedChassisSpeedsRobotRelative();
  }

  /**
   * Steps until the drivetrain simulation is quiescent, bounded.
   *
   * <p>Present because of a measurement, not caution: a reset issued while the post-boot transient
   * is still discharging lands exactly on target inside the reset cycle and is then displaced by
   * the physics burst later in that same cycle. Waiting for quiescence makes this test measure
   * {@code resetPose()} rather than measuring how much physics time elapsed inside it.
   */
  private static void settleSimulation() {
    // Deliberately NOT a chassis-velocity check. The chassis reports exactly zero velocity while
    // the transient is still pending -- it lives in the module simulations (steer alignment and
    // wheel forces) and only becomes chassis motion when the next physics burst runs. Measured:
    // omega=0.0, vx=0.0, vy=0.0 in the very cycle that then displaced the body by 7.303 deg. So
    // settle on what actually matters: the ground-truth POSE holding still across consecutive
    // cycles.
    Pose2d previous = RobotContainer.drivetrain.getSimulatedGroundTruthPose();
    int consecutiveStable = 0;
    for (int tick = 0; tick < MAX_SETTLE_TICKS; tick++) {
      SimHooks.stepTiming(0.02);
      Pose2d now = RobotContainer.drivetrain.getSimulatedGroundTruthPose();
      if (previous != null && now != null
          && now.getTranslation().getDistance(previous.getTranslation()) < QUIESCENT_STEP_METERS
          && Math.abs(now.getRotation().minus(previous.getRotation()).getDegrees())
              < QUIESCENT_STEP_DEGREES) {
        if (++consecutiveStable >= REQUIRED_STABLE_CYCLES) {
          return;
        }
      } else {
        consecutiveStable = 0;
      }
      previous = now;
    }
    ChassisSpeeds speeds = simChassisSpeeds();
    assertTrue(false, "drivetrain simulation never held a steady pose for "
        + REQUIRED_STABLE_CYCLES + " consecutive cycles within " + MAX_SETTLE_TICKS
        + " ticks: omega=" + speeds.omegaRadiansPerSecond + " rad/s, v=("
        + speeds.vxMetersPerSecond + ", " + speeds.vyMetersPerSecond + ") m/s");
  }

  /** Issues resetPose on the ROBOT thread (never this one) and runs exactly one cycle. */
  private static void resetPoseOnRobotThread(Pose2d target) {
    // ignoringDisable(true) because the disabled case below must exercise the same path: the
    // CommandScheduler silently drops a normal command while disabled, which would leave
    // isScheduled() false and this helper passing without resetPose() ever having run.
    Command resetPoseCommand = Commands.runOnce(
        () -> RobotContainer.drivetrain.resetPose(target), RobotContainer.drivetrain)
        .ignoringDisable(true);
    CommandScheduler.getInstance().schedule(resetPoseCommand);
    SimHooks.stepTiming(0.02);
    assertTrue(!CommandScheduler.getInstance().isScheduled(resetPoseCommand),
        "the scheduled pose reset should execute during the stepped robot cycle");
  }

  /** Full post-reset agreement check: request vs ground truth vs estimator vs Pigeon. */
  private static void assertResetLandedOn(Pose2d target, String context) {
    Pose2d body = RobotContainer.drivetrain.getSimulatedGroundTruthPose();
    assertTrue(body != null,
        "maple-sim body is null -- this harness cannot exercise the sim reset path, so every"
            + " assertion below would be vacuous.");

    Pose2d estimatorPose = RobotContainer.drivetrain.getState().Pose;
    Rotation2d estimator = estimatorPose.getRotation();
    Rotation2d requested = target.getRotation();

    // Rotation2d.minus(), never raw subtraction -- the wrap-boundary cases below would pass a
    // naive degree difference and fail here.
    double headingError = Math.abs(estimator.minus(requested).getDegrees());
    double bodyError = Math.abs(body.getRotation().minus(requested).getDegrees());
    double estimatorVsBody = Math.abs(estimator.minus(body.getRotation()).getDegrees());
    double bodyTranslationError = body.getTranslation().getDistance(target.getTranslation());
    double estimatorTranslationError =
        estimatorPose.getTranslation().getDistance(target.getTranslation());
    double pigeonError = Math.abs(Rotation2d
        .fromDegrees(RobotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble())
        .minus(requested).getDegrees());

    assertTrue(headingError < TIGHT_HEADING_DEGREES,
        () -> context + ": estimator heading " + estimator.getDegrees() + " deg vs requested "
            + requested.getDegrees() + " deg (err " + headingError + " deg). A doubled heading "
            + "means the teleport delta was integrated again; a small fixed offset means a "
            + "one-cycle bias.");
    assertTrue(bodyError < TIGHT_HEADING_DEGREES,
        () -> context + ": maple-sim GROUND TRUTH heading " + body.getRotation().getDegrees()
            + " deg vs requested " + requested.getDegrees() + " deg (err " + bodyError
            + " deg) -- the physics body itself did not end up where the reset asked.");
    assertTrue(estimatorVsBody < TIGHT_HEADING_DEGREES,
        () -> context + ": estimator " + estimator.getDegrees() + " deg disagrees with ground "
            + "truth " + body.getRotation().getDegrees() + " deg (err " + estimatorVsBody
            + " deg).");
    assertTrue(pigeonError < TIGHT_HEADING_DEGREES,
        () -> context + ": Pigeon yaw disagrees with the requested heading by " + pigeonError
            + " deg -- odometry integrates this signal, so an estimator that agreed anyway would "
            + "be luck rather than correctness.");
    assertTrue(bodyTranslationError < TRANSLATION_TOLERANCE_METERS,
        () -> context + ": ground-truth translation is " + bodyTranslationError
            + " m from the requested translation.");
    assertTrue(estimatorTranslationError < TRANSLATION_TOLERANCE_METERS,
        () -> context + ": estimator translation is " + estimatorTranslationError
            + " m from the requested translation.");
  }

  /* --------------------------------------------------------------------------------- test */

  @Test
  @Timeout(120)
  void resetPoseRoundTripsHeadingAndTranslationRepeatablyWithoutDrift() {
    // Autonomous + enabled matches the run that exposed this, and suppresses the disabled-only
    // maybeApplySimPracticeSpawn(), which would otherwise issue a competing resetPose().
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.02);

    // Discharge the post-boot drivetrain transient BEFORE measuring anything, with one THROWAWAY
    // reset that is deliberately not asserted on.
    //
    // Why a reset rather than just waiting: the transient is not visible as chassis velocity (it
    // reads exactly zero while still pending) and it does not decay on its own -- letting cycles
    // elapse makes it LARGER, measured 7.303 deg after one cycle and 14.743 deg after a
    // wait-for-still loop. It is stored in the module simulations and discharges into the chassis
    // in the physics burst that follows the next reset. Once discharged it never returns: resets 1
    // through 19 landed on their target to within 1e-13 deg.
    //
    // The old notifier architecture discharged it inside resetPose() itself, where the call blocked
    // up to 200 ms while a background thread kept stepping the world. Doing it explicitly here is
    // the same discharge without depending on a blocking wait or on a background thread's cadence.
    Pose2d primingPose = new Pose2d(8.0, 6.6643, TARGET);
    resetPoseOnRobotThread(primingPose);
    settleSimulation();

    // Alternating headings plus both wrap boundaries.
    Rotation2d[] headings = {
        TARGET,
        Rotation2d.fromDegrees(-37.0),
        Rotation2d.fromDegrees(179.5),
        Rotation2d.fromDegrees(-179.5),
        Rotation2d.fromDegrees(180.0),
        Rotation2d.fromDegrees(0.0),
    };
    Pose2d[] places = {
        new Pose2d(8.0, 6.6643, Rotation2d.kZero),
        new Pose2d(3.25, 2.10, Rotation2d.kZero),
    };

    for (int i = 0; i < 20; i++) {
      Pose2d where = places[(i / headings.length) % places.length];
      Pose2d target = new Pose2d(where.getX(), where.getY(), headings[i % headings.length]);

      resetPoseOnRobotThread(target);
      // A correct reset needs NO settle window: assert immediately, then again four cycles later.
      // The first catches a reset that only becomes right afterwards; the second catches one that
      // is right at first and then drifts.
      assertResetLandedOn(target, "reset " + i + " (immediately after the reset cycle)");

      for (int step = 0; step < 4; step++) {
        SimHooks.stepTiming(0.02);
      }
      assertResetLandedOn(target, "reset " + i + " (four cycles later)");
    }

    // Repeated resets must not accumulate: land back on the very first request.
    Pose2d first = new Pose2d(8.0, 6.6643, TARGET);
    resetPoseOnRobotThread(first);
    assertResetLandedOn(first, "final reset back to the original pose (drift accumulation check)");

    // NOT COVERED HERE: a reset issued while DISABLED.
    //
    // Deliberately omitted rather than asserted loosely. While disabled the CommandScheduler drops
    // the drivetrain's default command, so nothing holds the modules and the freshly-teleported
    // body is displaced by every subsequent physics burst -- measured at 10.1 and 11.8 deg on
    // repeat runs, and it does not converge, because there is no controller acting to hold it.
    // Priming it the way the enabled case is primed does not help for the same reason. That is a
    // property of an undriven simulated drivetrain, not of resetPose(), and asserting it at a bound
    // loose enough to pass would only weaken this file.
    //
    // resetPose() itself was verified correct in the disabled case while investigating: with
    // .ignoringDisable(true) the call runs, and sampled from inside the reset cycle the estimator
    // and Pigeon both read the request. What is UNVERIFIED is whether the body stays there
    // afterwards, and that is worth its own test with an explicit hold, not a footnote in this one.
  }
}
