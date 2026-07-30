// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Regression guard: {@link CommandSwerveDrivetrain#resetPose(Pose2d)} must round-trip HEADING in
 * simulation, not just translation.
 *
 * <p>{@code SimSpawnPoseOwnershipTest.resetPoseReadbackIsObservable()} only ever asserted on
 * translation, so this fault was invisible to the entire suite.
 *
 * <p>The fault: MapleSimSwerveDrivetrain drives the sim Pigeon's raw yaw off the physics body, and
 * CTRE's odometry integrates gyro DELTAS. Seeding the estimator before teleporting the body left
 * the Pigeon reporting the pre-teleport heading for a tick or two; when it caught up, odometry
 * folded that step in as real rotation, settling at {@code 2*target - heading_before}.
 *
 * <p>Measured 2026-07-29 before the fix: seeding to 78.296 deg with the body at -22.05 deg settled
 * the estimator at 177.28 deg. On the "Left Trench Neutral" autonomous control run (body at 0 deg)
 * it settled at 156.6 deg against a 78.296 deg target, handing PathPlanner a 78 deg heading error
 * on the very first sample while translation error was still 0.000 m -- which drove rotational
 * feedback to 4.1 rad/s, spun the robot for ~0.7 s, and seeded the position error that the
 * translation loop then amplified. Fixing the ordering took the same run's peak lateral error from
 * 2.151 m to 0.058 m and its minimum battery voltage from 8.35 V to 11.60 V.
 *
 * <p>Full {@link Robot} boot because the fault only exists once the sim notifier thread is running.
 * Timing is stepped rather than resumed so the readback is not confounded by the robot driving away
 * between the reset and the assertion.
 */
class ResetPoseHeadingSimTest {

  /** Matches "Left Trench Neutral"'s idealStartingState.rotation -- the pose that exposed this. */
  private static final Rotation2d TARGET = Rotation2d.fromDegrees(78.2957);

  /** Generous: the body coasts a little across the settle window. The fault was ~99 deg. */
  private static final double TOLERANCE_DEGREES = 5.0;

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

  @Test
  @Timeout(60)
  void resetPoseRoundTripsHeadingNotJustTranslation() {
    // Autonomous + enabled matches the run that exposed this, and suppresses the disabled-only
    // maybeApplySimPracticeSpawn(), which would otherwise issue a competing resetPose().
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.02);

    Pose2d target = new Pose2d(8.0, 6.6643, TARGET);

    // resetPose() is normally called by PathPlanner from the robot/scheduler thread. Schedule
    // the reset while timing is paused, then let the real robot loop execute it with one stepped
    // periodic cycle. Calling resetPose() directly from this JUnit thread races the robot loop
    // and MapleSim notifier, which can manufacture the very intermittency this test is meant to
    // detect.
    Command resetPoseCommand = Commands.runOnce(
        () -> RobotContainer.drivetrain.resetPose(target), RobotContainer.drivetrain);
    CommandScheduler.getInstance().schedule(resetPoseCommand);
    SimHooks.stepTiming(0.02);
    assertTrue(!CommandScheduler.getInstance().isScheduled(resetPoseCommand),
        "the scheduled pose reset should execute during the stepped robot cycle");

    // Let the Pigeon catch-up step land. Pre-fix, this is exactly where the doubling appeared.
    for (int i = 0; i < 4; i++) {
      SimHooks.stepTiming(0.02);
    }

    Pose2d body = RobotContainer.drivetrain.getSimulatedGroundTruthPose();
    assertTrue(body != null,
        "maple-sim body is null -- this harness cannot exercise the sim reset path, so the"
            + " assertion below would be vacuous.");

    Rotation2d estimator = RobotContainer.drivetrain.getState().Pose.getRotation();
    double driftFromBody = Math.abs(estimator.minus(body.getRotation()).getDegrees());
    double errorFromTarget = Math.abs(estimator.minus(TARGET).getDegrees());

    System.out.println("[resetPoseHeading] target=" + TARGET.getDegrees()
        + " body=" + body.getRotation().getDegrees()
        + " estimator=" + estimator.getDegrees());

    assertTrue(errorFromTarget < TOLERANCE_DEGREES,
        () -> "resetPose did not round-trip heading: asked " + TARGET.getDegrees()
            + " deg, estimator settled at " + estimator.getDegrees()
            + " deg. A value near " + (2 * TARGET.getDegrees())
            + " means the teleport delta is being integrated again.");

    // The estimator must agree with physics ground truth, not merely with the request.
    assertTrue(driftFromBody < TOLERANCE_DEGREES,
        () -> "estimator heading " + estimator.getDegrees()
            + " deg disagrees with maple-sim ground truth " + body.getRotation().getDegrees()
            + " deg after a pose reset.");
  }
}
