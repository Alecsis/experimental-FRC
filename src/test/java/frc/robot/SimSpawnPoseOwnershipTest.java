// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Regression guard for the mid-auto teleport fixed 2026-07-19.
 *
 * <p>The DS can attach at the same instant it enables into autonomous. When that happened, the
 * alliance resolved for the first time on an ENABLED tick, and the sim practice spawn -- then
 * sharing the operator-perspective guard {@code !m_hasAppliedOperatorPerspective || isDisabled()}
 * -- reset the pose one loop after AutoBuilder had already seeded the path start, teleporting the
 * robot to midfield. Evidence: logs/akit_26-07-19_00-52-32.wpilog, pose steps
 * (12.887, 0.570) -> (8.259, 4.022) across t=3.88s -> 3.90s with Enabled and Autonomous both true.
 *
 * <p>Calls {@link CommandSwerveDrivetrain#periodic()} directly rather than driving
 * {@link Robot#startCompetition()} (the RobotLifecycleTest pattern) because the assertion is on
 * CTRE's own {@code getState().Pose}, not on an AdvantageKit-logged value -- so the
 * periodicBeforeUser/periodicAfterUser flush cycle that RobotLifecycleTest needs is not required
 * here. {@code mapleSim} is non-null in this harness because the constructor calls
 * {@code startSimThread()} under {@code Utils.isSimulation()}, so the spawn path is genuinely
 * reachable; {@link #resetPoseReadbackIsObservable()} exists to prove that rather than assume it.
 */
class SimSpawnPoseOwnershipTest {

  /** Stand-in for a pose AutoBuilder would have seeded; far from simPracticeSpawn's midfield. */
  private static final Pose2d PATH_START = new Pose2d(12.8865, 0.5700, Rotation2d.fromDegrees(-90.0));

  /** simPracticeSpawn sits at field centre; anything within this of it means the spawn fired. */
  private static final double SPAWN_MATCH_TOLERANCE_METERS = 0.25;

  private CommandSwerveDrivetrain drivetrain;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    // Cold DS: not attached, no alliance. This is the state that reproduced the bug.
    DriverStationSim.setDsAttached(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    drivetrain = TunerConstants.createDrivetrain();
  }

  @AfterEach
  void teardown() {
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  /**
   * Guards the main test against a false positive. If resetPose() did not show up in
   * getState().Pose, the main test's assertion would pass no matter what periodic() did.
   */
  @Test
  @Timeout(30)
  void resetPoseReadbackIsObservable() {
    drivetrain.resetPose(PATH_START);
    Pose2d readback = drivetrain.getState().Pose;
    assertTrue(
        readback.getTranslation().getDistance(PATH_START.getTranslation()) < 0.1,
        () ->
            "resetPose() is not observable through getState().Pose in this harness (got "
                + readback
                + "). The spawn-ownership test below would be vacuous -- fix the harness before"
                + " trusting it.");
  }

  @Test
  @Timeout(30)
  void spawnDoesNotStompPathStartPoseWhenDsAttachesAtAutoEnable() {
    // Disabled with no DS attached: alliance is unknown, so the spawn must not have latched.
    for (int i = 0; i < 5; i++) {
      drivetrain.periodic();
    }

    // Stand in for autonomousInit() -> AutoBuilder.resetOdom(pathStart).
    drivetrain.resetPose(PATH_START);

    // The trigger: DS attaches, alliance resolves, and we enable into auto all at once.
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    assertTrue(
        DriverStation.getAlliance().isPresent(),
        "harness precondition: alliance must resolve on the enabled tick, otherwise neither the old"
            + " nor the new code would reach the spawn and this test proves nothing");

    for (int i = 0; i < 10; i++) {
      drivetrain.periodic();
    }

    Pose2d spawn = FieldConstants.simPracticeSpawn(Alliance.Blue);
    Pose2d actual = drivetrain.getState().Pose;
    double distanceToSpawn = actual.getTranslation().getDistance(spawn.getTranslation());

    assertTrue(
        distanceToSpawn > SPAWN_MATCH_TOLERANCE_METERS,
        () ->
            "Pose was teleported to simPracticeSpawn while enabled in autonomous. Expected to stay"
                + " near "
                + PATH_START
                + " but got "
                + actual
                + " (distance to spawn "
                + distanceToSpawn
                + " m). The sim practice spawn must never reset pose while enabled.");
  }
}
