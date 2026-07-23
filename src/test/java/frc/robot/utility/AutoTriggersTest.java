// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Constructs CommandSwerveDrivetrain directly via TunerConstants.createDrivetrain(), matching
 * CommandSwerveDrivetrainSanitizeSpeedsTest's own precedent -- CommandSwerveDrivetrain is not a
 * singleton (docs/claudex/architecture.md's "Rule 2 note"), so a fresh instance per test class is
 * safe without the forkEvery=1/full-Robot machinery the Superstructure-touching tests need.
 */
class AutoTriggersTest {
  private CommandSwerveDrivetrain drivetrain;

  @BeforeEach
  void setup() {
    HAL.initialize(500, 0);
    DriverStationSim.resetData();
    SimHooks.pauseTiming();
    drivetrain = TunerConstants.createDrivetrain();
  }

  @AfterEach
  void teardown() {
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  @Test
  @Timeout(15)
  void withinToleranceOfIsFalseFarAwayAndTrueAtTarget() {
    Pose2d farAway = new Pose2d(20.0, 20.0, Rotation2d.kZero);
    BooleanSupplier nearOrigin = AutoTriggers.withinToleranceOf(drivetrain, Pose2d.kZero, 0.5);

    drivetrain.resetPose(farAway);
    assertFalse(nearOrigin.getAsBoolean(), "20m away must read false for a 0.5m tolerance");

    drivetrain.resetPose(Pose2d.kZero);
    assertTrue(nearOrigin.getAsBoolean(),
        "after resetPose(kZero), a 0.5m tolerance around the origin must read true");
  }

  @Test
  @Timeout(15)
  void pathFollowingCompleteReflectsIsFollowingAutoPath() {
    BooleanSupplier complete = AutoTriggers.pathFollowingComplete(drivetrain);

    assertTrue(complete.getAsBoolean(),
        "no path has ever run -- isFollowingAutoPath() defaults false, so 'complete' must be true");
  }
}
