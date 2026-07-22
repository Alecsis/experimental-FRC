// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;

/**
 * Condition-supplier library for Superstructure's bounded sequences (see
 * Superstructure#intakeSequence(BooleanSupplier, double) / #shootingSequence(BooleanSupplier,
 * double)). Every condition here is sourced from signals genuinely simulated today (drivetrain
 * pose, PathPlanner path-following state) -- deliberately excludes any note/fuel-presence
 * signal, which needs a hardware decision not yet made (docs/superpowers/specs/
 * 2026-07-22-autonomous-completion-trigger-framework-design.md's Non-goals).
 */
public final class AutoTriggers {
  private AutoTriggers() {}

  /** True once the drivetrain's fused pose is within {@code toleranceMeters} of {@code target}. */
  public static BooleanSupplier withinToleranceOf(
      CommandSwerveDrivetrain drivetrain, Pose2d target, double toleranceMeters) {
    return () -> drivetrain.getState().Pose.getTranslation().getDistance(target.getTranslation())
        <= toleranceMeters;
  }

  /** True once PathPlanner reports no active path (path complete or none running). */
  public static BooleanSupplier pathFollowingComplete(CommandSwerveDrivetrain drivetrain) {
    return () -> !drivetrain.isFollowingAutoPath();
  }
}
