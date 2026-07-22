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

  /**
   * True once PathPlanner reports no active path (path complete or none running).
   *
   * <p><b>Caveat:</b> {@link CommandSwerveDrivetrain#isFollowingAutoPath()} defaults to {@code
   * false} until PathPlanner's first {@code setLogActivePathCallback} fires with a non-empty path
   * list -- so this supplier reads {@code true} <i>before any path has ever started</i>, not just
   * after one finishes. If this is wired into a {@code parallel(path, sequence)} block (e.g. {@code
   * intakeSequence(pathFollowingComplete(drivetrain), 5.0)}), it can read {@code true} on the very
   * first tick(s), before the path callback fires, exiting the sequence immediately instead of
   * waiting for the path to actually complete. No latching/state-tracking exists to guard against
   * this -- verify the callback has fired at least once before relying on this for "path is done"
   * semantics.
   */
  public static BooleanSupplier pathFollowingComplete(CommandSwerveDrivetrain drivetrain) {
    return () -> !drivetrain.isFollowingAutoPath();
  }
}
