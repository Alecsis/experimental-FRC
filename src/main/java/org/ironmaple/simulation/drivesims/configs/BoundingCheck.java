package org.ironmaple.simulation.drivesims.configs;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Compatibility override for MapleSim 0.4.0-beta's validation helper.
 *
 * <p>That release uses a lower bound of 4.0 for drive gear reductions even though its
 * {@code SwerveModuleSimulationConfig} constructor documents only {@code >1} as valid. The
 * WCP SwerveX2S X3 module used by this robot is correctly configured at 3.7142857142857144:1.
 * Keep the real ratio in the physics model and suppress only this exact known-valid warning;
 * every other MapleSim bound check retains the upstream behavior.
 *
 * <p>This class intentionally mirrors the upstream fully-qualified class name. The fat-JAR task
 * excludes the dependency copy so this compatibility version is the one loaded at runtime.
 * Remove it, its JAR exclusion, and this note when MapleSim fixes the lower bound.
 */
public final class BoundingCheck {
  private static final double kWcpX2sX3DriveRatio = 3.7142857142857144;

  private BoundingCheck() {}

  public static void check(
      double value, double lowerBound, double upperBound, String variableName, String unit) {
    if (lowerBound <= value && value <= upperBound) {
      return;
    }

    if ("drive gear ratio".equals(variableName)
        && Math.abs(value - kWcpX2sX3DriveRatio) < 1e-12) {
      return;
    }

    String errorMessage =
        "The provided \"" + variableName + "\" is " + value + unit
            + ", which seems abnormal, please check its correctness";
    DriverStation.reportError(errorMessage, true);
  }
}
