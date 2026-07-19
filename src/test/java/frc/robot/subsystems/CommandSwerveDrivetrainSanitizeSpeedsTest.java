// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.generated.TunerConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Guards {@link CommandSwerveDrivetrain#sanitizeAutoSpeeds(ChassisSpeeds)}, the controller-output
 * sanitizer that sits between {@code PPHolonomicDriveController}'s unbounded feedback (kP=5 on
 * translation, confirmed architecturally unbounded -- see CLAUDE.md's sixteenth-session findings)
 * and CTRE's {@code ApplyRobotSpeeds} request. Uses the drivetrain's own {@code getKinematics()} so
 * saturation respects module geometry rather than clamping vx/vy/omega independently.
 */
class CommandSwerveDrivetrainSanitizeSpeedsTest {

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
  @Timeout(30)
  void oversizedTranslationIsBoundedToMaxSpeed() {
    // Pure translation, no rotation: every module's velocity equals the chassis translation
    // vector regardless of module position, so the desaturated magnitude is exactly the
    // drivetrain's speed-at-12-volts ceiling -- not an approximation.
    ChassisSpeeds raw = new ChassisSpeeds(20.0, 0.0, 0.0);

    ChassisSpeeds bounded = drivetrain.sanitizeAutoSpeeds(raw);

    double boundedMagnitude =
        Math.hypot(bounded.vxMetersPerSecond, bounded.vyMetersPerSecond);
    assertEquals(
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
        boundedMagnitude,
        1e-6,
        "oversized translation should be desaturated down to exactly the drivetrain's max speed");
  }

  @Test
  @Timeout(30)
  void speedsWithinLimitPassThroughUnchanged() {
    ChassisSpeeds raw = new ChassisSpeeds(2.0, 1.0, 1.0);

    ChassisSpeeds bounded = drivetrain.sanitizeAutoSpeeds(raw);

    assertEquals(raw.vxMetersPerSecond, bounded.vxMetersPerSecond, 1e-6);
    assertEquals(raw.vyMetersPerSecond, bounded.vyMetersPerSecond, 1e-6);
    assertEquals(raw.omegaRadiansPerSecond, bounded.omegaRadiansPerSecond, 1e-6);
  }
}
