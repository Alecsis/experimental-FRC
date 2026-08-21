// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

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

  @Test
  @Timeout(30)
  void prepareAutoSpeedsDiscretizesBeforeSanitizing() {
    // A combined translation+rotation command is exactly the case ChassisSpeeds.discretize()
    // exists for -- it couples a small vx/vy correction in from omega. Comparing against an
    // independently-computed expected value (built from the same public API, not a magic number)
    // pins the composition order: discretize() must run before sanitizeAutoSpeeds(), matching
    // docs/Path_Following_Tuning_Readiness_Audit.md's own sequencing note.
    ChassisSpeeds raw = new ChassisSpeeds(2.0, 1.0, 1.5);

    ChassisSpeeds actual = drivetrain.prepareAutoSpeeds(raw);

    ChassisSpeeds expectedDiscretized =
        ChassisSpeeds.discretize(raw, edu.wpi.first.wpilibj.TimedRobot.kDefaultPeriod);
    ChassisSpeeds expected = drivetrain.sanitizeAutoSpeeds(expectedDiscretized);

    assertEquals(expected.vxMetersPerSecond, actual.vxMetersPerSecond, 1e-9);
    assertEquals(expected.vyMetersPerSecond, actual.vyMetersPerSecond, 1e-9);
    assertEquals(expected.omegaRadiansPerSecond, actual.omegaRadiansPerSecond, 1e-9);
  }

  @Test
  @Timeout(30)
  void prepareAutoSpeedsActuallyChangesCombinedTranslationRotationInput() {
    // Guards against a no-op discretize() call (e.g. a future refactor accidentally passing
    // dtSeconds=0, which would make discretize() an identity function and silently defeat this
    // whole phase). Pure translation or pure rotation alone would not exercise the coupling term.
    ChassisSpeeds raw = new ChassisSpeeds(2.0, 1.0, 1.5);

    ChassisSpeeds prepared = drivetrain.prepareAutoSpeeds(raw);
    ChassisSpeeds sanitizedWithoutDiscretize = drivetrain.sanitizeAutoSpeeds(raw);

    boolean differs =
        Math.abs(prepared.vxMetersPerSecond - sanitizedWithoutDiscretize.vxMetersPerSecond) > 1e-9
            || Math.abs(prepared.vyMetersPerSecond - sanitizedWithoutDiscretize.vyMetersPerSecond) > 1e-9;
    assertTrue(differs, "discretize() should introduce a measurable vx/vy coupling term "
        + "for a combined translation+rotation command");
  }
}
