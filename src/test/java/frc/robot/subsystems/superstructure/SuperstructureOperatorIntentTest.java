// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.superstructure.Superstructure.OperatorIntent;
import org.junit.jupiter.api.Test;

/**
 * Pure arbitration coverage for the operator control board. {@link
 * Superstructure#resolveOperatorIntent(boolean, boolean, boolean, boolean, boolean)} is static and
 * side-effect free precisely so the priority/resume rules can be pinned down exhaustively here --
 * no HAL, no Robot boot, no singletons, no scheduler timing. The end-to-end subsystem effects of
 * each intent are covered separately by {@code OperatorBoxArbitrationTest} and {@code
 * OperatorBoxFallbackShotTest}.
 *
 * <p>The whole point of resolving from live control state rather than a remembered flag is that
 * the maintained switches (5 and 6) are the physical source of truth: there is no latch that can
 * disagree with the panel, which is what the legacy {@code ctrlBtn} boolean could do.
 *
 * <p>Argument order throughout is (button1 fallbackShot, button2 visionShot, button3 eject,
 * button5 bounce, button6 intake). Button 4 is deliberately absent -- it is reserved and unbound.
 */
class SuperstructureOperatorIntentTest {
  private static OperatorIntent resolve(
      boolean fallbackShot, boolean visionShot, boolean eject, boolean bounce, boolean intake) {
    return Superstructure.resolveOperatorIntent(fallbackShot, visionShot, eject, bounce, intake);
  }

  @Test
  void nothingPressedIsStowed() {
    assertEquals(OperatorIntent.STOWED, resolve(false, false, false, false, false),
        "with the whole board idle the robot must fall back to the stowed/idle posture");
  }

  @Test
  void intakeToggleAloneIntakes() {
    assertEquals(OperatorIntent.INTAKING, resolve(false, false, false, false, true),
        "toggle 6 on its own is the persistent intake mode");
  }

  @Test
  void bounceToggleAloneBounces() {
    assertEquals(OperatorIntent.BOUNCING, resolve(false, false, false, true, false),
        "toggle 5 on its own is the pivot bounce/agitate mode");
  }

  @Test
  void bounceToggleOutranksIntakeToggleWhenBothOn() {
    assertEquals(OperatorIntent.BOUNCING, resolve(false, false, false, true, true),
        "both maintained switches on: toggle 5 decides the special pivot behavior, and BOUNCING "
            + "keeps the roller running, so nothing is lost by toggle 6 losing this arbitration");
  }

  @Test
  void ejectOutranksEverything() {
    assertEquals(OperatorIntent.EJECTING, resolve(true, true, true, true, true),
        "button 3 is the highest-priority momentary override -- unjamming must win over any "
            + "shoot request and over both maintained toggles");
  }

  @Test
  void visionShotOutranksFallbackShotAndBothToggles() {
    assertEquals(OperatorIntent.VISION_SHOT, resolve(true, true, false, true, true),
        "button 2 outranks the button 1 fallback and both maintained toggles");
  }

  @Test
  void fallbackShotOutranksBothToggles() {
    assertEquals(OperatorIntent.FALLBACK_SHOT, resolve(true, false, false, true, true),
        "button 1 is a momentary override, so it beats the maintained switches");
  }

  @Test
  void releasingVisionShotResumesIntakeToggle() {
    assertEquals(OperatorIntent.VISION_SHOT, resolve(false, true, false, false, true),
        "toggle 6 held + button 2 pressed shoots");
    assertEquals(OperatorIntent.INTAKING, resolve(false, false, false, false, true),
        "releasing button 2 with toggle 6 still physically on resumes intake without the operator "
            + "cycling the switch -- the legacy ctrlBtn behavior, derived instead of remembered");
  }

  @Test
  void releasingEjectResumesIntakeToggle() {
    assertEquals(OperatorIntent.EJECTING, resolve(false, false, true, false, true),
        "toggle 6 held + button 3 pressed ejects");
    assertEquals(OperatorIntent.INTAKING, resolve(false, false, false, false, true),
        "releasing button 3 with toggle 6 still physically on resumes intake");
  }

  @Test
  void releasingVisionShotResumesBounceToggle() {
    assertEquals(OperatorIntent.VISION_SHOT, resolve(false, true, false, true, false),
        "toggle 5 held + button 2 pressed shoots");
    assertEquals(OperatorIntent.BOUNCING, resolve(false, false, false, true, false),
        "releasing button 2 with toggle 5 still physically on resumes the bounce mode");
  }

  @Test
  void releasingEjectResumesBounceToggle() {
    assertEquals(OperatorIntent.EJECTING, resolve(false, false, true, true, false),
        "toggle 5 held + button 3 pressed ejects");
    assertEquals(OperatorIntent.BOUNCING, resolve(false, false, false, true, false),
        "releasing button 3 with toggle 5 still physically on resumes the bounce mode");
  }

  @Test
  void releasingFallbackShotResumesWhicheverToggleIsStillOn() {
    assertEquals(OperatorIntent.BOUNCING, resolve(false, false, false, true, false),
        "button 1 released with toggle 5 on resumes bounce");
    assertEquals(OperatorIntent.INTAKING, resolve(false, false, false, false, true),
        "button 1 released with only toggle 6 on resumes intake");
    assertEquals(OperatorIntent.STOWED, resolve(false, false, false, false, false),
        "button 1 released with neither toggle on returns to idle/stowed");
  }

  @Test
  void turningOffBounceFallsThroughToIntakeToggle() {
    assertEquals(OperatorIntent.BOUNCING, resolve(false, false, false, true, true),
        "both toggles on bounces");
    assertEquals(OperatorIntent.INTAKING, resolve(false, false, false, false, true),
        "toggle 5 off while toggle 6 stays on falls through to normal persistent intake");
  }

  @Test
  void turningOffBounceWithNoIntakeToggleStows() {
    assertEquals(OperatorIntent.STOWED, resolve(false, false, false, false, false),
        "toggle 5 off with toggle 6 also off ends the bounce and returns to idle/stowed");
  }

  @Test
  void everyIntentIsReachable() {
    // Guards the ordering itself: if a higher-priority branch ever swallowed a lower one, that
    // intent would become unreachable from the physical board and this would catch it.
    assertEquals(OperatorIntent.EJECTING, resolve(false, false, true, false, false));
    assertEquals(OperatorIntent.VISION_SHOT, resolve(false, true, false, false, false));
    assertEquals(OperatorIntent.FALLBACK_SHOT, resolve(true, false, false, false, false));
    assertEquals(OperatorIntent.BOUNCING, resolve(false, false, false, true, false));
    assertEquals(OperatorIntent.INTAKING, resolve(false, false, false, false, true));
    assertEquals(OperatorIntent.STOWED, resolve(false, false, false, false, false));
  }
}
