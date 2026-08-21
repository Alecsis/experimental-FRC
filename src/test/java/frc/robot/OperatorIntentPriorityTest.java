package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.superstructure.Superstructure;
import org.junit.jupiter.api.Test;

/** Pure arbitration table: physical toggle truth is evaluated every scheduler cycle. */
class OperatorIntentPriorityTest {
  @Test
  void resolvesMomentaryOverridesThenBounceThenPersistentIntake() {
    assertEquals(Superstructure.OperatorIntent.STOWED,
        Superstructure.resolveOperatorIntent(false, false, false, false, false));
    assertEquals(Superstructure.OperatorIntent.INTAKING,
        Superstructure.resolveOperatorIntent(false, false, false, false, true));
    assertEquals(Superstructure.OperatorIntent.BOUNCING,
        Superstructure.resolveOperatorIntent(false, false, false, true, false));
    assertEquals(Superstructure.OperatorIntent.BOUNCING,
        Superstructure.resolveOperatorIntent(false, false, false, true, true));
    assertEquals(Superstructure.OperatorIntent.FALLBACK_SHOT,
        Superstructure.resolveOperatorIntent(true, false, false, true, true));
    assertEquals(Superstructure.OperatorIntent.VISION_SHOT,
        Superstructure.resolveOperatorIntent(true, true, false, true, true));
    assertEquals(Superstructure.OperatorIntent.EJECTING,
        Superstructure.resolveOperatorIntent(true, true, true, true, true));
  }
}
