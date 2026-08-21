package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Pins {@link AutoRegressionTestBase#completedWithinCap}, the rule that decides a run's
 * {@code completed} field.
 *
 * <p><b>The defect this replaced.</b> "LT Neutral"'s auto command ends 15.02-15.04 s into
 * autonomous against a 15.0 s cap, and its {@code completed} field flipped in 2 of 6 measured
 * runs. Two independent causes, both removed:
 *
 * <ol>
 *   <li>The command's timing ran on the WPILib simulated clock from the tick that initialized it,
 *       while the cap ran on {@code System.nanoTime()} captured later, after {@code resumeTiming()}.
 *       Elapsed therefore under-counted the command's own autonomous time by tens of milliseconds
 *       -- the exact scale of the flip.
 *   <li>The cap was checked in the loop's {@code while} condition, before the sleep. A command
 *       found finished at a sample already past the cap was still credited {@code completed=true}.
 * </ol>
 *
 * <p>These are pure-function tests on purpose. The rule now consumes only two simulated timestamps,
 * so it can be exercised exhaustively in milliseconds instead of at 15 s per real run, and
 * repeatability is a property of the function rather than a hope about the host.
 */
class AutoRegressionBoundaryTest {
    private static final double kCap = AutoRegressionTolerances.kMaxRuntimeSeconds;
    /** One robot period -- the granularity a real command can land on. */
    private static final double kTick = AutoRegressionTolerances.kStepSeconds;
    /** Arbitrary non-zero origin: the rule must be origin-independent, not assume runs start at 0. */
    private static final double kStart = 1234.5;

    @Test
    void anAutoFinishingClearlyBeforeTheCapIsCompleted() {
        assertTrue(AutoRegressionTestBase.completedWithinCap(kStart + 12.44, kStart, kCap),
                "a route that ends 2.5 s inside the window completed");
    }

    @Test
    void anAutoStillRunningAtTheCapIsNotCompleted() {
        // The harness stops watching at the cap, so no end timestamp was ever recorded.
        assertFalse(AutoRegressionTestBase.completedWithinCap(null, kStart, kCap),
                "a command that never ended cannot be completed");
    }

    @Test
    void anAutoFinishingAfterTheCapIsNotCompletedEvenThoughItDidEnd() {
        // Precisely the LT Neutral case: it DOES finish, just not inside autonomous. Under the old
        // ordering this was the flip -- the harness saw "not scheduled" and credited it.
        assertFalse(AutoRegressionTestBase.completedWithinCap(kStart + 15.02, kStart, kCap),
                "ending 0.02 s past the cap is not completing within the cap");
        assertFalse(AutoRegressionTestBase.completedWithinCap(kStart + 15.04, kStart, kCap),
                "nor is ending 0.04 s past it");
    }

    @Test
    void theBoundaryItselfIsInclusiveAndOneTickEitherSideIsUnambiguous() {
        assertTrue(AutoRegressionTestBase.completedWithinCap(kStart + kCap, kStart, kCap),
                "ending exactly at the cap counts as inside it");
        assertTrue(AutoRegressionTestBase.completedWithinCap(kStart + kCap - kTick, kStart, kCap),
                "one tick early is inside");
        assertFalse(AutoRegressionTestBase.completedWithinCap(kStart + kCap + kTick, kStart, kCap),
                "one tick late is outside");
    }

    @Test
    void theVerdictIsRepeatableForAnAutoLandingExactlyOnTheBoundary() {
        // Requirement: a route landing on the boundary must give the SAME answer every time. The
        // rule is a pure function of two timestamps, so this is exhaustive rather than statistical.
        boolean first = AutoRegressionTestBase.completedWithinCap(kStart + kCap, kStart, kCap);
        for (int i = 0; i < 10_000; i++) {
            assertEquals(first, AutoRegressionTestBase.completedWithinCap(kStart + kCap, kStart, kCap),
                    "the boundary verdict must not vary between evaluations");
        }
        assertTrue(first);
    }

    @Test
    void theVerdictDoesNotDependOnTheClockOrigin() {
        // Guards against reintroducing an origin mismatch: the same *duration* must classify the
        // same way no matter where the run sits on the timeline.
        for (double origin : new double[] {0.0, 1.0, 1234.5, 86_400.0, 1.0e6}) {
            assertTrue(AutoRegressionTestBase.completedWithinCap(origin + 14.5, origin, kCap),
                    "14.5 s is inside the cap at origin " + origin);
            assertFalse(AutoRegressionTestBase.completedWithinCap(origin + 15.02, origin, kCap),
                    "15.02 s is outside the cap at origin " + origin);
        }
    }

    @Test
    void aLongPracticeRouteStaysNotCompleted() {
        // "Left Double Swipe Bump" runs ~27 s; under the 15 s cap the harness stops watching long
        // before it ends, so there is no end timestamp at all.
        assertFalse(AutoRegressionTestBase.completedWithinCap(null, kStart, kCap));
        // And even if it were watched to the end, 27 s is not within 15 s.
        assertFalse(AutoRegressionTestBase.completedWithinCap(kStart + 27.24, kStart, kCap));
    }

    @Test
    void aShortRouteStaysCompleted() {
        // "Middle Shoot" measured 9.125 s, "RT Neutral" 14.469 s -- both comfortably inside.
        assertTrue(AutoRegressionTestBase.completedWithinCap(kStart + 9.125, kStart, kCap));
        assertTrue(AutoRegressionTestBase.completedWithinCap(kStart + 14.469, kStart, kCap));
    }

    /**
     * Negative control. Without it, every assertion above could pass for a rule that is simply
     * "return false a lot". This transcribes the OLD logic -- a cap measured from a later origin,
     * and completion credited from whichever sample first observed the command gone, without
     * re-checking the cap -- and shows it classifies the LT Neutral case as completed while the new
     * rule does not. The defect was real, and this is the difference that removes it.
     */
    @Test
    void negativeControlTheOldWallClockOrderingDoesFlipTheSameCase() {
        // LT Neutral, measured: command ends 15.02 s after the tick that initialized it.
        double commandEndOffset = 15.02;
        // The old cap origin was captured after stepTiming()+resumeTiming(), i.e. LATER than the
        // command's own start, so its elapsed under-counted by that much. 40 ms is the measured
        // scale.
        double originLagSeconds = 0.04;

        boolean oldVerdict = oldWallClockRule(commandEndOffset, originLagSeconds, kCap);
        boolean newVerdict =
                AutoRegressionTestBase.completedWithinCap(kStart + commandEndOffset, kStart, kCap);

        assertTrue(oldVerdict,
                "negative control is vacuous unless the OLD rule really does credit a command that"
                        + " ended after the cap");
        assertFalse(newVerdict, "the new rule must not");
        assertEquals(!oldVerdict, newVerdict, "the two rules must disagree on exactly this case");
    }

    /**
     * The pre-fix rule, transcribed. Samples every tick; its elapsed is short by
     * {@code originLagSeconds}; it credits completion at the first sample where the command is gone
     * without re-testing the cap at that sample.
     */
    private static boolean oldWallClockRule(double commandEndOffsetSeconds, double originLagSeconds,
            double capSeconds) {
        double elapsed = 0.0;
        while (elapsed < capSeconds) {
            elapsed += kTick;
            double trueAutonomousTime = elapsed + originLagSeconds;
            if (trueAutonomousTime >= commandEndOffsetSeconds) {
                return true; // "not scheduled any more" -> completed, cap never re-checked
            }
        }
        return false;
    }
}
