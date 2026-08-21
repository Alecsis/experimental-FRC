package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.opentest4j.AssertionFailedError;

/**
 * Pins {@link AutoRegressionTestBase#assertMatchesGolden}'s identity guard, and specifically that
 * it runs BEFORE the behavioural comparisons.
 *
 * <p>Ordering is the whole point, so it is asserted rather than read off the source. A golden whose
 * {@code autoName} belongs to a different route can still satisfy every behavioural tolerance --
 * that is exactly how four renamed "* Recollect *" routes passed on 2026-08-20 against goldens
 * recorded 0.200 m and 180.0 deg away from the route being graded. If the identity check were
 * placed after the behavioural ones, or removed, {@link #identityIsCheckedBeforeBehaviour()} fails.
 *
 * <p>No Robot, no HAL, no MapleSim: this drives the extracted static comparison directly, which is
 * why it costs milliseconds instead of the ~17 s a real regression run takes.
 */
class AutoRegressionGoldenIdentityTest {

    private static AutoRegressionGolden golden(String autoName, boolean completed, double runtime,
            double maxLat, double maxLon) {
        AutoRegressionGolden g = new AutoRegressionGolden();
        g.autoName = autoName;
        g.recordedAtGitSha = "0000000";
        g.completed = completed;
        g.runtimeSeconds = runtime;
        g.maxLateralErrorMeters = maxLat;
        g.maxLongitudinalErrorMeters = maxLon;
        return g;
    }

    @Test
    void matchingIdentityAndWithinToleranceProceedsNormally() {
        AutoRegressionGolden g = golden("LT Neutral", false, 15.0, 0.09, 0.24);
        AutoRegressionGolden actual = golden("LT Neutral", false, 15.01, 0.11, 0.28);
        assertDoesNotThrow(() -> AutoRegressionTestBase.assertMatchesGolden(
                "LT Neutral", "LT_Neutral.json", g, actual));
    }

    @Test
    void mismatchedIdentityFailsWithTheGoldenFilenameAndBothNames() {
        // Behaviourally identical, so ONLY the identity guard can reject this pairing.
        AutoRegressionGolden g = golden("LT Neutral - Bump Recollect", false, 15.0, 0.09, 0.25);
        AutoRegressionGolden actual = golden("LT Neutral Recollect - Bump", false, 15.0, 0.09, 0.25);

        AssertionFailedError e = assertThrows(AssertionFailedError.class,
                () -> AutoRegressionTestBase.assertMatchesGolden(
                        "LT Neutral Recollect - Bump", "LT_Neutral_Bump_Recollect.json", g, actual));

        String m = e.getMessage();
        for (String required : List.of(
                "LT_Neutral_Bump_Recollect.json",
                "LT Neutral - Bump Recollect",
                "LT Neutral Recollect - Bump",
                "does not describe this route")) {
            assertTrue(m.contains(required),
                    () -> "identity failure message must name " + required + ", but was: " + m);
        }
    }

    @Test
    void identityIsCheckedBeforeBehaviour() {
        // Mismatched identity AND every behavioural comparison blown wide open. Whichever check
        // runs first owns the message; it must be the identity one, because the behavioural
        // numbers are not evidence of anything once the golden belongs to another route.
        AutoRegressionGolden g = golden("RT Neutral - Trench Recollect", false, 15.0, 0.08, 0.21);
        AutoRegressionGolden actual = golden("RT Neutral Recollect - Trench", true, 2.0, 9.0, 9.0);

        AssertionFailedError e = assertThrows(AssertionFailedError.class,
                () -> AutoRegressionTestBase.assertMatchesGolden(
                        "RT Neutral Recollect - Trench", "RT_Neutral_Trench_Recollect.json", g, actual));

        String m = e.getMessage();
        assertTrue(m.contains("does not describe this route"),
                () -> "identity must be rejected before completed/runtime/tracking are graded, but"
                        + " the failure was: " + m);
        assertTrue(!m.contains("completed changed"),
                () -> "the completed-changed comparison ran despite a mismatched golden: " + m);
        assertTrue(!m.contains("runtimeSeconds"),
                () -> "the runtime comparison ran despite a mismatched golden: " + m);
    }

    @Test
    void matchingIdentityStillLetsRealBehaviouralFailuresThrough() {
        // The guard must not become a shortcut that swallows genuine regressions.
        AutoRegressionGolden g = golden("RB Neutral", false, 15.0, 0.045, 0.112);
        AutoRegressionGolden actual = golden("RB Neutral", true, 13.0, 0.045, 0.112);

        AssertionFailedError e = assertThrows(AssertionFailedError.class,
                () -> AutoRegressionTestBase.assertMatchesGolden(
                        "RB Neutral", "RB_Neutral.json", g, actual));
        assertTrue(e.getMessage().contains("completed changed"),
                () -> "a real behavioural regression should still be reported, but was: "
                        + e.getMessage());
    }
}
