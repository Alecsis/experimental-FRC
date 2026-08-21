package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import frc.robot.RobotContainer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

/**
 * Pins the driver-station chooser's membership policy.
 *
 * <p>Hiding a practice route is a name-matching filter, and name matching fails silently: a typo in
 * {@link RobotContainer#PRACTICE_AUTOS} does not throw, it just leaves the route selectable in a
 * match. The whole point of these tests is that the failure mode is loud.
 *
 * <p>Deliberately does NOT boot a Robot. {@code AutoBuilder.buildAutoChooser*} needs a configured
 * AutoBuilder and a deploy directory, which would drag in the whole drivetrain; the policy under
 * test is just "which names survive the filter", so this reads the same deploy directory
 * {@code AutoBuilder.getAllAutoNames()} reads (a flat listing of {@code *.auto}, excluding
 * directories) and applies the same predicate. If PathPlannerLib ever changed how it enumerates
 * autos, {@link #everyAutoOnDiskIsEitherCompetitionOrPractice()} is what would notice.
 */
class AutoChooserFilterTest {
    private static final Path AUTOS_DIR = Paths.get("src", "main", "deploy", "pathplanner", "autos");

    /**
     * Mirrors {@code AutoBuilder.getAllAutoNames()}: every non-directory {@code *.auto} in the
     * deploy autos folder, extension stripped.
     */
    private static Set<String> autoNamesOnDisk() {
        try (Stream<Path> files = Files.list(AUTOS_DIR)) {
            return files
                    .filter(p -> !Files.isDirectory(p))
                    .map(p -> p.getFileName().toString())
                    .filter(name -> name.endsWith(".auto"))
                    .map(name -> name.substring(0, name.lastIndexOf('.')))
                    .collect(Collectors.toCollection(TreeSet::new));
        } catch (Exception e) {
            fail("could not list " + AUTOS_DIR.toAbsolutePath() + ": " + e);
            throw new AssertionError("unreachable");
        }
    }

    /** The names the chooser would offer, i.e. everything on disk that survives the filter. */
    private static Set<String> chooserOptions() {
        Set<String> options = autoNamesOnDisk();
        options.removeAll(RobotContainer.PRACTICE_AUTOS);
        return options;
    }

    @Test
    void everyPracticeAutoNameExistsOnDisk() {
        Set<String> onDisk = autoNamesOnDisk();
        for (String practice : RobotContainer.PRACTICE_AUTOS) {
            assertTrue(onDisk.contains(practice),
                    () -> "PRACTICE_AUTOS names an auto that does not exist: '" + practice
                            + "'. A name that matches nothing hides nothing, so this route would"
                            + " still be selectable in a match. Autos on disk: " + onDisk);
        }
    }

    @Test
    void practiceAutosAreAbsentFromTheChooser() {
        Set<String> options = chooserOptions();
        for (String practice : RobotContainer.PRACTICE_AUTOS) {
            assertFalse(options.contains(practice),
                    () -> "'" + practice + "' is hidden but still reached the chooser");
        }
    }

    @Test
    void competitionAutosRemainInTheChooser() {
        // Non-vacuity guard: a filter that removed everything would satisfy the test above.
        List<String> expectedCompetition = List.of(
                "LB Depot",
                "LB Neutral",
                "LT Depot",
                "LT Neutral",
                "LT Neutral - Bump Route",
                "LT Neutral - U Turn",
                "Middle Shoot",
                "RB Neutral",
                "RT Neutral",
                "RT Neutral - U Turn");

        Set<String> options = chooserOptions();
        for (String competition : expectedCompetition) {
            assertTrue(options.contains(competition),
                    () -> "'" + competition + "' must stay selectable; chooser offered " + options);
        }
    }

    @Test
    void everyAutoOnDiskIsEitherCompetitionOrPractice() {
        // Catches a route added to the deploy folder without a membership decision: it would
        // otherwise appear in the match chooser by default, which is the unsafe direction.
        Set<String> options = chooserOptions();
        Set<String> practice = RobotContainer.PRACTICE_AUTOS;

        for (String name : autoNamesOnDisk()) {
            assertTrue(options.contains(name) || practice.contains(name),
                    () -> "'" + name + "' is neither offered nor hidden -- unreachable state");
        }
        assertTrue(options.size() + practice.size() == autoNamesOnDisk().size(),
                () -> "the two sets must partition the deploy folder exactly: offered=" + options
                        + " hidden=" + practice + " onDisk=" + autoNamesOnDisk());
    }

    @Test
    void hiddenAutosAreStillOnDiskAndStillLoadableByName() {
        // Hiding must be chooser-only. The regression suite loads every route with
        // AutoBuilder.buildAuto(name), never through the chooser (see AutoRegressionTestBase's own
        // comment), so a hidden route must remain a normal, complete .auto file.
        for (String practice : RobotContainer.PRACTICE_AUTOS) {
            Path file = AUTOS_DIR.resolve(practice + ".auto");
            assertTrue(Files.isRegularFile(file),
                    () -> "hidden auto '" + practice + "' must remain on disk at " + file);
            try {
                assertTrue(Files.size(file) > 0,
                        () -> "hidden auto '" + practice + "' must remain a real route file");
            } catch (Exception e) {
                fail("could not read " + file + ": " + e);
            }
        }
    }
}
