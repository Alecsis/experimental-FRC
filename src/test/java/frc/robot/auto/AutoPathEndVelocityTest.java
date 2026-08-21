// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

/**
 * Static guard against uncommanded drift between autonomous path segments.
 *
 * <p>PathplannerLib's {@code FollowPathCommand.end(boolean)} deliberately does NOT zero the
 * drivetrain when the finished path declares a non-zero {@code goalEndState.velocity} -- its own
 * comment says "Only output 0 speeds when ending a path that is supposed to stop, this allows
 * interrupting the command to smoothly transition into some auto-alignment routine". That contract
 * is correct only when the NEXT path starts on the very next scheduler cycle.
 *
 * <p>A path placed inside a PathPlanner "parallel" block breaks that assumption: the block does not
 * advance until EVERY branch finishes, so a longer-running sibling (e.g. a timed intake sequence)
 * leaves the drivetrain rolling, uncommanded, at the path's declared end velocity for the remainder
 * of the block.
 *
 * <p>Measured on the "LT Neutral" auto 2026-07-29, before this guard existed: "Left Trench Start"
 * (goalEndState.velocity 0.85 m/s) finished at t=2.04s just 0.060 m from its designed endpoint --
 * excellent tracking -- and was then left rolling at 0.86 m/s for 2.94 s while its parallel sibling
 * "Intake Start Sequence" ran out its 5.0 s bound. The robot drifted 1.850 m, so the next path
 * ("Left Trench Neutral") began 1.909 m away from the robot, spiking lateral tracking error to
 * 1.608 m. This is NOT simulation-specific -- it follows entirely from PathplannerLib's scheduling
 * plus the .auto structure, so it happens identically on real hardware.
 *
 * <p>Pure JSON parsing -- no CommandScheduler, no Robot, no sim, no forkEvery/singleton concerns.
 * Sibling of {@link AutoCommandSafetyTest}, which guards a different parallel-block bug class
 * (unbounded NamedCommands stalling the auto forever).
 */
class AutoPathEndVelocityTest {
  private static final Path AUTOS_DIR = Paths.get("src", "main", "deploy", "pathplanner", "autos");
  private static final Path PATHS_DIR = Paths.get("src", "main", "deploy", "pathplanner", "paths");

  /**
   * Matches PathplannerLib's own {@code FollowPathCommand.end()} cutoff: it zeroes the drivetrain
   * only when {@code goalEndState.velocityMPS() < 0.1}.
   */
  private static final double kStopsAtEndVelocityMps = 0.1;

  @Test
  void everyPathInsideAParallelBlockEndsAtRest() throws IOException {
    assertTrue(Files.isDirectory(AUTOS_DIR), "expected " + AUTOS_DIR + " to exist");
    assertTrue(Files.isDirectory(PATHS_DIR), "expected " + PATHS_DIR + " to exist");
    ObjectMapper mapper = new ObjectMapper();
    List<String> violations = new ArrayList<>();

    List<Path> autoFiles;
    try (Stream<Path> files = Files.list(AUTOS_DIR)) {
      autoFiles = files.filter(p -> p.toString().endsWith(".auto")).toList();
    }
    assertFalse(autoFiles.isEmpty(),
        "found zero .auto files in " + AUTOS_DIR + " -- did the deploy layout change? "
            + "(this test performs zero checks if no autos are found, silently defeating its "
            + "purpose as a regression guard)");

    for (Path autoFile : autoFiles) {
      JsonNode root = mapper.readTree(autoFile.toFile());
      walk(autoFile.getFileName().toString(), root.get("command"), false, violations,
          AutoPathEndVelocityTest::goalEndVelocityOf);
    }

    if (!violations.isEmpty()) {
      fail("Found path(s) inside a \"parallel\" block declaring a non-zero goalEndState.velocity. "
          + "PathplannerLib will NOT stop the drivetrain when such a path ends, and the parallel "
          + "block does not advance to the next path until its slowest branch finishes -- so the "
          + "robot coasts uncommanded across that gap and the next path starts from the wrong "
          + "place:\n" + String.join("\n", violations));
    }
  }

  @Test
  void pathNestedInSequentialInsideParallelIsFlagged() throws IOException {
    // Non-vacuity + traversal blind-spot guard: a "path" nested inside a "sequential" branch of a
    // "parallel" block must still be flagged, even though it is not an immediate child of the
    // "parallel" node itself.
    String json = "{\"command\": {\"type\": \"parallel\", \"data\": {\"commands\": ["
        + "{\"type\": \"named\", \"data\": {\"name\": \"SomeTimedThing\"}},"
        + "{\"type\": \"sequential\", \"data\": {\"commands\": ["
        + "{\"type\": \"path\", \"data\": {\"pathName\": \"RollsOnForever\"}}"
        + "]}}"
        + "]}}}";
    JsonNode root = new ObjectMapper().readTree(json);
    List<String> violations = new ArrayList<>();

    walk("synthetic.auto", root.get("command"), false, violations, name -> 0.85);

    assertFalse(violations.isEmpty(),
        "expected a violation for \"RollsOnForever\" nested inside a sequential branch of a "
            + "parallel block, but the traversal found none -- the blind spot has regressed");
  }

  @Test
  void pathOutsideAnyParallelBlockIsNotFlagged() throws IOException {
    // A path in a plain sequential chain is exactly the case PathplannerLib's non-zero end
    // velocity is designed for -- the next path starts on the next cycle, so carrying momentum is
    // correct and must NOT be reported.
    String json = "{\"command\": {\"type\": \"sequential\", \"data\": {\"commands\": ["
        + "{\"type\": \"path\", \"data\": {\"pathName\": \"ChainsStraightIntoTheNextOne\"}},"
        + "{\"type\": \"path\", \"data\": {\"pathName\": \"TheNextOne\"}}"
        + "]}}}";
    JsonNode root = new ObjectMapper().readTree(json);
    List<String> violations = new ArrayList<>();

    walk("synthetic.auto", root.get("command"), false, violations, name -> 0.85);

    assertTrue(violations.isEmpty(),
        "a path in a plain sequential chain carries momentum into the next path by design and "
            + "must not be flagged, but got: " + violations);
  }

  /** Reads goalEndState.velocity out of the named .path file, or 0 if it has none. */
  private static double goalEndVelocityOf(String pathName) {
    Path pathFile = PATHS_DIR.resolve(pathName + ".path");
    if (!Files.isRegularFile(pathFile)) {
      return 0.0;
    }
    try {
      JsonNode goalEndState = new ObjectMapper().readTree(pathFile.toFile()).get("goalEndState");
      if (goalEndState == null || !goalEndState.has("velocity")) {
        return 0.0;
      }
      return goalEndState.get("velocity").asDouble();
    } catch (IOException e) {
      throw new IllegalStateException("could not read " + pathFile, e);
    }
  }

  private static void walk(String fileName, JsonNode command, boolean insideParallel,
      List<String> violations, java.util.function.ToDoubleFunction<String> endVelocityLookup) {
    if (command == null || !command.has("type")) {
      return;
    }
    String type = command.get("type").asText();
    JsonNode data = command.get("data");
    if ("path".equals(type)) {
      if (insideParallel) {
        String pathName = data.get("pathName").asText();
        double endVelocity = endVelocityLookup.applyAsDouble(pathName);
        if (endVelocity >= kStopsAtEndVelocityMps) {
          violations.add(fileName + ": \"" + pathName + "\" ends at " + endVelocity
              + " m/s inside a parallel block -- PathplannerLib leaves the drivetrain rolling "
              + "at that speed until the block's slowest branch finishes");
        }
      }
      return;
    }
    if (data != null && data.has("commands")) {
      boolean childInsideParallel = insideParallel || "parallel".equals(type);
      for (JsonNode child : data.get("commands")) {
        walk(fileName, child, childInsideParallel, violations, endVelocityLookup);
      }
    }
  }
}
