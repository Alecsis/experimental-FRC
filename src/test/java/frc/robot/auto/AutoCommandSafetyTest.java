// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.RobotContainer;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

/**
 * Static guard against a specific bug class: a hold-forever/unbounded Command registered as a
 * NamedCommand and used inside a PathPlanner "parallel" block (ParallelCommandGroup, requires
 * every branch to finish) silently stalls the auto forever. This is the same bug class as the
 * thirteenth-session {@code intakeCmd()} fix (see {@code Superstructure.intakeSequence}'s javadoc)
 * -- this test guards the whole class structurally instead of relying on catching it by hand
 * per-auto. Pure JSON parsing -- no CommandScheduler, no Robot, no sim, no forkEvery/singleton
 * concerns.
 */
class AutoCommandSafetyTest {
  private static final Path AUTOS_DIR = Paths.get("src", "main", "deploy", "pathplanner", "autos");

  @Test
  void everyParallelBlockNamedCommandIsBounded() throws IOException {
    assertTrue(Files.isDirectory(AUTOS_DIR), "expected " + AUTOS_DIR + " to exist");
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
      walk(autoFile.getFileName().toString(), root.get("command"), false, violations);
    }

    if (!violations.isEmpty()) {
      fail("Found NamedCommand(s) inside a \"parallel\" block that are not in "
          + "RobotContainer.BOUNDED_NAMED_COMMANDS -- an unbounded (hold-forever) Command here "
          + "will silently stall its auto forever, exactly like the original intakeCmd() bug:\n"
          + String.join("\n", violations));
    }
  }

  @Test
  void namedCommandNestedInSequentialInsideParallelIsFlagged() throws IOException {
    // Regression case for the traversal blind spot: a "named" command nested inside a
    // "sequential" branch of a "parallel" block must still be flagged, even though it is not
    // an immediate child of the "parallel" node itself.
    String json = "{\"command\": {\"type\": \"parallel\", \"data\": {\"commands\": ["
        + "{\"type\": \"path\", \"data\": {\"pathName\": \"Foo\"}},"
        + "{\"type\": \"sequential\", \"data\": {\"commands\": ["
        + "{\"type\": \"named\", \"data\": {\"name\": \"UnboundedHoldForever\"}},"
        + "{\"type\": \"wait\", \"data\": {\"waitTime\": 1.0}}"
        + "]}}"
        + "]}}}";
    ObjectMapper mapper = new ObjectMapper();
    JsonNode root = mapper.readTree(json);
    List<String> violations = new ArrayList<>();

    walk("synthetic.auto", root.get("command"), false, violations);

    assertFalse(violations.isEmpty(),
        "expected a violation for \"UnboundedHoldForever\" nested inside a sequential branch of "
            + "a parallel block, but the traversal found none -- the blind spot has regressed");
  }

  private static void walk(String fileName, JsonNode command, boolean insideParallel,
      List<String> violations) {
    if (command == null || !command.has("type")) {
      return;
    }
    String type = command.get("type").asText();
    JsonNode data = command.get("data");
    if ("named".equals(type)) {
      if (insideParallel) {
        String name = data.get("name").asText();
        if (!RobotContainer.BOUNDED_NAMED_COMMANDS.contains(name)) {
          violations.add(fileName + ": \"" + name + "\" used inside a parallel block but not "
              + "in BOUNDED_NAMED_COMMANDS");
        }
      }
      return;
    }
    if (data != null && data.has("commands")) {
      boolean childInsideParallel = insideParallel || "parallel".equals(type);
      for (JsonNode child : data.get("commands")) {
        walk(fileName, child, childInsideParallel, violations);
      }
    }
  }
}
