// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.auto;

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
 * Static guard against the exact bug class that hung every in-scope auto until fixed (see
 * Superstructure's intakeSequence javadoc / docs/claudex/history.md's thirteenth-session entry):
 * a hold-forever Command registered as a NamedCommand and used inside a PathPlanner "parallel"
 * block (ParallelCommandGroup, requires every branch to finish) silently stalls the auto forever.
 * Pure JSON parsing -- no CommandScheduler, no Robot, no sim, no forkEvery/singleton concerns.
 */
class AutoCommandSafetyTest {
  private static final Path AUTOS_DIR = Paths.get("src", "main", "deploy", "pathplanner", "autos");

  @Test
  void everyParallelBlockNamedCommandIsBounded() throws IOException {
    assertTrue(Files.isDirectory(AUTOS_DIR), "expected " + AUTOS_DIR + " to exist");
    ObjectMapper mapper = new ObjectMapper();
    List<String> violations = new ArrayList<>();

    try (Stream<Path> files = Files.list(AUTOS_DIR)) {
      for (Path autoFile : files.filter(p -> p.toString().endsWith(".auto")).toList()) {
        JsonNode root = mapper.readTree(autoFile.toFile());
        collectParallelBlockViolations(autoFile.getFileName().toString(), root.get("command"),
            violations);
      }
    }

    if (!violations.isEmpty()) {
      fail("Found NamedCommand(s) inside a \"parallel\" block that are not in "
          + "RobotContainer.BOUNDED_NAMED_COMMANDS -- an unbounded (hold-forever) Command here "
          + "will silently stall its auto forever, exactly like the original intakeCmd() bug:\n"
          + String.join("\n", violations));
    }
  }

  private static void collectParallelBlockViolations(String fileName, JsonNode command,
      List<String> violations) {
    if (command == null || !command.has("type")) {
      return;
    }
    String type = command.get("type").asText();
    JsonNode data = command.get("data");
    if ("parallel".equals(type) && data != null && data.has("commands")) {
      for (JsonNode child : data.get("commands")) {
        if ("named".equals(child.get("type").asText())) {
          String name = child.get("data").get("name").asText();
          if (!RobotContainer.BOUNDED_NAMED_COMMANDS.contains(name)) {
            violations.add(fileName + ": \"" + name + "\" used inside a parallel block but not "
                + "in BOUNDED_NAMED_COMMANDS");
          }
        }
      }
    }
    if (data != null && data.has("commands")) {
      for (JsonNode child : data.get("commands")) {
        collectParallelBlockViolations(fileName, child, violations);
      }
    }
  }
}
