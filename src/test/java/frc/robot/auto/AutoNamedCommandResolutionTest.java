// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.NamedCommands;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import frc.robot.Robot;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Guards a different bug class than {@link AutoCommandSafetyTest} (which checks a NamedCommand
 * used inside a "parallel" block is bounded): this checks that every "named" command referenced by
 * any {@code .auto} file -- anywhere, not just inside parallel blocks -- actually resolves to a
 * command registered via {@code NamedCommands.registerCommand} in {@code RobotContainer}'s
 * constructor. PathplannerLib's own {@code NamedCommands.getCommand()} does not throw for an
 * unregistered name: it prints a {@code DriverStation.reportWarning} and silently substitutes
 * {@code Commands.none()} (confirmed by reading the decompiled
 * {@code PathplannerLib-java-2026.1.2-sources.jar}) -- a typo'd or renamed reference in a
 * {@code .auto} file would compile fine, never fail {@code compileJava}, and just silently skip
 * that step of the auto with nothing but an easy-to-miss driver-station warning. Needs a real
 * {@link Robot} boot (unlike {@code AutoCommandSafetyTest}'s pure-JSON check) because
 * {@code NamedCommands}'s registry is only populated once {@code RobotContainer}'s constructor has
 * actually run.
 */
class AutoNamedCommandResolutionTest {
    private static final Path AUTOS_DIR = Paths.get("src", "main", "deploy", "pathplanner", "autos");

    private Robot robot;
    private Thread robotThread;

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
        DriverStationSim.resetData();
        SimHooks.pauseTiming();

        robot = new Robot();
        robotThread = new Thread(robot::startCompetition, getClass().getSimpleName() + "-competition");
        robotThread.setDaemon(true);
        robotThread.start();
        SimHooks.waitForProgramStart();
    }

    @AfterEach
    void tearDown() throws InterruptedException {
        try {
            robot.endCompetition();
            robotThread.join(1000);
            assertFalse(robotThread.isAlive(),
                    "startCompetition() loop should have exited within 1s of endCompetition()");
            robot.close();
        } finally {
            SimHooks.resumeTiming();
        }
    }

    @Test
    @Timeout(30)
    void everyAutoNamedCommandResolvesToARegisteredCommand() throws IOException {
        assertTrue(robotThread.isAlive(),
                "startCompetition() loop should still be running after waitForProgramStart()");

        // Sanity-checks the assertion mechanism itself, cheaply, without needing a synthetic .auto
        // fixture: proves NamedCommands.hasCommand() genuinely returns false for something never
        // registered, so a real typo/rename below would actually be caught, not pass vacuously.
        assertFalse(NamedCommands.hasCommand("DefinitelyNotRegisteredXYZ123"),
                "expected an unregistered name to report hasCommand()==false -- if this fails, the "
                        + "resolution check below cannot be trusted to catch anything");

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
            for (String name : collectNamedCommandNames(root.get("command"))) {
                if (!NamedCommands.hasCommand(name)) {
                    violations.add(autoFile.getFileName() + ": \"" + name + "\" is not registered via "
                            + "NamedCommands.registerCommand -- PathPlanner would silently substitute "
                            + "Commands.none() for this step at runtime");
                }
            }
        }

        if (!violations.isEmpty()) {
            fail("Found NamedCommand reference(s) that do not resolve to any command registered in "
                    + "RobotContainer's constructor:\n" + String.join("\n", violations));
        }
    }

    /** Recursively collects every "named" command's name anywhere in a PathPlanner auto command tree. */
    static Set<String> collectNamedCommandNames(JsonNode command) {
        Set<String> names = new HashSet<>();
        collectInto(command, names);
        return names;
    }

    private static void collectInto(JsonNode command, Set<String> names) {
        if (command == null || !command.has("type")) {
            return;
        }
        String type = command.get("type").asText();
        JsonNode data = command.get("data");
        if ("named".equals(type)) {
            names.add(data.get("name").asText());
            return;
        }
        if (data != null && data.has("commands")) {
            for (JsonNode child : data.get("commands")) {
                collectInto(child, names);
            }
        }
    }
}
