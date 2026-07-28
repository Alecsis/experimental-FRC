// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Guards a real observability gap found during the autonomous reliability audit: neither
 * {@code Robot.autonomousInit()} nor {@code teleopInit()} ever recorded whether the scheduled
 * autonomous command finished on its own or got cut off (teleop transition, a disable-triggered
 * cancel, anything else) -- a wpilog from a real match could not distinguish "auto completed" from
 * "auto got interrupted" after the fact. {@link Robot#wrapAutonomousForTelemetry(Command)} is the
 * fix under test here, exercised directly (bypassing the auto chooser/NetworkTables, which
 * {@code AutoRegressionTestBase} already documents defaults to a trivial {@code Commands.none()})
 * so both the natural-finish and interrupted-cancel paths can be driven deterministically in one
 * test method -- see {@code CommandSwerveDrivetrainSysIdReverseSimWorkflowTest}'s discovery this
 * same day that a second {@code @Test} method in one class throws from AdvantageKit's
 * {@code Logger.start()} JVM singleton (build.gradle's {@code forkEvery = 1} forks per class, not
 * per method); keeping both scenarios in a single test method avoids a second class entirely.
 */
class RobotAutoTerminationTelemetryTest {
    private static final String kRunningEntryName = "/RealOutputs/Auto/Running";
    private static final String kEndedInterruptedEntryName = "/RealOutputs/Auto/EndedInterrupted";
    private static final String kBooleanType = "boolean";

    private Robot robot;
    private Thread robotThread;
    private boolean toreDown;
    private boolean timingResumed;

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
        toreDown = false;
        timingResumed = false;
    }

    @AfterEach
    void tearDown() throws InterruptedException {
        if (!toreDown) {
            finishCompetitionAndClose();
        }
        if (!timingResumed) {
            SimHooks.resumeTiming();
        }
    }

    private void finishCompetitionAndClose() throws InterruptedException {
        robot.endCompetition();
        robotThread.join(1000);
        assertFalse(robotThread.isAlive(),
                "startCompetition() loop should have exited within 1s of endCompetition()");
        robot.close();
        toreDown = true;
    }

    @Test
    @Timeout(30)
    void telemetryDistinguishesNaturalFinishFromInterruption() throws IOException, InterruptedException {
        assertTrue(robotThread.isAlive(),
                "startCompetition() loop should still be running after waitForProgramStart()");

        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        // Phase 1: a short command that finishes on its own -- EndedInterrupted must read false.
        SimHooks.pauseTiming();
        Command naturallyFinishing = Robot.wrapAutonomousForTelemetry(new WaitCommand(0.2));
        CommandScheduler.getInstance().schedule(naturallyFinishing);
        // Same paused-schedule-then-single-step pattern as AutoRegressionTestBase -- avoids the
        // documented cross-thread scheduling race between this test thread's schedule() call and
        // the robot thread's concurrently-running periodic loop.
        SimHooks.stepTiming(0.02);
        SimHooks.resumeTiming();
        timingResumed = true;

        Thread.sleep(600);
        assertFalse(CommandScheduler.getInstance().isScheduled(naturallyFinishing),
                "the wrapped 0.2s WaitCommand should have finished naturally by now");

        // Phase 2: a long command, cancelled mid-flight -- EndedInterrupted must read true.
        Command cancelled = Robot.wrapAutonomousForTelemetry(new WaitCommand(10));
        CommandScheduler.getInstance().schedule(cancelled);
        Thread.sleep(200);
        assertTrue(CommandScheduler.getInstance().isScheduled(cancelled),
                "the wrapped 10s WaitCommand should still be running before it is cancelled");
        CommandScheduler.getInstance().cancel(cancelled);
        Thread.sleep(200);

        finishCompetitionAndClose();

        Path wpilog = findNewestWpilog();
        assertTrue(wpilog != null, "No .wpilog found in logs/ after the run -- SIM mode should have written one.");

        // wrapAutonomousForTelemetry() logs Auto/Running on every schedule/end, so it shows every
        // true/false transition: index 0 is phase 1's schedule-time write, index 1 is phase 1's
        // natural end, index 2 is phase 2's schedule-time write, index 3 is phase 2's cancelled end.
        // Auto/EndedInterrupted is write-on-change in the underlying log (empirically confirmed:
        // repeated same-value writes collapse to a single sample), so its deduped series only ever
        // shows [false, true] here: false from the very start (nothing has been interrupted yet)
        // flipping to true exactly once, at phase 2's cancel -- proving it never spuriously turned
        // true during phase 1's natural finish, and did turn true once phase 2 was cancelled.
        List<Boolean> running = readBooleanSeries(wpilog, kRunningEntryName);
        List<Boolean> endedInterrupted = readBooleanSeries(wpilog, kEndedInterruptedEntryName);

        assertEquals(List.of(true, false, true, false), running,
                kRunningEntryName + ": expected true,false,true,false (schedule/end x2 phases), got " + running);
        assertEquals(List.of(false, true), endedInterrupted,
                kEndedInterruptedEntryName + ": expected false (never interrupted through phase 1's natural "
                        + "finish) then true exactly once (phase 2's cancel), got " + endedInterrupted);
    }

    /**
     * Reads every sample of a single boolean AdvantageKit entry from a wpilog, using WPILib's own
     * {@link DataLogReader}/{@link DataLogRecord} -- same library {@code WpilogTrajectoryErrorReader}
     * and {@code CommandSwerveDrivetrainSysIdSimWorkflowTest} already use.
     */
    private static List<Boolean> readBooleanSeries(Path wpilogFile, String entryName) throws IOException {
        DataLogReader reader = new DataLogReader(wpilogFile.toString());
        if (!reader.isValid()) {
            throw new IOException("Not a valid WPILOG file: " + wpilogFile);
        }

        Map<Integer, String> activeNames = new HashMap<>();
        Map<Integer, String> activeTypes = new HashMap<>();
        List<Boolean> samples = new ArrayList<>();

        for (DataLogRecord record : reader) {
            if (record.isStart()) {
                DataLogRecord.StartRecordData start = record.getStartData();
                activeNames.put(start.entry, start.name);
                activeTypes.put(start.entry, start.type);
                continue;
            }
            if (record.isFinish() || record.isSetMetadata()) {
                continue;
            }

            String name = activeNames.get(record.getEntry());
            String type = activeTypes.get(record.getEntry());
            if (!entryName.equals(name) || !kBooleanType.equals(type)) {
                continue;
            }

            samples.add(record.getBoolean());
        }

        return samples;
    }

    /** Newest-by-mtime, matching AutoRegressionTestBase's own findNewestWpilog() convention. */
    private static Path findNewestWpilog() throws IOException {
        Path logsDir = Paths.get("logs");
        if (!Files.isDirectory(logsDir)) {
            return null;
        }
        try (Stream<Path> files = Files.list(logsDir)) {
            return files.filter(p -> p.toString().endsWith(".wpilog"))
                    .max(Comparator.comparingLong(p -> p.toFile().lastModified()))
                    .orElse(null);
        }
    }
}
