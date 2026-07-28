// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Phase 1 Task 1.1 of docs/superpowers/plans/2026-07-28-autonomous-velocity-migration.md: a
 * sim-only integration check that the already-wired SysId command lifecycle, AdvantageKit
 * telemetry, and wpilog data capture all actually work end-to-end.
 *
 * <p><b>THIS IS NOT HARDWARE CHARACTERIZATION.</b> No kS/kV/kA value is computed here; only
 * pipeline behavior is checked (command runs without error, telemetry changes during the run, the
 * resulting wpilog is readable by the same DataLogReader-based tooling the auto-regression suite's
 * analysis readers use). Real characterization requires the physical robot -- see
 * docs/SysId_Characterization_Checklist.md SS6 and this plan's Phase 1 TODO.
 */
class CommandSwerveDrivetrainSysIdSimWorkflowTest {
    // Bounded windows, well under SysIdRoutine.Config's default 10s timeout -- proves the
    // lifecycle works without waiting out the full routine. 3s matches the checklist's own stated
    // quasistatic minimum (SS4); 1.5s is comfortably inside its "couple of seconds" dynamic norm.
    private static final long kQuasistaticWindowMillis = 3000;
    private static final long kDynamicWindowMillis = 1500;

    private static final String kAppliedVoltsEntryName = "/RealOutputs/Drive/AppliedVoltsPerModule";
    private static final String kDoubleArrayType = "double[]";

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
    @Timeout(45)
    void translationSysIdLifecycleRunsAndCapturesTelemetryInSim() throws IOException, InterruptedException {
        assertTrue(robotThread.isAlive(),
                "startCompetition() loop should still be running after waitForProgramStart()");

        // SysIdRoutine.Mechanism's setControl() calls are ignored while disabled (WPILib safety
        // interlock) -- teleop-enabled, matching docs/SysId_Characterization_Checklist.md SS4's
        // "Teleop or Test" recommendation.
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
        drivetrain.useTranslationSysId();

        Command quasistatic = drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
        CommandScheduler.getInstance().schedule(quasistatic);
        // Same paused-schedule-then-single-step pattern as AutoRegressionTestBase -- avoids the
        // documented cross-thread scheduling race between this test thread's schedule() call and
        // the robot thread's concurrently-running periodic loop.
        SimHooks.stepTiming(0.02);
        SimHooks.resumeTiming();
        timingResumed = true;

        Thread.sleep(kQuasistaticWindowMillis);
        assertTrue(CommandScheduler.getInstance().isScheduled(quasistatic),
                "quasistatic command should still be running mid-window, not have errored out early");
        CommandScheduler.getInstance().cancel(quasistatic);

        Command dynamic = drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward);
        CommandScheduler.getInstance().schedule(dynamic);
        Thread.sleep(kDynamicWindowMillis);
        assertTrue(CommandScheduler.getInstance().isScheduled(dynamic),
                "dynamic command should still be running mid-window, not have errored out early");
        CommandScheduler.getInstance().cancel(dynamic);

        finishCompetitionAndClose();

        Path wpilog = findNewestByExtension("logs", ".wpilog");
        assertTrue(wpilog != null, "No .wpilog found in logs/ after the run -- SIM mode should have written one.");

        AppliedVoltsSummary summary = readAppliedVoltsPerModule(wpilog);
        assertTrue(summary.sampleCount() > 10,
                "expected many Drive/AppliedVoltsPerModule samples across the run, got only "
                        + summary.sampleCount());
        assertTrue(summary.maxAbsVolts() > 0.5,
                "expected a real nonzero applied-voltage sample during the SysId window (quasistatic ramps "
                        + "to ~3V over 3s, dynamic steps to 4V) -- got max abs " + summary.maxAbsVolts()
                        + "V, suggesting the routine never actually drove the motors");
        assertTrue(summary.distinctRoundedValues() > 3,
                "expected AppliedVoltsPerModule to show multiple distinct values (a ramping/stepping signal), "
                        + "not a constant -- got only " + summary.distinctRoundedValues() + " distinct values");

        // Phoenix's SignalLogger writes a separate .hoot file (docs/SysId_Characterization_
        // Checklist.md SS3) -- the real analyzer's data source, distinct from this wpilog. Whether
        // headless JUnit sim (no real CAN bus) actually produces one is exactly one of the open
        // questions this workflow validation exists to surface, not something to assume either
        // way -- recorded as an observation, not asserted.
        boolean hootLogProduced = findNewestByExtension("logs", ".hoot") != null;
        System.out.println("[SysId sim workflow validation] .hoot log produced in logs/: " + hootLogProduced);
    }

    private record AppliedVoltsSummary(int sampleCount, double maxAbsVolts, int distinctRoundedValues) {}

    /**
     * Reads every {@code Drive/AppliedVoltsPerModule} sample from a wpilog, using WPILib's own
     * {@link DataLogReader}/{@link DataLogRecord} -- the same library
     * {@code frc.robot.auto.WpilogTrajectoryErrorReader} uses -- to prove the "analysis pipeline
     * can consume the generated data" requirement, not just that a file exists.
     */
    private static AppliedVoltsSummary readAppliedVoltsPerModule(Path wpilogFile) throws IOException {
        DataLogReader reader = new DataLogReader(wpilogFile.toString());
        if (!reader.isValid()) {
            throw new IOException("Not a valid WPILOG file: " + wpilogFile);
        }

        Map<Integer, String> activeNames = new HashMap<>();
        Map<Integer, String> activeTypes = new HashMap<>();
        boolean sawEntry = false;
        int sampleCount = 0;
        double maxAbs = 0.0;
        Set<Double> distinctRounded = new HashSet<>();

        for (DataLogRecord record : reader) {
            if (record.isStart()) {
                DataLogRecord.StartRecordData start = record.getStartData();
                activeNames.put(start.entry, start.name);
                activeTypes.put(start.entry, start.type);
                if (start.name.equals(kAppliedVoltsEntryName)) {
                    sawEntry = true;
                }
                continue;
            }
            if (record.isFinish() || record.isSetMetadata()) {
                continue;
            }

            String name = activeNames.get(record.getEntry());
            String type = activeTypes.get(record.getEntry());
            if (!kAppliedVoltsEntryName.equals(name) || !kDoubleArrayType.equals(type)) {
                continue;
            }

            for (double volts : record.getDoubleArray()) {
                if (Double.isNaN(volts)) {
                    continue;
                }
                sampleCount++;
                maxAbs = Math.max(maxAbs, Math.abs(volts));
                distinctRounded.add(Math.round(volts * 10.0) / 10.0);
            }
        }

        if (!sawEntry) {
            throw new IOException("wpilog " + wpilogFile + " never logged " + kAppliedVoltsEntryName
                    + " -- is logDriveMotorVoltages() wired up?");
        }

        return new AppliedVoltsSummary(sampleCount, maxAbs, distinctRounded.size());
    }

    /** Newest-by-mtime, matching AutoRegressionTestBase's own findNewestWpilog() convention. */
    private static Path findNewestByExtension(String dir, String extension) throws IOException {
        Path logsDir = Paths.get(dir);
        if (!Files.isDirectory(logsDir)) {
            return null;
        }
        try (Stream<Path> files = Files.list(logsDir)) {
            return files.filter(p -> p.toString().endsWith(extension))
                    .max(Comparator.comparingLong(p -> p.toFile().lastModified()))
                    .orElse(null);
        }
    }
}
