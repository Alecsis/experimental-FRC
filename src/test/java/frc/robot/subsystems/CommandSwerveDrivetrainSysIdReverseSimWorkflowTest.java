// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.datalog.DataLogReader;
import org.wpilib.datalog.DataLogRecord;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.sysid.SysIdRoutine;
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
 * Companion to {@link CommandSwerveDrivetrainSysIdSimWorkflowTest}, which only exercised
 * {@link SysIdRoutine.Direction#kForward}. This class exercises {@link SysIdRoutine.Direction#kReverse}
 * for both quasistatic and dynamic, completing all four routines a real characterization session
 * requires (docs/SysId_Characterization_Checklist.md SS4's "run all four directions" procedure).
 *
 * <p>Deliberately a separate top-level class rather than a second {@code @Test} method on the
 * existing file: AdvantageKit's {@code Logger} is a JVM-wide singleton that can only be started
 * once (confirmed empirically -- a second {@code new Robot()} in the same JVM throws
 * {@code IllegalThreadStateException} from {@code Logger.start()}), and this project's
 * {@code build.gradle} only forks a new JVM per test <b>class</b> ({@code forkEvery = 1}), not per
 * test method -- the same reason every {@code AutoRegressionTestBase} subclass is its own class
 * with exactly one {@code Robot()}-booting test.
 *
 * <p><b>THIS IS NOT HARDWARE CHARACTERIZATION.</b> No kS/kV/kA value is computed here; only
 * pipeline behavior is checked, mirroring the forward-direction test exactly.
 */
class CommandSwerveDrivetrainSysIdReverseSimWorkflowTest {
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
    void reverseSysIdLifecycleRunsAndCapturesTelemetryInSim() throws IOException, InterruptedException {
        assertTrue(robotThread.isAlive(),
                "startCompetition() loop should still be running after waitForProgramStart()");

        // Same teleop-enabled interlock rationale as the forward-direction test.
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
        drivetrain.useTranslationSysId();

        Command quasistatic = drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
        CommandScheduler.getInstance().schedule(quasistatic);
        SimHooks.stepTiming(0.02);
        SimHooks.resumeTiming();
        timingResumed = true;

        Thread.sleep(kQuasistaticWindowMillis);
        assertTrue(CommandScheduler.getInstance().isScheduled(quasistatic),
                "reverse quasistatic command should still be running mid-window, not have errored out early");
        CommandScheduler.getInstance().cancel(quasistatic);

        Command dynamic = drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse);
        CommandScheduler.getInstance().schedule(dynamic);
        Thread.sleep(kDynamicWindowMillis);
        assertTrue(CommandScheduler.getInstance().isScheduled(dynamic),
                "reverse dynamic command should still be running mid-window, not have errored out early");
        CommandScheduler.getInstance().cancel(dynamic);

        finishCompetitionAndClose();

        Path wpilog = findNewestByExtension("logs", ".wpilog");
        assertTrue(wpilog != null, "No .wpilog found in logs/ after the run -- SIM mode should have written one.");

        AppliedVoltsSummary summary = readAppliedVoltsPerModule(wpilog);
        assertTrue(summary.sampleCount() > 10,
                "expected many Drive/AppliedVoltsPerModule samples across the run, got only "
                        + summary.sampleCount());
        // Reverse is distinguished from forward by sign, not just magnitude -- minVolts() < 0
        // proves the routine actually commanded negative-direction voltage, not merely "some
        // nonzero value" (which a maxAbsVolts()-only check, as used by the forward test, would not
        // catch if reverse silently behaved like forward).
        assertTrue(summary.minVolts() < -0.5,
                "expected a real negative applied-voltage sample during the reverse SysId window -- got min "
                        + summary.minVolts() + "V, suggesting the reverse direction never actually drove the "
                        + "motors backward");
        assertTrue(summary.distinctRoundedValues() > 3,
                "expected AppliedVoltsPerModule to show multiple distinct values (a ramping/stepping signal), "
                        + "not a constant -- got only " + summary.distinctRoundedValues() + " distinct values");

        boolean hootLogProduced = findNewestByExtension("logs", ".hoot") != null;
        System.out.println("[SysId sim workflow validation] .hoot log produced in logs/ (reverse run): "
                + hootLogProduced);
    }

    private record AppliedVoltsSummary(int sampleCount, double minVolts, int distinctRoundedValues) {}

    private static AppliedVoltsSummary readAppliedVoltsPerModule(Path wpilogFile) throws IOException {
        DataLogReader reader = new DataLogReader(wpilogFile.toString());
        if (!reader.isValid()) {
            throw new IOException("Not a valid WPILOG file: " + wpilogFile);
        }

        Map<Integer, String> activeNames = new HashMap<>();
        Map<Integer, String> activeTypes = new HashMap<>();
        boolean sawEntry = false;
        int sampleCount = 0;
        double minVolts = 0.0;
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
                minVolts = Math.min(minVolts, volts);
                distinctRounded.add(Math.round(volts * 10.0) / 10.0);
            }
        }

        if (!sawEntry) {
            throw new IOException("wpilog " + wpilogFile + " never logged " + kAppliedVoltsEntryName
                    + " -- is logDriveMotorVoltages() wired up?");
        }

        return new AppliedVoltsSummary(sampleCount, minVolts, distinctRounded.size());
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
