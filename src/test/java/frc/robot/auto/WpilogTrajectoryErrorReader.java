package frc.robot.auto;

import org.wpilib.datalog.DataLogReader;
import org.wpilib.datalog.DataLogRecord;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

/**
 * Scans a .wpilog for the two error entries TrajectoryErrorTracker already logs
 * (frc.robot.utility.TrajectoryErrorTracker#logAndClearFreshness) and returns the max-abs value
 * of each over the file. Read-only, test-scope only -- never touches TrajectoryErrorTracker or
 * any production file.
 *
 * <p>Built on WPILib's own {@link DataLogReader}/{@link DataLogRecord} (org.wpilib.util.
 * datalog, a transitive dependency of wpi.java.deps.wpilib() already present on the test
 * classpath -- the same classes AdvantageKit's own WPILOGReader uses internally), not a
 * hand-rolled binary parser.
 */
final class WpilogTrajectoryErrorReader {
    // AdvantageKit prefixes every Logger.recordOutput() call with "/RealOutputs/" (SIM/REAL
    // modes) before writing to the WPILOG -- confirmed against the actual log this suite produces
    // (python SKILLS/parse_akit_log.py ... --grep Trajectory), not assumed from the raw name
    // TrajectoryErrorTracker passes to recordOutput().
    static final String kLateralEntryName = "/RealOutputs/Trajectory/ErrorLateralMeters";
    static final String kLongitudinalEntryName = "/RealOutputs/Trajectory/ErrorLongitudinalMeters";
    private static final String kDoubleType = "double";

    record MaxErrors(double maxLateralErrorMeters, double maxLongitudinalErrorMeters) {}

    private WpilogTrajectoryErrorReader() {}

    static MaxErrors readMaxErrors(Path wpilogFile) throws IOException {
        DataLogReader reader = new DataLogReader(wpilogFile.toString());
        if (!reader.isValid()) {
            throw new IOException("Not a valid WPILOG file: " + wpilogFile);
        }

        // Entry IDs can be reused after a Finish record (per the WPILOG spec), so track the
        // currently-active name/type for each ID rather than assuming a fixed mapping.
        Map<Integer, String> activeNames = new HashMap<>();
        Map<Integer, String> activeTypes = new HashMap<>();
        boolean sawLateralEntry = false;
        boolean sawLongitudinalEntry = false;
        double maxLateral = 0.0;
        double maxLongitudinal = 0.0;

        for (DataLogRecord record : reader) {
            if (record.isStart()) {
                DataLogRecord.StartRecordData start = record.getStartData();
                activeNames.put(start.entry, start.name);
                activeTypes.put(start.entry, start.type);
                if (start.name.equals(kLateralEntryName)) {
                    sawLateralEntry = true;
                } else if (start.name.equals(kLongitudinalEntryName)) {
                    sawLongitudinalEntry = true;
                }
                continue;
            }
            if (record.isFinish() || record.isSetMetadata()) {
                continue;
            }

            String name = activeNames.get(record.getEntry());
            String type = activeTypes.get(record.getEntry());
            if (name == null || !kDoubleType.equals(type)) {
                continue;
            }

            double value = record.getDouble();
            if (Double.isNaN(value)) {
                continue; // NaN means "no setpoint/tangent yet" (TrajectoryErrorTracker), not an error sample.
            }
            if (name.equals(kLateralEntryName)) {
                maxLateral = Math.max(maxLateral, Math.abs(value));
            } else if (name.equals(kLongitudinalEntryName)) {
                maxLongitudinal = Math.max(maxLongitudinal, Math.abs(value));
            }
        }

        if (!sawLateralEntry || !sawLongitudinalEntry) {
            throw new IOException(
                    "wpilog " + wpilogFile + " never logged " + kLateralEntryName + " / "
                            + kLongitudinalEntryName + " -- is TrajectoryErrorTracker wired up?");
        }

        return new MaxErrors(maxLateral, maxLongitudinal);
    }
}
