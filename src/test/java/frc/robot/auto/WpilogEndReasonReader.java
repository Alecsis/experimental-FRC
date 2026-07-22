package frc.robot.auto;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

/**
 * Scans a .wpilog for whether a given string entry was ever logged, at least once. Deliberately
 * minimal compared to WpilogTrajectoryErrorReader -- this only needs presence, not a computed
 * max, to prove Superstructure's end-reason telemetry (Task 1 of the completion-trigger framework
 * plan) actually reaches the log in a real run.
 */
final class WpilogEndReasonReader {
    private static final String kStringType = "string";

    private WpilogEndReasonReader() {}

    static boolean wasLogged(Path wpilogFile, String entryName) throws IOException {
        DataLogReader reader = new DataLogReader(wpilogFile.toString());
        if (!reader.isValid()) {
            throw new IOException("Not a valid WPILOG file: " + wpilogFile);
        }

        Map<Integer, String> activeNames = new HashMap<>();
        Map<Integer, String> activeTypes = new HashMap<>();

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
            if (entryName.equals(name) && kStringType.equals(type)) {
                return true;
            }
        }

        return false;
    }
}
