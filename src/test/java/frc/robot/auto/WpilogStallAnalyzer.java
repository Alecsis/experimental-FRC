package frc.robot.auto;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Post-hoc stall detection from a .wpilog, not a live poll during the run. A live NetworkTables
 * poll of /RealOutputs/Trajectory/SetpointFresh was tried first and rejected: that entry is only
 * "true" for the single periodic tick a fresh PathPlanner setpoint arrives, and a real run of
 * "LT Neutral" showed it firing exactly once at t=0.324s -- a ~20ms test-thread poll loop that
 * isn't synchronized to the robot's own tick reliably missed that one-tick pulse (confirmed
 * empirically: the poll never observed it even though the log did). The wpilog captures every
 * tick synchronously from the actual robot loop, so it doesn't have that aliasing problem -- same
 * reasoning the user already applied when choosing post-hoc log parsing over a live read for the
 * max-tracking-error metric.
 *
 * <p>Pose source is /RealOutputs/Odometry/Robot (Telemetry.java's own CTRE-telemetry-rate
 * publish, the same entry Milestone 1's own validation hand-verified against raw setpoint bytes),
 * not a live drivetrain.getState().Pose read.
 */
final class WpilogStallAnalyzer {
    private static final String kPoseEntryName = "/RealOutputs/Odometry/Robot";
    private static final String kSetpointFreshEntryName = "/RealOutputs/Trajectory/SetpointFresh";
    private static final String kPoseType = "struct:Pose2d";
    private static final String kBooleanType = "boolean";

    record Result(boolean stalled, String diagnostic) {}

    private WpilogStallAnalyzer() {}

    static Result checkForStall(Path wpilogFile, double stallWindowSeconds,
            double stallThresholdMeters, double graceSeconds) throws IOException {
        DataLogReader reader = new DataLogReader(wpilogFile.toString());
        if (!reader.isValid()) {
            throw new IOException("Not a valid WPILOG file: " + wpilogFile);
        }

        Map<Integer, String> activeNames = new HashMap<>();
        Map<Integer, String> activeTypes = new HashMap<>();
        // {timestampSeconds, xMeters, yMeters}, sorted below -- WPILOG doesn't guarantee
        // per-entry record order (see parse_akit_log.py's own docstring on this).
        List<double[]> poseTrace = new ArrayList<>();
        List<Double> freshSetpointTimestamps = new ArrayList<>();

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
            if (name == null) {
                continue;
            }
            double tSeconds = record.getTimestamp() / 1e6;

            if (name.equals(kPoseEntryName) && kPoseType.equals(type) && record.getSize() == 24) {
                ByteBuffer buf = record.getRawBuffer().order(ByteOrder.LITTLE_ENDIAN);
                double x = buf.getDouble(0);
                double y = buf.getDouble(8);
                poseTrace.add(new double[] {tSeconds, x, y});
            } else if (name.equals(kSetpointFreshEntryName) && kBooleanType.equals(type)) {
                if (record.getBoolean()) {
                    freshSetpointTimestamps.add(tSeconds);
                }
            }
        }

        if (poseTrace.isEmpty()) {
            throw new IOException("wpilog " + wpilogFile + " never logged " + kPoseEntryName
                    + " -- is Telemetry.telemeterize() wired up?");
        }
        if (freshSetpointTimestamps.isEmpty()) {
            return new Result(false, "no fresh PathPlanner setpoint ever logged -- "
                    + "path-following-window stall check skipped entirely (nothing to gate on)");
        }

        poseTrace.sort((a, b) -> Double.compare(a[0], b[0]));
        double lastFreshSetpointSeconds = freshSetpointTimestamps.get(freshSetpointTimestamps.size() - 1);

        int windowStart = 0;
        for (int i = 0; i < poseTrace.size(); i++) {
            double tI = poseTrace.get(i)[0];
            while (poseTrace.get(windowStart)[0] < tI - stallWindowSeconds) {
                windowStart++;
            }
            if (windowStart == i) {
                continue; // not enough history yet for a full window
            }
            if (tI > lastFreshSetpointSeconds + graceSeconds) {
                continue; // past the path-following phase -- stationary aim/shoot is expected here
            }
            double dx = poseTrace.get(i)[1] - poseTrace.get(windowStart)[1];
            double dy = poseTrace.get(i)[2] - poseTrace.get(windowStart)[2];
            double moved = Math.hypot(dx, dy);
            if (moved < stallThresholdMeters) {
                double windowSpan = tI - poseTrace.get(windowStart)[0];
                return new Result(true, String.format(
                        "stalled at t=%.2fs: moved %.3fm over the preceding %.2fs (threshold %.3fm), "
                                + "still within %.2fs of the last fresh path setpoint (t=%.2fs)",
                        tI, moved, windowSpan, stallThresholdMeters, graceSeconds, lastFreshSetpointSeconds));
            }
        }

        return new Result(false, "no stall detected");
    }
}
