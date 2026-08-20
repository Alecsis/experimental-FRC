package frc.robot.auto;

import org.wpilib.datalog.DataLogReader;
import org.wpilib.datalog.DataLogRecord;

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
        // {timestampSeconds, 1|0} -- SetpointFresh is a de-duplicated LEVEL in the log, so both
        // edges matter, not just the true ones. See checkForStall's javadoc.
        List<double[]> setpointFreshTransitions = new ArrayList<>();

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
                setpointFreshTransitions.add(
                        new double[] {tSeconds, record.getBoolean() ? 1.0 : 0.0});
            }
        }

        if (poseTrace.isEmpty()) {
            throw new IOException("wpilog " + wpilogFile + " never logged " + kPoseEntryName
                    + " -- is Telemetry.telemeterize() wired up?");
        }

        return checkForStall(poseTrace, setpointFreshTransitions, stallWindowSeconds,
                stallThresholdMeters, graceSeconds);
    }

    /**
     * Pure stall rule, split out from the file reader so it can be unit-tested without a .wpilog
     * (see WpilogStallAnalyzerTest).
     *
     * <p>A sample counts as "path-following active" when a fresh PathPlanner setpoint arrived
     * within {@code graceSeconds} before it. The stall check runs only where the ENTIRE trailing
     * window is active, i.e. a path was driving for the whole window.
     *
     * <p>That last part is the point: gating on the globally-last fresh setpoint instead (the
     * original rule) silently assumed the robot never stops until the final path ends. It does now
     * -- "LT Neutral" holds position between path segments while its parallel intake branch runs
     * (see AutoPathEndVelocityTest) -- and that deliberate wait was being reported as a stall.
     * Requiring the whole window to be active also keeps the dwell suppressed for the first window
     * after the next path starts, when the trailing window still overlaps the wait.
     *
     * <p>{@code Trajectory/SetpointFresh} is written every periodic tick and a fresh setpoint
     * arrives on every tick while a path drives, so WPILOG's change-only de-duplication turns it
     * into a LEVEL: one {@code true} record at the rising edge, one {@code false} at the falling
     * edge (measured: 5 records across a 15 s "LT Neutral" run). It is therefore passed in as
     * transitions and reconstructed here, not treated as a list of per-tick pulses.
     *
     * @param poseTrace {@code {timestampSeconds, xMeters, yMeters}} samples, any order
     * @param setpointFreshTransitions {@code {timestampSeconds, 1|0}} level transitions, any order
     */
    static Result checkForStall(List<double[]> poseTrace, List<double[]> setpointFreshTransitions,
            double stallWindowSeconds, double stallThresholdMeters, double graceSeconds) {
        boolean everFresh = setpointFreshTransitions.stream().anyMatch(r -> r[1] != 0.0);
        if (!everFresh) {
            return new Result(false, "no fresh PathPlanner setpoint ever logged -- "
                    + "path-following-window stall check skipped entirely (nothing to gate on)");
        }

        List<double[]> trace = new ArrayList<>(poseTrace);
        trace.sort((a, b) -> Double.compare(a[0], b[0]));
        List<double[]> fresh = new ArrayList<>(setpointFreshTransitions);
        fresh.sort((a, b) -> Double.compare(a[0], b[0]));

        // Index of the most recent sample that was NOT path-following active. Any window reaching
        // back to or past it overlaps a no-path-driving stretch and must not be judged.
        int lastInactiveIndex = -1;
        int freshCursor = -1;
        // Index of the newest sample at or before (tI - stallWindowSeconds), so the comparison
        // below always spans a FULL window. Starts at -1 = no such sample yet.
        int windowStart = -1;

        for (int i = 0; i < trace.size(); i++) {
            double tI = trace.get(i)[0];
            while (freshCursor + 1 < fresh.size() && fresh.get(freshCursor + 1)[0] <= tI) {
                freshCursor++;
            }
            // Level in effect at tI: true => a path is driving right now; false => a path stopped
            // driving at that transition, and the grace window still counts as active.
            boolean active = freshCursor >= 0
                    && (fresh.get(freshCursor)[1] != 0.0
                            || tI <= fresh.get(freshCursor)[0] + graceSeconds);
            if (!active) {
                lastInactiveIndex = i;
            }

            while (windowStart + 1 < trace.size()
                    && trace.get(windowStart + 1)[0] <= tI - stallWindowSeconds) {
                windowStart++;
            }
            if (windowStart < 0) {
                continue; // not enough history yet to span a full stallWindowSeconds
            }
            if (lastInactiveIndex >= windowStart) {
                continue; // window overlaps a stretch with no path driving -- stationary is expected
            }
            double dx = trace.get(i)[1] - trace.get(windowStart)[1];
            double dy = trace.get(i)[2] - trace.get(windowStart)[2];
            double moved = Math.hypot(dx, dy);
            if (moved < stallThresholdMeters) {
                double windowSpan = tI - trace.get(windowStart)[0];
                return new Result(true, String.format(
                        "stalled at t=%.2fs: moved %.3fm over the preceding %.2fs (threshold %.3fm), "
                                + "with a path actively driving throughout that window "
                                + "(most recent SetpointFresh transition t=%.2fs, grace %.2fs)",
                        tI, moved, windowSpan, stallThresholdMeters, fresh.get(freshCursor)[0],
                        graceSeconds));
            }
        }

        return new Result(false, "no stall detected");
    }
}
