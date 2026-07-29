package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for {@link WpilogStallAnalyzer}'s stall rule, exercised through its pure
 * trace-in/result-out core so no .wpilog file or robot boot is needed.
 *
 * <p>Motivating defect (measured 2026-07-29): the analyzer suspended its check only for samples
 * past the GLOBALLY LAST fresh path setpoint. That was indistinguishable from "a path is currently
 * driving" only as long as the robot never stopped mid-auto. Once "Left Trench Start" was corrected
 * to end at rest (see {@link AutoPathEndVelocityTest}), "LT Neutral" gained a legitimate ~2.8 s
 * stationary dwell between path segments -- the robot waiting, on target, for its parallel
 * "Intake Start Sequence" branch to finish -- and the analyzer reported that deliberate wait as a
 * stall. The analyzer's own comment already states the intended rule ("past the path-following
 * phase -- stationary aim/shoot is expected here"); it just measured "the path-following phase"
 * against the wrong reference point.
 *
 * <p>{@code Trajectory/SetpointFresh} is recorded every periodic tick by
 * {@code TrajectoryErrorTracker.logAndClearFreshness()}, and a fresh setpoint arrives on every
 * tick while a path is driving -- so, after WPILOG's change-only record de-duplication, the entry
 * reads back as a LEVEL: one {@code true} record at the rising edge, one {@code false} at the
 * falling edge. Measured on a real "LT Neutral" log: 5 records total across a 15 s run, not the
 * ~700 a per-tick pulse would produce. The transitions are therefore passed here as
 * {@code {timestampSeconds, 1|0}} pairs and reconstructed into intervals.
 */
class WpilogStallAnalyzerTest {
  private static final double kWindowSeconds = 1.0;
  private static final double kThresholdMeters = 0.05;
  private static final double kGraceSeconds = 0.5;

  @Test
  void stationaryDwellBetweenTwoPathSegmentsIsNotAStall() {
    // Path A drives 0.0-2.0s, robot then waits in place 2.0-5.0s (parallel sibling still running),
    // path B drives 5.0-7.0s. The wait is deliberate, not a hang.
    List<double[]> fresh = levels(0.0, 2.0, 5.0, 7.0);

    List<double[]> trace = new ArrayList<>();
    appendMoving(trace, 0.0, 2.0, 0.0);
    appendStationary(trace, 2.0, 5.0, 2.0);
    appendMoving(trace, 5.0, 7.0, 2.0);

    WpilogStallAnalyzer.Result result =
        WpilogStallAnalyzer.checkForStall(trace, fresh, kWindowSeconds, kThresholdMeters,
            kGraceSeconds);

    assertFalse(result.stalled(),
        "a stationary phase between two path segments is a deliberate wait, not a stall: "
            + result.diagnostic());
  }

  @Test
  void stationaryWhileAPathIsActivelyDrivingIsStillAStall() {
    // Non-vacuity guard: the check must still catch the real failure it exists for -- fresh
    // setpoints keep arriving (a path IS commanding motion) but the robot never moves.
    List<double[]> fresh = levels(0.0, 5.0);
    List<double[]> trace = new ArrayList<>();
    appendStationary(trace, 0.0, 5.0, 0.0);

    WpilogStallAnalyzer.Result result =
        WpilogStallAnalyzer.checkForStall(trace, fresh, kWindowSeconds, kThresholdMeters,
            kGraceSeconds);

    assertTrue(result.stalled(),
        "a motionless robot while a path is actively driving must still be reported as stalled");
  }

  @Test
  void dwellIsStillSuppressedForTheFirstWindowAfterTheNextPathStarts() {
    // The trailing window is one second long, so for up to a second after path B starts the window
    // still contains the dwell's motionless samples. Those must not resurrect the false stall.
    List<double[]> fresh = levels(0.0, 2.0, 5.0, 7.0);

    List<double[]> trace = new ArrayList<>();
    appendMoving(trace, 0.0, 2.0, 0.0);
    appendStationary(trace, 2.0, 5.0, 2.0);
    // Path B accelerates from rest, so it barely moves during its first ~0.3s.
    appendStationary(trace, 5.0, 5.3, 2.0);
    appendMoving(trace, 5.3, 7.0, 2.0);

    WpilogStallAnalyzer.Result result =
        WpilogStallAnalyzer.checkForStall(trace, fresh, kWindowSeconds, kThresholdMeters,
            kGraceSeconds);

    assertFalse(result.stalled(),
        "the dwell must stay suppressed while it is still inside the trailing window: "
            + result.diagnostic());
  }

  /**
   * Builds {@code {timestamp, 1|0}} level transitions from alternating rising/falling edge times,
   * mirroring how WPILOG stores the de-duplicated {@code Trajectory/SetpointFresh} entry.
   */
  private static List<double[]> levels(double... edges) {
    List<double[]> out = new ArrayList<>();
    for (int i = 0; i < edges.length; i++) {
      out.add(new double[] {edges[i], i % 2 == 0 ? 1.0 : 0.0});
    }
    return out;
  }

  /** Appends 50 Hz samples advancing 1 m/s in +x, starting from xStart. */
  private static void appendMoving(List<double[]> trace, double startSeconds, double endSeconds,
      double xStart) {
    for (double t = startSeconds; t < endSeconds - 1e-9; t += 0.02) {
      trace.add(new double[] {t, xStart + (t - startSeconds), 0.0});
    }
  }

  /** Appends 50 Hz samples that never move. */
  private static void appendStationary(List<double[]> trace, double startSeconds,
      double endSeconds, double x) {
    for (double t = startSeconds; t < endSeconds - 1e-9; t += 0.02) {
      trace.add(new double[] {t, x, 0.0});
    }
  }
}
