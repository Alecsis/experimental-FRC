package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;

/**
 * Pins the two halves of the stall rule together, and pins both against the measured RB Neutral
 * false positive that motivated replacing endpoint displacement with accumulated path length.
 *
 * <p><b>Why two implementations exist at all:</b> {@link AutonomousHealthMonitor} runs live inside
 * robotPeriodic() as telemetry, while {@link WpilogStallAnalyzer} grades a finished run post-hoc
 * from a .wpilog for the regression suite. Production must not depend on the test source set, so
 * the rule is written twice. That duplication is only safe while the two agree, which is what
 * {@link #bothImplementationsAgreeAcrossTheSameTraces()} enforces.
 *
 * <p><b>The defect being fixed.</b> Both used {@code hypot(newest - oldest)} over the trailing
 * window. RB Neutral decelerates to rest at Neutral Position 3, drives 0.51 m back the other way
 * along {@code Bump Neutral Right}, then continues onto {@code Right Bump End - Special} -- so
 * inside one 1.0 s window the robot doubles back and its endpoint displacement collapses toward
 * zero while it is plainly moving. Measured in 3 of 3 runs
 * ({@code logs/akit_26-08-20_01-46-03.wpilog}, {@code akit_26-08-19_18-07-00.wpilog},
 * {@code akit_26-08-19_16-36-12.wpilog}): {@code /RealOutputs/Auto/Health/Stalled} went true at
 * t = 2.983-3.043 s with a path actively driving.
 *
 * <p>{@link #kMeasuredRbNeutralReversal} is not a reconstruction -- it is the literal
 * {@code /RealOutputs/Odometry/Robot} slice from the first of those logs.
 */
class StallDetectorMirroringTest {
    private static final double kWindowSeconds = 1.0;
    private static final double kThresholdMeters = 0.05;
    private static final double kGraceSeconds = 0.5;

    /**
     * {@code {timestampSeconds, xMeters, yMeters}}, verbatim from
     * {@code logs/akit_26-08-20_01-46-03.wpilog} (RB Neutral), t in [1.900, 3.020] -- the window
     * containing the reversal. {@code /RealOutputs/Trajectory/SetpointFresh} in that log is a
     * single true level from t = 0.181 s to t = 6.001 s, so a path is driving across every sample
     * here and the check is NOT suppressed by the grace logic.
     *
     * <p>Across this slice the robot travels 0.818 m of path but ends only 0.092 m from where it
     * started. The old endpoint rule reports a stall at t = 3.000 s having "moved" 0.043 m.
     */
    private static final double[][] kMeasuredRbNeutralReversal = {
        {1.900428, 6.169906, 2.383939},
        {1.920578, 6.194706, 2.381230},
        {1.940818, 6.200535, 2.377932},
        {1.960700, 6.214008, 2.382019},
        {1.981551, 6.229629, 2.380615},
        {2.001064, 6.246181, 2.381794},
        {2.020584, 6.272732, 2.384732},
        {2.040297, 6.282305, 2.386602},
        {2.060050, 6.300645, 2.388809},
        {2.080917, 6.318053, 2.388692},
        {2.100509, 6.339180, 2.392482},
        {2.120292, 6.359082, 2.394159},
        {2.140933, 6.395392, 2.394848},
        {2.160466, 6.397103, 2.396630},
        {2.180443, 6.415746, 2.397068},
        {2.200151, 6.433612, 2.397605},
        {2.220845, 6.450318, 2.395092},
        {2.240925, 6.469183, 2.397887},
        {2.260625, 6.504387, 2.398586},
        {2.280814, 6.504134, 2.399272},
        {2.300916, 6.522605, 2.399961},
        {2.320502, 6.541619, 2.400239},
        {2.340875, 6.564119, 2.397735},
        {2.360437, 6.573903, 2.398559},
        {2.380573, 6.585963, 2.395647},
        {2.400640, 6.604943, 2.391315},
        {2.421271, 6.607138, 2.390315},
        {2.441008, 6.611478, 2.386227},
        {2.460353, 6.613365, 2.383225},
        {2.480317, 6.612561, 2.381378},
        {2.500627, 6.605848, 2.381042},
        {2.521104, 6.599472, 2.381775},
        {2.540825, 6.594823, 2.383251},
        {2.560977, 6.582358, 2.384586},
        {2.580639, 6.564467, 2.386505},
        {2.600768, 6.560353, 2.387698},
        {2.620637, 6.533681, 2.387228},
        {2.640834, 6.532109, 2.387515},
        {2.660967, 6.501834, 2.385848},
        {2.680074, 6.500621, 2.385916},
        {2.700432, 6.484293, 2.384459},
        {2.720160, 6.466254, 2.382531},
        {2.740571, 6.449256, 2.380461},
        {2.760506, 6.426846, 2.377261},
        {2.780324, 6.413624, 2.375463},
        {2.800945, 6.391361, 2.371839},
        {2.820570, 6.378327, 2.369672},
        {2.840731, 6.348823, 2.364194},
        {2.861051, 6.341402, 2.362639},
        {2.880936, 6.329720, 2.360297},
        {2.901113, 6.315335, 2.357358},
        {2.920028, 6.301084, 2.354613},
        {2.941307, 6.287788, 2.352177},
        {2.960596, 6.276198, 2.350105},
        {2.980085, 6.265639, 2.348004},
        {3.000112, 6.253623, 2.344870},
    };

    // ---- The measured false positive ----

    @Test
    void measuredRbNeutralReversalIsNoLongerReportedAsAStall() {
        WpilogStallAnalyzer.Result result = analyze(trace(kMeasuredRbNeutralReversal));

        assertFalse(result.stalled(),
                "the measured RB Neutral direction reversal is real motion (0.818 m of path across"
                        + " the slice), not a stall: " + result.diagnostic());
    }

    @Test
    void measuredRbNeutralReversalIsNoLongerReportedAsAStallByTheLiveMonitor() {
        assertFalse(replayThroughMonitor(kMeasuredRbNeutralReversal),
                "the live monitor logged Stalled=true here in 3 of 3 recorded runs; accumulated"
                        + " path length must not repeat that");
    }

    /**
     * Negative control. Without it, the two assertions above could pass for the wrong reason -- a
     * rule that never fires also never false-fires. This runs the ORIGINAL endpoint-displacement
     * algorithm, transcribed unchanged, over the exact same measured samples and shows that it
     * does report the stall. So the samples genuinely contain the defect, and the fix is what
     * removes it.
     */
    @Test
    void negativeControlTheOldEndpointRuleDoesFailOnTheSameMeasuredSamples() {
        double[][] samples = kMeasuredRbNeutralReversal;
        Double stalledAt = null;
        double reported = Double.NaN;
        for (int i = 0; i < samples.length; i++) {
            double tI = samples[i][0];
            int windowStart = -1;
            while (windowStart + 1 < samples.length
                    && samples[windowStart + 1][0] <= tI - kWindowSeconds) {
                windowStart++;
            }
            if (windowStart < 0) {
                continue;
            }
            // The pre-fix rule, verbatim: straight-line distance between the window's endpoints.
            double moved = Math.hypot(samples[i][1] - samples[windowStart][1],
                    samples[i][2] - samples[windowStart][2]);
            if (moved < kThresholdMeters) {
                stalledAt = tI;
                reported = moved;
                break;
            }
        }

        assertNotNull(stalledAt,
                "negative control is vacuous: the old endpoint rule must fail on these samples,"
                        + " otherwise they do not capture the defect being fixed");
        assertEquals(3.000, stalledAt, 0.01,
                "the old rule's false positive should land on the reversal, near t=3.00s");
        assertTrue(reported < kThresholdMeters,
                "old rule reported " + reported + " m, which should be under the "
                        + kThresholdMeters + " m threshold");
    }

    // ---- Production / analyzer mirroring ----

    @Test
    void theTwoImplementationsUseTheSameNoiseFloor() {
        assertEquals(AutonomousHealthMonitor.kStallSampleNoiseFloorMeters,
                WpilogStallAnalyzer.kStallSampleNoiseFloorMeters,
                "the duplicated rule is only safe while its constants match");
    }

    @Test
    void theTwoImplementationsUseTheSameWindowAndThreshold() {
        assertEquals(kWindowSeconds, AutonomousHealthMonitor.kStallWindowSeconds);
        assertEquals(kThresholdMeters, AutonomousHealthMonitor.kStallTranslationThresholdMeters);
        assertEquals(kGraceSeconds, AutonomousHealthMonitor.kStalePathSetpointGraceSeconds);
        assertEquals(AutoRegressionTolerances.kStallWindowSeconds,
                AutonomousHealthMonitor.kStallWindowSeconds);
        assertEquals(AutoRegressionTolerances.kStallTranslationThresholdMeters,
                AutonomousHealthMonitor.kStallTranslationThresholdMeters);
    }

    /**
     * Semantic mirroring: the same motion must get the same verdict from both halves. Deliberately
     * not asserting tick-for-tick equality -- the analyzer anchors its window on the newest sample
     * at or before {@code t - window} while the monitor keeps every sample newer than
     * {@code t - window}, a pre-existing half-sample difference that predates this change. What
     * must agree is the answer.
     */
    @Test
    void bothImplementationsAgreeAcrossTheSameTraces() {
        List<Case> cases = List.of(
                new Case("measured RB Neutral reversal", kMeasuredRbNeutralReversal, false),
                new Case("motionless", stationary(0.0, 3.0, 1.0, 1.0), true),
                new Case("steady 1 m/s", monotonic(0.0, 3.0, 1.0), false),
                new Case("synthetic out-and-back", outAndBack(0.0, 3.0, 0.6), false),
                new Case("crawl above threshold", monotonic(0.0, 3.0, 0.08), false),
                new Case("crawl below threshold", monotonic(0.0, 3.0, 0.02), true));

        for (Case c : cases) {
            WpilogStallAnalyzer.Result analyzer = analyze(trace(c.samples()));
            boolean monitor = replayThroughMonitor(c.samples());
            assertEquals(c.expectStalled(), analyzer.stalled(),
                    "analyzer disagreed with the expected verdict for '" + c.name() + "': "
                            + analyzer.diagnostic());
            assertEquals(c.expectStalled(), monitor,
                    "live monitor disagreed with the expected verdict for '" + c.name() + "'");
            assertEquals(analyzer.stalled(), monitor,
                    "production monitor and wpilog analyzer diverged on '" + c.name() + "'");
        }
    }

    private record Case(String name, double[][] samples, boolean expectStalled) {}

    // ---- Harness ----

    /**
     * The former divergence, now closed. Before the full-window guard, the monitor graded partial
     * history against a whole-window threshold and briefly reported a stall during initial
     * acceleration -- measured as {@code /RealOutputs/Auto/Health/Stalled} true from t = 0.204 s to
     * t = 0.364 s in all three RB Neutral logs. The analyzer never did, because it refuses to judge
     * until {@code windowStart >= 0}. Both now decline.
     *
     * <p>This replaces the test that used to PIN that divergence as known-and-accepted.
     */
    @Test
    void neitherImplementationJudgesBeforeItsWindowIsFull() {
        double[][] slowStart = monotonic(0.0, 3.0, 1.0);

        AutonomousHealthMonitor monitor = freshMonitorOver(slowStart, 2); // only 2 samples fed
        assertFalse(monitor.isStalled(),
                "0.02 m of travel is under the threshold, but only 0.02 s of history exists --"
                        + " the monitor must not judge yet");

        assertFalse(analyze(trace(slowStart)).stalled(),
                "the analyzer likewise declines to judge until a full window exists");
    }

    /**
     * A stationary robot must still be caught the moment a full window HAS accumulated -- the guard
     * above must delay the verdict, not suppress it.
     */
    @Test
    void bothImplementationsStallAsSoonAsAFullStationaryWindowExists() {
        double[][] frozen = stationary(0.0, 3.0, 1.0, 1.0);

        assertFalse(freshMonitorOver(frozen, 10).isStalled(),
                "0.18 s of history is not a window; no verdict yet");
        assertTrue(freshMonitorOver(frozen, 60).isStalled(),
                "past 1.0 s of motionless path-following, the stall must be reported");
        assertTrue(analyze(trace(frozen)).stalled(),
                "and the analyzer must agree");
    }

    /**
     * Feeds a {@code {t,x,y}} trace through the live monitor the way robotPeriodic() would, with
     * path-following active throughout, and reports whether it ever declared a stall.
     *
     * <p>No gating needed any more: production now applies the same full-window rule internally, so
     * a mid-run slice replays faithfully on its own.
     */
    private static boolean replayThroughMonitor(double[][] samples) {
        double[] clock = {0.0};
        Pose2d[] pose = {new Pose2d()};
        Supplier<Pose2d> poseSupplier = () -> pose[0];
        AutonomousHealthMonitor monitor = new AutonomousHealthMonitor(
                () -> 0.0,
                () -> 0.0,
                poseSupplier,
                () -> clock[0],
                () -> 0.0, // always path-following, so the grace logic never suppresses the check
                () -> 0.0);

        boolean everStalled = false;
        for (double[] sample : samples) {
            clock[0] = sample[0];
            pose[0] = new Pose2d(sample[1], sample[2], pose[0].getRotation());
            monitor.update();
            everStalled |= monitor.isStalled();
        }
        return everStalled;
    }

    /** Runs a monitor over the first {@code sampleCount} samples of a trace and returns it. */
    private static AutonomousHealthMonitor freshMonitorOver(double[][] samples, int sampleCount) {
        double[] clock = {0.0};
        Pose2d[] pose = {new Pose2d()};
        Supplier<Pose2d> poseSupplier = () -> pose[0];
        AutonomousHealthMonitor monitor = new AutonomousHealthMonitor(
                () -> 0.0, () -> 0.0, poseSupplier, () -> clock[0], () -> 0.0, () -> 0.0);
        for (int i = 0; i < sampleCount && i < samples.length; i++) {
            clock[0] = samples[i][0];
            pose[0] = new Pose2d(samples[i][1], samples[i][2], pose[0].getRotation());
            monitor.update();
        }
        return monitor;
    }

    private static WpilogStallAnalyzer.Result analyze(List<double[]> trace) {
        // One rising edge before the first sample and no falling edge: a path drives throughout.
        List<double[]> fresh = new ArrayList<>();
        fresh.add(new double[] {trace.get(0)[0] - 1.0, 1.0});
        return WpilogStallAnalyzer.checkForStall(trace, fresh, kWindowSeconds, kThresholdMeters,
                kGraceSeconds);
    }

    private static List<double[]> trace(double[][] samples) {
        List<double[]> out = new ArrayList<>();
        for (double[] s : samples) {
            out.add(new double[] {s[0], s[1], s[2]});
        }
        return out;
    }

    private static double[][] stationary(double from, double to, double x, double y) {
        List<double[]> out = new ArrayList<>();
        for (double t = from; t < to - 1e-9; t += 0.02) {
            out.add(new double[] {t, x, y});
        }
        return out.toArray(new double[0][]);
    }

    private static double[][] monotonic(double from, double to, double metersPerSecond) {
        List<double[]> out = new ArrayList<>();
        for (double t = from; t < to - 1e-9; t += 0.02) {
            out.add(new double[] {t, (t - from) * metersPerSecond, 0.0});
        }
        return out.toArray(new double[0][]);
    }

    /** Drives +x for half the span at {@code metersPerSecond}, then retraces its own steps. */
    private static double[][] outAndBack(double from, double to, double metersPerSecond) {
        double midpoint = (from + to) / 2.0;
        List<double[]> out = new ArrayList<>();
        for (double t = from; t < to - 1e-9; t += 0.02) {
            double travelled = t <= midpoint
                    ? (t - from) * metersPerSecond
                    : (midpoint - from) * metersPerSecond - (t - midpoint) * metersPerSecond;
            out.add(new double[] {t, travelled, 0.0});
        }
        return out.toArray(new double[0][]);
    }
}
