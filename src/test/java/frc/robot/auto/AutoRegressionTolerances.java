package frc.robot.auto;

/**
 * Placeholder pass/fail tolerances for the auto regression suite. All values here are
 * intentionally loose (see docs/superpowers/specs/2026-07-18-auto-regression-suite-design.md) --
 * the sim-timing doc already measured ~0.3m RMS run-to-run jitter on identical code, so max error
 * will scatter more than that. Tightening these is a deliberate later fast-follow, not part of
 * this milestone.
 */
final class AutoRegressionTolerances {
    /**
     * Real-time autonomous period cap -- also doubles as the run-loop timeout. The harness runs
     * in real time, not SimHooks-accelerated time (see AutoRegressionTestBase's setUp() comment),
     * so this many real wall-clock seconds actually elapse per test.
     */
    static final double kMaxRuntimeSeconds = 15.0;

    /** Real-time poll interval used by the run loop (Thread.sleep-paced, matches the robot period). */
    static final double kStepSeconds = 0.02;

    /** Rolling window (real seconds) over which the stall detector checks translation. */
    static final double kStallWindowSeconds = 1.0;

    /** Minimum translation required over kStallWindowSeconds to not be considered stalled. */
    static final double kStallTranslationThresholdMeters = 0.05;

    /**
     * Every in-scope auto ends with a stationary parallel(Orbit, Shooting Sequence) phase --
     * deliberately holding position to aim and shoot, not stuck. WpilogStallAnalyzer (post-hoc,
     * from /RealOutputs/Trajectory/SetpointFresh's last true timestamp in the wpilog) suspends the
     * stall check once this many real seconds have passed since the last fresh PathPlanner path
     * setpoint, since that means path-following has handed off to a stationary named-command
     * phase.
     */
    static final double kStalePathSetpointGraceSeconds = 0.5;

    /** Golden runtimeSeconds comparison: +/- this many seconds. */
    static final double kRuntimeToleranceSeconds = 3.0;

    /** Golden maxLateralErrorMeters comparison: golden + this much absolute headroom. */
    static final double kLateralErrorHeadroomMeters = 0.75;

    /** Golden maxLongitudinalErrorMeters comparison: golden + this much absolute headroom. */
    static final double kLongitudinalErrorHeadroomMeters = 1.0;

    private AutoRegressionTolerances() {}
}
