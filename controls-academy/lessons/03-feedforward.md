# Lesson 3: Feedforward (kS, kV, kA)

## Learning objective
Understand feedforward as "predict the voltage this setpoint needs, before error even exists" —
and see concretely why FF+small-PID beats PID-alone for tracking a *moving* setpoint, not just
holding a fixed one.

## Concepts introduced
- `SimpleMotorFeedforward` (`kS` static friction, `kV` velocity gain, `kA` acceleration gain)
- Why FF alone can't reject disturbances (no error feedback) and PID alone lags a ramping
  setpoint (all correction, no prediction) — the two are complementary, not competitors
- Why this repo's own production `driveGains` has `kA` unset (`TunerConstants.java:32`, real gap
  the September playbook exists to close) — a live example of "FF is incomplete without real
  characterization," motivating Lesson 4

## Existing code to reuse
`academy.motor.SingleMotorSim` (L1) and `academy.pid.PidSweepTest`'s harness structure (L2) — add
a `SimpleMotorFeedforward` term computed from *hand-picked, plausible-but-fake* kS/kV/kA (real
characterization is Lesson 4's job, not this lesson's).

## New simulation command / demo
`academy.feedforward.FeedforwardComparisonTest` (JUnit): command a *ramping* velocity setpoint
(not a step) three ways — PID-only, FF-only, FF+PID — log all three under
`Academy/Feedforward/<mode>/...`.

## What to observe in Phoenix Tuner X
N/A (simulation-only, same as Lessons 1-2).

## What to observe in AdvantageScope
Track `TrackingError = Setpoint - Measured` for all three modes on a ramp. Expect: PID-only shows
a persistent lag (proportional to ramp rate divided by kP — the classic "PID always trails a
ramp" result); FF-only tracks the ramp shape well but drifts if the hand-picked kS/kV are even
slightly wrong; FF+PID (small PID gains, doing only correction) tracks tightly with minimal lag.

## Common failure modes
- Treating feedforward as "the fix" and cranking PID gains to zero — any FF error (and real FF is
  never perfect) then has nothing correcting it. FF+PID is the point, not FF-instead-of-PID.
- Picking kV from a datasheet free-speed number without accounting for gearing — kV must be in
  units consistent with what the loop actually measures and commands (rps and volts here).
- Applying kS with the wrong sign convention when the setpoint is negative — `kS` should oppose
  static friction regardless of direction, not be a fixed offset.

## Small experiments to build intuition
1. Zero out kV specifically, keep kS/kA — watch the ramp-tracking degrade proportionally to ramp
   speed (kV is exactly the "how much voltage per unit of steady velocity" term).
2. Deliberately mis-set kS too high — observe a constant offset/overshoot right at direction
   reversals, the textbook signature of over-compensated static friction.
3. Compare settling behavior on a *step* (not ramp) input across all three modes from Lesson 1/2
   — confirm FF alone doesn't fix a step's steady-state gap the way I-term does, reinforcing they
   solve different problems.

## Success criteria
Explain, in your own words, why the drivetrain's real `Slot0Configs` has both nonzero kS/kV *and*
nonzero kP (`TunerConstants.java:26-34`) rather than relying on one or the other. Move to
Lesson 4 once the ramp-tracking comparison plot makes intuitive sense without re-reading this doc.
