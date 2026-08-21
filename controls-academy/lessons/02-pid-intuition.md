# Lesson 2: PID Intuition (P, I, D)

## Learning objective
Build hands-on intuition for what each PID term does to a step response, using Lesson 1's motor
harness as the plant. Learn to recognize oscillation, overshoot, steady-state error, and
integral windup by sight, not just by formula.

## Concepts introduced
- Proportional gain and steady-state error
- Integral gain and windup (plus why WPILib/Phoenix6 closed loops need windup limits)
- Derivative gain and its damping/noise-amplification tradeoff
- Step response vocabulary: rise time, overshoot, settling time

## Existing code to reuse
`academy.motor.SingleMotorSim` from Lesson 1, unmodified plant. This lesson only changes
`Slot0Configs` values and observes the same harness.

## New simulation command / demo
`academy.pid.PidSweepTest` (JUnit): run `SingleMotorSim` across a small matrix of
(kP, kI, kD) tuples (e.g. P-only, P+I, P+D, P+I+D with one "good" and one "too high" value each),
logging each run's step response under a distinct AdvantageKit prefix
(`Academy/Pid/Sweep/<label>/...`) so all runs can be diffed in one AdvantageScope session.

## What to observe in Phoenix Tuner X
Still simulation-only — no live Tuner X signal. Defer real Tuner X PID-tuning observation to
Lesson 5 (steer, which has a live-hardware analogue in this codebase) or September (real
drivetrain).

## What to observe in AdvantageScope
Overlay all sweep runs' `MeasuredVelocity` traces on one plot. Look for:
- P-only: proportional steady-state gap (carried over from Lesson 1).
- P+I: gap closes to zero, but watch for slow oscillation if I is too aggressive.
- P+D: faster settling, but D amplifies any measurement noise — inject a small noise term into
  `SingleMotorSim`'s velocity readback for this run specifically to see it.
- Deliberately-too-high P: sustained oscillation, the textbook "found the gain that rings" case.

## Common failure modes
- Tuning D first "because it's the last letter" — D only makes sense once P (and often I) already
  gets you close; D on top of a bad P is chasing noise, not error.
- Integral windup: command a setpoint the motor genuinely cannot reach (e.g. beyond simulated
  free speed) and watch the integral term run away — this is the concrete, hands-on version of
  why real Slot0 configs need `PeakOutput`/current limits.
- Treating "zero overshoot" as always the goal — for a drivetrain, some overshoot may be
  acceptable if it settles fast; the "right" response shape depends on what's being controlled.

## Small experiments to build intuition
1. Reproduce sustained oscillation, then halve P — confirm it stabilizes (classic root-locus
   intuition without ever drawing a root-locus plot).
2. Add I to a P-only response with a real steady-state gap — confirm the gap closes to ~0 given
   enough time, and note *how much* time (this is what "the I term is slow" means concretely).
3. Force integral windup (unreachable setpoint), observe the overshoot when the setpoint later
   becomes reachable — the windup "unwind" lag is the graphic version of the bug.

## Success criteria
Given a step-response plot you've never seen before, correctly label which term (or combination)
produced it — steady-state gap, ringing, slow crawl-to-target, or windup-overshoot — for at least
3 of 4 held-out sweep runs. Move to Lesson 3 once you can do this by eye.
