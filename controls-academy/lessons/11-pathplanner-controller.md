# Lesson 11: PathPlanner Controller

## Learning objective
Instantiate `PPHolonomicDriveController` standalone against Lesson 6's module stack, and
reproduce — as a guided, hands-on exercise rather than a read-only case study — the 16th-session
finding that this repo's own production controller's feedback term is unclamped and can command
speeds far beyond the chassis's real ceiling.

## Concepts introduced
- `PPHolonomicDriveController` (feedforward + feedback split: `calculateRobotRelativeSpeeds()`)
- Why PathPlanner's target-state feedforward comes from the *trajectory plan itself*
  (`targetState.feedforwards`), not from the controller's own output — a distinction that matters
  when reasoning about where an oversized command originates
- Unclamped proportional feedback: `kP × position_error` has no inherent ceiling, so a large
  enough position error produces an arbitrarily large commanded speed unless something downstream
  bounds it

## Existing code to reuse
`utility/LoggingHolonomicDriveController.java` — read this as the reference: it's a
`PPHolonomicDriveController` subclass that logs `Trajectory/CommandedFeedforwardSpeeds`/
`CommandedFeedbackSpeeds`/`CommandedTotalSpeeds` as an *exact* algebraic split. Also read
`CommandSwerveDrivetrain.java:364-368` for the actual production gains
(`PIDConstants(5,0,0)` translation, `PIDConstants(3,0,0)` rotation) and
`CommandSwerveDrivetrain.sanitizeAutoSpeeds()` (Lesson 8) for how the *output* of this unclamped
controller is currently bounded downstream rather than the controller itself.

## New simulation command / demo
`academy.auto.ControllerSaturationTest` (JUnit): instantiate a `PPHolonomicDriveController` with
the same `kP=5`/`kP=3` gains against the academy chassis, inject a large position error (e.g. a
1.5m+ tracking offset, matching the crossover math `kSpeedAt12Volts / kP` from the 16th-session
finding), and confirm the raw feedback term alone exceeds the chassis's true max speed —
reproducing the finding on your own numbers rather than trusting the doc.

## What to observe in Phoenix Tuner X
N/A (pure controller math + Lesson 6's simulated modules).

## What to observe in AdvantageScope
Split-out feedforward/feedback/total speed traces (mirroring `LoggingHolonomicDriveController`'s
signal names) across a range of injected position errors — identify visually where total crosses
the chassis's real speed ceiling, and confirm it's overwhelmingly the feedback term doing it, not
feedforward.

## Common failure modes
- Concluding "just lower kP" without checking what tracking behavior that trades away — this
  lesson's job is to *characterize* the saturation, not prescribe a fix; a real fix needs the
  traction-ceiling context from Lesson 4-ish's dynamics (or the actual μ figure from the real
  robot, not academy-invented).
- Forgetting that PathPlanner's rotation feedback is structurally bounded (heading error wraps to
  ±π) while translation feedback is not — don't expect the same saturation behavior from both.
- Testing only "reasonable" position errors and missing the saturation entirely — the whole point
  is to inject an error large enough to expose it, matching how the production finding was
  actually discovered (via a real large-tracking-error run, not a nominal one).

## Small experiments to build intuition
1. Reproduce the crossover point algebraically (`chassisMaxSpeed / kP`) and confirm it matches
   where your own `ControllerSaturationTest` first sees total speed exceed the ceiling.
2. Compare feedback-only magnitude across a range of position errors from small to large —
   confirm it scales linearly with error (proportional control, no surprises) right up to and
   past the point where that's physically nonsensical.
3. Repeat the same exercise for rotation error and confirm it never exceeds the chassis's true
   rotational max, contrasting the bounded-by-wrapping rotation case against the unbounded
   translation case.

## Success criteria
`ControllerSaturationTest` demonstrates the same qualitative finding as the 16th session's
production investigation, on your own numbers, and you can state the crossover formula from
memory. Move to Lesson 12 once both hold.
