# Lesson 7: ChassisSpeeds

## Learning objective
Understand `ChassisSpeeds` (vx, vy, omega) as the chassis-level command interface sitting between
"what should the robot do" and "what should each module do," and the field-relative vs.
robot-relative distinction that trips up nearly everyone the first time.

## Concepts introduced
- `ChassisSpeeds` fields (vx forward, vy left/strafe, omega counterclockwise-positive)
- `ChassisSpeeds.fromFieldRelativeSpeeds()` and the gyro-angle dependency it introduces
- `ChassisSpeeds.discretize()` — and the confirmed-absent gap in this repo's own production
  pipeline (`docs/Path_Following_Tuning_Readiness_Audit.md`: no `discretize()` call exists
  anywhere, grep-confirmed) as a live, concrete case study rather than an abstract warning

## Existing code to reuse
None directly executable yet (Lesson 6's modules are the consumer, wired in Lesson 8) — this
lesson is conceptual/math exercises plus small pure-function unit tests.

## New simulation command / demo
`academy.chassis.ChassisSpeedsConversionTest` (JUnit, no sim time needed): round-trip
field-relative → robot-relative → field-relative conversions across several gyro angles, confirm
identity within floating-point tolerance. A second test, `DiscretizeComparisonTest`, deliberately
integrates *without* `discretize()` vs. *with* it over several timesteps at high omega, to make
the "un-discretized chassis speeds drift under rotation" effect visible rather than theoretical.

## What to observe in Phoenix Tuner X
N/A — this lesson is math-only.

## What to observe in AdvantageScope
Plot the discretize-vs-not comparison's resulting trajectory (integrated x/y position over time
at constant high omega) — the non-discretized version should visibly curve away from the
discretized one, growing with omega and timestep size.

## Common failure modes
- Forgetting which axis is "forward" vs. "left" (WPILib convention: +x forward, +y left) and
  getting mirror-image behavior that looks like a sign bug elsewhere.
- Applying field-relative conversion with a stale or wrong-sign gyro angle — produces a
  drivetrain that drives correctly only along one field axis.
- Assuming `discretize()` is a nice-to-have — the audit doc's finding is that it's *silently
  absent* in production, not that it was considered and rejected; treat its absence as a real
  known gap, not a non-issue.

## Small experiments to build intuition
1. Fix a nonzero omega and vx, integrate position with and without `discretize()` across
   increasing timestep sizes — confirm the drift grows with both omega and dt (making concrete
   *why* faster rotation rates make the gap matter more).
2. Convert the same field-relative command at gyro angles of 0°, 90°, 180°, 270° and confirm the
   robot-relative result rotates exactly as expected each time.
3. Introduce a deliberately wrong gyro sign and observe which specific motion direction inverts.

## Success criteria
`ChassisSpeedsConversionTest` passes, and you can explain — using your own `DiscretizeComparisonTest`
plot, not just the audit doc's prose — why the production drivetrain's missing `discretize()`
call is a real (if currently tolerated) gap rather than a cosmetic one. Move to Lesson 8 once both
hold.
