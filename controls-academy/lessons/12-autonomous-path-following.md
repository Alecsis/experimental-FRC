# Lesson 12: Autonomous Path Following

## Learning objective
Run one real PathPlanner `.path` file end to end through `AutoBuilder` against the academy
chassis stack (Lessons 6-11 composed), and build the "did this actually complete" harness pattern
this repo already uses in production — on a much smaller, fully-understood system first.

## Concepts introduced
- `AutoBuilder.followPath()` / the PathPlanner trajectory-following command lifecycle
- Reading an existing `.path` file's waypoints/constraints directly (not authoring a new one)
- Stall detection and tracking-error measurement as a *post-hoc log analysis* problem, not
  something computed live during the run

## Existing code to reuse
`src/test/java/frc/robot/AutoRegressionTestBase.java` (and its one concrete subclass,
`LtNeutralAutoRegressionTest`) is the direct template: schedule a path-following command on a
real `CommandScheduler` thread, let it run, then post-hoc analyze the produced `.wpilog` via
`WpilogTrajectoryErrorReader`/`WpilogStallAnalyzer` against a golden. Also required reading:
`docs/claudex/architecture.md`'s "Auto regression testing pattern" section — the documented rules
(one JUnit class per auto due to singleton leakage, real time not accelerated time for anything
touching the native odometry thread, never schedule from the test thread while real time is
flowing) apply here just as much as in production.

## New simulation command / demo
`academy.auto.AcademyPathFollowingTest` — pick one small, simple existing `.path` file (read-only,
don't author a new one this lesson), run it through `AutoBuilder.followPath()` against the
academy chassis, and build your own minimal golden-comparison harness modeled on
`AutoRegressionTestBase` but scoped down (no need for the full multi-auto golden-JSON
infrastructure yet — a single hardcoded tolerance check is fine for lesson purposes).

## What to observe in Phoenix Tuner X
N/A (simulation).

## What to observe in AdvantageScope
Ghost trajectory (the `.path` file's nominal plan) overlaid against your academy chassis's actual
followed pose — this is the same visual pattern used to diagnose the real `LB Neutral`/mid-field
spawn-reset bugs documented in `docs/claudex/history.md`, applied here to a system you built and
fully understand.

## Common failure modes
- Scheduling the path-following command from the test thread while real time is flowing (the
  documented `AutoRegressionTestBase` gotcha) — produces the same intermittent
  truncated-log/spurious-`completed=true` race noted in the 16th-session history entry.
- Forgetting that stall/tracking-error analysis in this repo's pattern is *always* post-hoc log
  analysis, never a live assertion mid-run — trying to assert live tracking error inside the
  running command fights the architecture instead of using it.
- Picking a `.path` file with nontrivial event markers or named-command dependencies for a first
  attempt — start with the simplest available path, not a representative "real" auto.

## Small experiments to build intuition
1. Run the same path twice and compare tracking-error results — quantify how much run-to-run
   variance exists even before touching any gains (this is the `sim-timing-determinism.md` noise
   floor, now measured on your own harness rather than taken on faith).
2. Deliberately detune one of Lesson 11's controller gains (e.g. halve translation kP) and
   observe how the ghost-vs-actual overlay changes — connect Lesson 11's abstract saturation
   finding to a visible, concrete path-following degradation.
3. Try a path requiring a direction reversal partway through and watch for the Lesson 6
   module-optimize behavior kicking in mid-path.

## Success criteria
`AcademyPathFollowingTest` runs to completion with a tracking error you can defend as reasonable
(no fixed universal number — compare to what you observed in experiment 1's run-to-run baseline),
and you can produce the ghost-vs-actual AdvantageScope overlay unassisted. Move to Lesson 13 once
both hold.
