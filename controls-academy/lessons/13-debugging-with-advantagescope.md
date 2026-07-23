# Lesson 13: Debugging with AdvantageScope

## Learning objective
This lesson is the payoff for logging everything from Lesson 1 onward: build fluency reading
AdvantageKit `.wpilog` output directly (both via AdvantageScope and via a script), using your own
academy lessons' logs as the dataset instead of unfamiliar production data.

## Concepts introduced
- AdvantageScope layout patterns: line-graph overlays, 2D field views, table views for structured
  arrays
- Reading a `.wpilog` programmatically (entry table, timestamped values) as an alternative to the
  GUI — useful for scripted comparisons across many runs (e.g. Lesson 12's run-to-run variance
  experiment) that would be tedious by hand in the GUI alone
- The `/RealOutputs/` vs `/ReplayOutputs/` path prefix AdvantageKit applies to logged names —
  a common first-time gotcha when grepping for a signal by its `Logger.recordOutput()` name

## Existing code to reuse
`SKILLS/parse_akit_log.py` — a pure-stdlib WPILOG parser, directly importable as a Python module.
Its CLI (`<log>` for an entry summary, `--grep <term>`, `--dump <exact entry name>`,
`--pose-entry <name> --jump-threshold <m>` for frame-to-frame pose-jump analysis) is exactly the
tool for scripted analysis across Lessons 1-12's accumulated logs. Also read
`docs/claudex/design/trajectory-error-instrumentation.md` §6 for existing, already-designed
AdvantageScope layout recipes (Auto Overview, Trajectory Error, Auto Scorecard, vision cross-check)
— adapt these layouts to academy signals rather than inventing new ones from scratch.

## New simulation command / demo
No new sim demo — this lesson is a **tooling/analysis** lesson. Concretely: (1) build one saved
AdvantageScope layout per prior lesson category (motor-level, module-level, chassis-level, auto-
level), and (2) write `academy/analyze_lessons.py`, a small script built on `parse_akit_log.py`
that pulls Lesson 12's run-to-run tracking-error numbers across N repeated runs into a simple
table, without opening AdvantageScope at all.

## What to observe in Phoenix Tuner X
N/A — this lesson is exclusively about post-hoc log analysis tooling.

## What to observe in AdvantageScope
Deliberately revisit 2-3 earlier lessons' logs (e.g. Lesson 2's PID sweep, Lesson 10's vision
fusion) using only the layouts built in this lesson, without re-reading those lessons' docs first
— the test is whether the layouts alone are enough to reconstruct what happened.

## Common failure modes
- Grepping for a raw `Logger.recordOutput("Academy/Motor/...")` name and getting no results
  because the actual stored entry is prefixed `/RealOutputs/Academy/Motor/...` — the single most
  common first-time `parse_akit_log.py` mistake, per its own documented behavior.
- Building one enormous do-everything AdvantageScope layout instead of focused per-topic layouts
  — makes every debugging session start with panel-hunting instead of signal-reading.
- Treating the GUI and the script as competitors rather than complements — the GUI is for
  exploring one run visually, the script is for comparing many runs numerically; use both.

## Small experiments to build intuition
1. Given only a saved layout (no lesson doc open), reconstruct what a Lesson 2 PID-sweep run
   was testing, purely from the signal names and shapes visible.
2. Use `parse_akit_log.py --pose-entry` against Lesson 9's odometry-drift log and confirm it
   flags the slip-injected run's larger position jumps vs. the zero-slip run's near-absence of
   jumps.
3. Extend `analyze_lessons.py` to summarize Lesson 4's SysId fit-vs-injected-truth error across
   several noise-level runs in one table, rather than eyeballing each log individually.

## Success criteria
You can navigate to any prior lesson's key signal in AdvantageScope in under a minute without
re-reading that lesson's doc, and `analyze_lessons.py` produces a numeric comparison across at
least two lessons' repeated runs. Move to Lesson 14 once both hold.
