# SysId Simulation Workflow Validation

> ⚠️ **SIMULATION-DERIVED PLACEHOLDER — DO NOT USE FOR REAL ROBOT CHARACTERIZATION.**
> Every `kS`/`kV` number in this document comes from a headless JUnit **simulation** run, not the
> physical robot. They are influenced by maple-sim's own unmeasured friction placeholders
> (`kDriveFrictionVoltage`/`kSteerFrictionVoltage`, `TunerConstants.java:98-99`), which makes any
> "characterization" derived from them circular by construction — the sim would just be handing
> back numbers close to what was already fed into it. **They must never be pasted into
> `TunerConstants.driveGains`, and must never be treated as production characterization.** Real
> hardware characterization is required before Phase 2 of
> `docs/superpowers/plans/2026-07-28-autonomous-velocity-migration.md` can proceed — see
> `docs/SysId_Characterization_Checklist.md` §4 for that physical procedure. This document exists
> only to prove the **analysis pipeline** (wpilog → matched telemetry → linear-regression fit)
> produces a sane, non-degenerate result when pointed at real (if simulated) data — nothing more.

## Purpose (Phase 1, Task 1.2)

Task 1.1 (`CommandSwerveDrivetrainSysIdSimWorkflowTest`, commit `7518244`) proved the SysId
**command lifecycle and telemetry capture** work in sim: the routine runs without error and
`Drive/AppliedVoltsPerModule` ramps/steps during the window. It deliberately computed no gain
value.

Task 1.2 closes the loop on the **analysis leg**: given a wpilog that already contains the right
telemetry, does the data actually support the kind of linear-regression fit SysId's own analyzer
performs? This document records that check, done manually against a real (if sim-derived) log, not
performed by the SysId analyzer tool itself.

## Source run

- Test: `CommandSwerveDrivetrainSysIdSimWorkflowTest.translationSysIdLifecycleRunsAndCapturesTelemetryInSim`
- Re-run for this task (fresh log, same test, no production code touched):
  `./gradlew test --tests "frc.robot.subsystems.CommandSwerveDrivetrainSysIdSimWorkflowTest"` — **PASSED**.
- wpilog: `logs/akit_26-07-28_11-56-19.wpilog` (3s quasistatic-forward window + 1.5s
  dynamic-forward window, translation SysId routine, per the test's own bounded windows).
- Repo state: `refactor/hybrid` at `238afff`, tree clean before this task's own commit.

## Method

A one-off analysis script (not committed — this task is documentation/analysis-only, per its own
scope) reused the project's existing `SKILLS/parse_akit_log.py` wpilog reader to:

1. Read every `/RealOutputs/Drive/AppliedVoltsPerModule` sample (`double[4]`), reducing to the
   mean absolute applied voltage across the 4 drive modules per tick.
2. Read every `/RealOutputs/DriveState/Speeds` sample (`struct:ChassisSpeeds`) and take `vx`
   (translation SysId drives straight ahead, so chassis `vx` stands in for per-module speed —
   adequate for a placeholder fit, not a substitute for SysId's own per-module analysis).
3. Pair the two by nearest timestamp (both are logged every periodic tick; all 151 voltage samples
   matched a speed sample within 20ms).
4. Fit the same two-term model SysId's own quasistatic analysis uses,
   `V = kS·sign(vx) + kV·vx`, via ordinary least squares (2×2 normal-equations solve — no external
   numerics library, consistent with this repo's existing dependency-free `SKILLS/` scripts).

This mirrors `docs/SysId_Characterization_Checklist.md`'s description of what the real Phoenix
Tuner X / WPILib SysId analyzer computes from a `.hoot` log — done here against the AdvantageKit
wpilog instead, since Task 1.1 already found (and documented) that headless sim produces no
`.hoot` file (no real CAN bus present).

## Result

| Quantity | Value |
|---|---|
| Applied-volts samples | 151 |
| Chassis-speed samples | 198 |
| Paired samples (≤20ms apart) | 151 |
| Time span analyzed | 0.002s → 3.063s |
| `vx` range | 0.0000 – 1.6133 m/s |
| Applied-volts range | 0.0000 – 4.0000 V |
| **`kS` (placeholder)** | **0.1849 V** |
| **`kV` (placeholder)** | **1.7293 V·s/m** |
| R² | 0.9198 (n=151) |

R² = 0.92 over 151 samples means the paired data cleanly supports a two-term linear fit — the
pipeline (telemetry capture → timestamp matching → regression) works end-to-end and produces a
non-degenerate result, which is this task's entire success criterion.

**Explicitly not a claim:** that 0.1849/1.7293 are numerically reasonable, close to any real value,
or usable for anything beyond proving the pipeline. `TunerConstants.driveGains` currently uses
`kS=0.1, kV=0.124` (the unedited CTRE Tuner X template default) — the sim-placeholder `kV` here is
~14× larger, which is expected and uninteresting: maple-sim's friction model and this repo's own
open-loop gear/gain assumptions have no obligation to resemble a tuned real robot, and no attempt
was made to reconcile them, per this task's own success criteria.

## What this does and does not prove

- **Proves:** the full command → telemetry → wpilog → timestamp-matched analysis pipeline is
  wired correctly and produces a well-conditioned (non-singular, high-R²) regression from real
  logged data, not synthetic/hand-built test fixtures.
- **Does not prove:** anything about the physical robot's actual `kS`/`kV`/`kA`. That requires the
  physical characterization run described in `docs/SysId_Characterization_Checklist.md` §4, which
  remains the hard blocker on Phase 2's Task 2.1 (see this document's warning banner and the
  plan's own Phase 1 TODO, both of which must survive past this point, not be quietly dropped now
  that a number exists).

## Next step

Phase 1 (Tasks 1.1 and 1.2) is now complete as scoped. Phase 2 remains blocked on physical-robot
access — its precondition (real hardware characterization) is unaffected by this document.
