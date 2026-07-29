# Autonomous Observability Phase 1 — Validation Plan

**Date:** 2026-07-28
**Purpose:** Define the validation phase that must happen **between** "the telemetry plumbing exists"
(Phase 0.5, complete) and "a recovery decision is allowed to read that telemetry and act" (an
unnamed, not-yet-scoped future phase). This document proposes **no recovery actions** — no command
cancellation, no `pathfindToPose` trigger, no vision-backoff implementation. Its only job is to
define what evidence would have to exist before any of that becomes safe to design, per the audit-
first philosophy this project has followed since the Recovery Audit.

**A naming note, so a future session doesn't confuse the two:** this repo already has an unrelated
"Phase 1" — the Autonomous Velocity migration plan's sim-only SysId workflow validation
(`docs/superpowers/plans/2026-07-28-autonomous-velocity-migration.md`), complete and committed. This
document's "Phase 1" is a different track entirely: the recovery/observability thread that started
with the Recovery Audit and produced `AutonomousHealthMonitor` (Phase 0.5). Both tracks happen to
converge on the same underlying blocker (the chassis-PID divergence bug, §5 below) — that is a real
dependency, not a naming coincidence, and is called out explicitly where it matters.

**Inputs synthesized (no new investigation performed):**
- `docs/Autonomous_Recovery_Readiness_Assessment.md` — the go/no-go synthesis that concluded "not
  ready," and the source of the four `Auto/Health/*` placeholder thresholds' own caveats.
- `docs/Autonomous_Observability_Phase0_5_Audit.md` — this session's own prior verification that
  Phase 0.5 is complete, in-scope, and behavior-neutral.
- `docs/Autonomous_Recovery_Audit.md` — findings F1-F13, the 4-layer framework sketch, and the
  regression-test plan sketch (§"Regression test plan," not yet written).
- `docs/Autonomous_Disturbance_Simulation_Report.md` — the four-trial MapleSim experiment and its
  Findings 1-4.

---

## 1. What signals need validation before they can drive decisions?

Every signal `AutonomousHealthMonitor` (Phase 0.5) already computes or could plausibly consume,
carried over from the Readiness Assessment's own trust table (§2) plus the two audits it synthesizes:

| Signal | Currently computes correctly? | Currently trustworthy as a decision input? |
|---|---|---|
| `Trajectory/Error{Lateral,Longitudinal,Heading}` → `Auto/Health/TrackingDegraded` | Yes (Recovery Audit F4: arithmetic not in question) | **No** — Disturbance Report Finding 1: the undisturbed control run alone produces 3.10m of lateral error. A threshold that must also ignore 3m of "normal" noise cannot usefully detect a real disturbance. |
| Rolling-window pose delta → `Auto/Health/Stalled` | Yes (ported from `WpilogStallAnalyzer`, TDD-verified in Phase 0.5) | **Unproven at the chosen threshold** — `kStallTranslationThresholdMeters=0.05m`/`kStallWindowSeconds=1.0s` were copied from `AutoRegressionTolerances` as placeholders, and that same threshold is independently documented (Readiness Assessment §3.4) as sitting on a razor's edge in `LtNeutralAutoRegressionTest` (margins as tight as 0.025m-0.049m against 0.050m, repeatedly, across many sessions). |
| `Vision/RejectedJumpMeters` → `Auto/Health/VisionUnhealthy` | Yes, as a per-cycle fact (Recovery Audit F5) | **Partially** — Disturbance Report Finding 3 gives it real numbers for the first time (242 consecutive rejections on a 2.0m hit, zero on 0.4m/1.0m hits), but the medium (1.0m) trial landed "right at" the gate threshold as measured against the estimate's own drift — meaning it is not yet proven whether ordinary PID-driven drift alone (no real hit) could also trigger the same streak. |
| `Auto/EndReason`/`Auto/EndedInterrupted` (command completion) | Yes, cross-validated (Lifecycle Audit continuation, gate 3) | **No, as a success proxy** — Disturbance Report Finding 2: all four displacement magnitudes (0/0.4/1.0/2.0m) report `completed=true` at identical elapsed time. Completion and correctness are proven independent variables. |
| `hasGamePiece` (sim) | Yes, populated from maple-sim ground truth | Untested as a decision input — currently unread by any production code at all (Recovery Audit F1); not yet a "signal" in the observability sense, just an unwired field. |
| `Shooter.isJammed()` | Yes, structurally proven via `Intake`'s identical pattern | Untested as a decision input for the same reason — never called from `Superstructure.SHOOTING` (F3). |

**The unifying problem, not per-signal:** every threshold-based signal above is downstream of the
same chassis-PID divergence bug (Readiness Assessment §1, Disturbance Report Finding 1). Validating
any one signal in isolation, without first re-baselining against a fixed or bounded controller, risks
tuning a threshold against noise rather than against real disturbance.

## 2. What experiments prove each signal is trustworthy?

Each experiment below reuses an already-built harness — no new test infrastructure is proposed,
consistent with the "smallest safe fix" pattern this project has followed throughout (Recovery Audit
F3/F4's own recommendations).

### 2a. Chassis PID validation

**Experiment:** re-run `PathDisturbanceSimTestBase`'s existing `NoDisturbanceControlTest` (the
zero-displacement control from the Disturbance Report) immediately after whatever chassis-PID fix or
bound lands (clamped `PPHolonomicDriveController` output, retuned gains, or both — the specific fix
is out of this document's scope; see §5).
**Trustworthy result:** the control run's final lateral error drops from the documented 3.10m to a
value small enough that it stops dominating the `TrackingDegraded` threshold — i.e., an order of
magnitude closer to the ~0.03m error the same run shows immediately after path-start, before the
oscillation grows (Disturbance Report, Finding 1's own description of the error trace).
**This experiment cannot be run yet** — it is gated on the fix itself, which is gated on physical-
robot SysId access (`CLAUDE.md`'s standing note). Recorded here so it's ready to execute the moment
that access exists, not designed from scratch then.

### 2b. Disturbance simulation rerun

**Experiment:** re-run all four `PathDisturbanceSimTestBase` subclasses
(`NoDisturbanceControlTest`/`SmallDisplacementDisturbanceTest`/`MediumDisplacementDisturbanceTest`/
`SevereDisplacementDisturbanceTest`) unmodified, against the post-fix chassis PID, and re-analyze with
the same scratchpad methodology the original report used (`SKILLS/parse_akit_log.py`'s parser,
timestamp-paired channels).
**Trustworthy result, per signal:**
- Tracking error: the three disturbed runs (0.4/1.0/2.0m) should now show a **monotonic** relationship
  between injected displacement and peak post-injection error — larger shove, larger and/or
  longer-lived error. The original report's runs did not show this (medium's 3.47m exceeded severe's
  0.36m), which the Readiness Assessment and this report both attribute to the PID bug rather than to
  a real property of disturbance magnitude.
- Finding 4 specifically requires this rerun to resolve: does the severe (2.0m) run's near-zero final
  error reproduce (suggesting some real recovery mechanism worth understanding) or disappear (confirming
  it was an artifact of the chaotic pre-fix controller interacting with `setSimulationWorldPose()`'s
  velocity-reset side effect, as hypothesized)? The Disturbance Report's own authors explicitly declined
  to conclude either way pending exactly this rerun.
**This experiment is also gated on 2a landing first** — re-running against the same broken baseline
would just reproduce the original report's confound.

### 2c. Trajectory error thresholds

**Experiment:** once 2a/2b establish what a "normal, undisturbed" run's error trace looks like
post-fix, feed that data (plus the four post-fix disturbance runs) into the same kind of threshold
derivation `AutoRegressionTolerances` already uses for its golden-file tolerances — i.e., set
`kTrackingDegradedLateralMeters`/`kTrackingDegradedLongitudinalMeters` from an actual distribution of
observed "normal" error, not copied wholesale from a test-suite tolerance file that was itself tuned
for a different purpose (regression-golden matching, not live disturbance detection).
**Trustworthy result:** the threshold should sit clearly above the post-fix control run's peak error
and clearly below the smallest disturbance magnitude's peak error, with visible separation — not
adjacent/overlapping ranges the way the current 1.0m placeholder sits ambiguously close to the
Disturbance Report's original medium-trial numbers.

### 2d. Vision rejection behavior

**Experiment:** re-run the same four-trial disturbance harness post-PID-fix and re-measure the
medium (1.0m) trial specifically — the one case that "landed right at" the gate threshold in the
original run (Disturbance Report, Finding 3). Also worth a fifth, currently-missing trial: a **no-
displacement run under sustained wheel slip or intentionally noisy odometry**, to test whether PID-
driven drift alone (with no real hit at all) can independently push the estimate far enough from
vision's read to trigger the same rejection streak — the specific risk the Readiness Assessment
flags in §3.2 but says "wasn't isolated as its own experiment."
**Trustworthy result:** confirms whether `Vision/RejectedJumpMeters`'s existing 1.0m gate, and any
future consecutive-rejection counter built on top of it (F5), can be set to a value that distinguishes
"real hit" from "PID noise" — the entire premise a future backoff design (F5) depends on.

### 2e. Mechanism success verification (F1/F3)

**Experiment:** these two are explicitly the Readiness Assessment's own "not prerequisites — can
start independently" carve-out (§4, last paragraph) and the Recovery Audit's own recommended
first/second implementation items (§"Recommended implementation order," items 2-3). Unlike 2a-2d,
they do **not** depend on the chassis-PID fix at all — they are pure subsystem-internal signals.
- **F1 (intake):** a sim-only test spawning a fuel piece in intake range (reusing the same
  `IntakeSimulation` `IntakeIOSim.java` already exercises), asserting `hasGamePiece` flips at the
  expected moment. This validates the *signal*, not a recovery action — it is the same "plumbing,
  not decision" distinction Phase 0.5 already established for `TrajectoryErrorTracker`.
- **F3 (shooter jam):** mirror `Intake`'s existing jam-recovery test pattern
  (`forceJamConditionForTest`/`clearJamOverrideForTest`, `Intake.java:168-177`) ported to `Shooter`,
  proving `Shooter.isJammed()` flips correctly under a forced-jam condition.
**Trustworthy result:** both signals flip correctly and are logged (AdvantageKit, not just
`SmartDashboard` — closing the gap F3 flags: `Shooter/Index Stall` today is dashboard-only and never
reaches a wpilog). Neither experiment requires touching `Superstructure`'s state-machine transitions
themselves yet — only proving the signal, matching this whole plan's audit-first, validate-before-wire
philosophy.

### 2f. Replay-based validation workflow

None of the above requires new sim infrastructure — every harness this plan proposes reusing
(`PathDisturbanceSimTestBase`, `AutoRegressionTestBase`, `WpilogTrajectoryErrorReader`,
`WpilogStallAnalyzer`, `parse_akit_log.py`) already exists and is already proven. What's missing is a
**repeatable comparison workflow**: a script or documented procedure that takes two `.wpilog` files
(pre-fix baseline, post-fix rerun) and produces the same kind of before/after table the Disturbance
Report built by hand via a scratchpad script. This does not need to be a production tool — a
documented, reproducible scratchpad recipe (as the original report already provides in its own
"Reproducing this experiment" section) is sufficient, and keeps this validation phase itself
audit-only rather than growing new permanent tooling surface. Recommendation: extend the existing
scratchpad script (not committed, per the original report's own choice) rather than build a new one,
so 2a-2d can each be answered by re-running one script against two log sets.

## 3. What telemetry is missing?

Cross-referencing Phase 0.5's actual output (`docs/Autonomous_Observability_Phase0_5_Audit.md` §3)
against what §2's experiments need:

- **Ground-truth pose is not logged anywhere.** The Disturbance Report explicitly worked around this
  (test-side stdout prints, not AdvantageKit) because adding it would have meant touching production
  `CommandSwerveDrivetrain.periodic()`, out of scope for an evidence-only session. Any future
  before/after comparison (§2b) still needs this workaround, or a deliberate, minimal, explicitly-
  scoped addition of a `Drive/SimGroundTruthPose` output gated behind `Utils.isSimulation()` (never
  reaches real hardware, so it cannot become a false decision input the way a real-robot signal could).
  **Not proposed as an implementation item here** — flagged as a gap this plan's experiments will hit,
  for the mentor to decide on before §2 begins.
- **No consecutive-rejection counter exists yet for vision** (Recovery Audit F5, Disturbance Report
  Finding 3's own language: "nothing consumes `Vision/RejectedJumpMeters` at runtime"). Phase 0.5's
  `AutonomousHealthMonitor.consecutiveVisionRejections` **does** already exist as of Phase 0.5 — this
  is not a gap, it's already-built plumbing; noted here so a future reader doesn't re-propose it.
- **No AdvantageKit-logged shooter-jam signal** — `Shooter/Index Stall` is `SmartDashboard`-only
  (Recovery Audit F3), so it can't be reviewed post-match or regression-tested the way every other
  signal in this project can. Needed before §2e's F3 experiment can produce reviewable evidence.
- **No live `hasGamePiece` consumer or logged pass/fail intake outcome** — the field exists in sim
  (F1) but nothing reads or logs a derived "did intake succeed" boolean. Needed for §2e's F1
  experiment to produce more than an internal test assertion.
- **No recovery-action-attribution use yet** — Phase 0.5 already added `AutoEndReason.
  RECOVERY_CANCELLATION` as a reserved enum value (Phase 0.5 Audit §3/§6), correctly unused. Nothing
  further needed here until an actual recovery action exists — listed for completeness, not as a gap.

## 4. What regression tests are required?

Following the Recovery Audit's own sketched plan (§"Regression test plan," never written) and this
project's established RED-then-GREEN convention:

| Test | Purpose | Depends on |
|---|---|---|
| Re-run of `PathDisturbanceSimTestBase`'s 4 subclasses, post-PID-fix | Produces the §2a/§2b evidence | Chassis-PID fix landing |
| A new "PID-noise-only" 5th disturbance-harness variant (no injected shove, deliberately noisy/degraded odometry input if feasible, or an extended-duration no-disturbance run) | Answers §2d's open question: can drift alone trip the vision gate? | Chassis-PID fix landing (same confound as 2a/2b) |
| `AutonomousHealthMonitorTest` threshold tests, re-tuned | Once §2c derives real thresholds, the existing 14-test suite's threshold-boundary tests (`trackingDegradedWhenLateralErrorExceedsThreshold()` etc.) need their expected constants updated to match — mechanical, not a new design | §2c |
| A shooter-jam analog to `Intake.java:168-177`'s test-support pattern, plus a `ShooterJamTest`-equivalent | Validates F3's signal per §2e | None — can start immediately, in parallel with everything else |
| An intake-success sim test using real `IntakeSimulation` | Validates F1's signal per §2e | None — can start immediately, in parallel with everything else |
| **Non-regression:** confirm `AutonomousHealthMonitorTest.updateNeverSchedulesOrCancelsCommands()` still passes after any threshold retuning | Re-proves Phase 0.5's core safety property survives this validation phase's own changes | Any change to `AutonomousHealthMonitor` |

None of these tests are proposed as new infrastructure — all extend existing, already-proven test
classes or existing harness patterns (`PathDisturbanceSimTestBase`, `Intake`'s own jam-test support
methods). This matches the Recovery Audit's own guidance (§"Regression test plan") that F1/F3/F4/F5
each have a proven template to reuse rather than inventing a new one.

## 5. What prerequisites block Phase 1 recovery behavior?

Restating and refining the Readiness Assessment's §4 dependency order, now specifically in terms of
*this validation phase's* exit criteria rather than the original assessment's more general framing:

1. **Chassis-PID divergence fix or bound** — hard-blocks §2a, §2b, §2c, and half of §2d (the
   PID-noise-only vision experiment). Itself hard-blocked on physical-robot SysId access
   (`CLAUDE.md`'s standing "Autonomous Velocity migration" track, Phase 2). **This is the single
   dependency every other item in this plan traces back to**, exactly as the Readiness Assessment
   already concluded — this document does not change that conclusion, only operationalizes what
   happens once it's resolved.
2. **A repeatable replay/comparison workflow (§2f)** — needed before any of §2a-2d's "before/after"
   claims can be made rigorously rather than by eyeballing two log dumps. Low effort (extend an
   existing scratchpad script), no dependency on (1) — **can start now**.
3. **F1/F3 signal validation (§2e)** — no dependency on (1) at all. **Can start now**, and is
   explicitly recommended first by both the Recovery Audit (§"Recommended implementation order," items
   2-3) and the Readiness Assessment (§4, final paragraph).
4. **Ground-truth telemetry decision (§3's first bullet)** — a small scoping decision, not blocked on
   anything, but needs an explicit mentor call before any code is written (even sim-only,
   `Utils.isSimulation()`-gated logging is a production-file change, which this plan's own instruction
   set excludes from being decided unilaterally here).

**Everything else — any actual recovery *action* (cancel a command, invoke `pathfindToPose`, alter
the vision gate's trust) — remains blocked on all of the above**, and additionally on a dedicated
design/brainstorming pass for each of Layer 3 (F5's backoff N/window) and Layer 4 (F8/F9's supervisor
trigger), exactly as both the Recovery Audit and Readiness Assessment already concluded. This plan
does not shorten that list — it only breaks "wait for the PID fix" into concrete, executable steps for
the moment that fix lands, plus identifies the two items (F1/F3 signal validation, the replay
workflow) that don't need to wait at all.

---

## Staged implementation order

```
Stage A (no dependency — can start immediately, in either order or in parallel):
  A1. Build/extend the replay-comparison scratchpad workflow (§2f)
  A2. F1 signal validation: intake hasGamePiece sim test (§2e)
  A3. F3 signal validation: Shooter.isJammed() test + AdvantageKit-logged jam signal (§2e, §3)

Stage B (blocked on physical-robot SysId access / chassis-PID fix landing):
  B1. Chassis-PID validation experiment (§2a) — confirms the fix actually bounds the control run
  B2. Disturbance simulation rerun, all 4 magnitudes (§2b) — requires B1 to have landed
  B3. Vision-rejection PID-noise experiment + medium-trial re-measurement (§2d) — requires B1

Stage C (requires Stage B's data):
  C1. Derive real TrackingDegraded thresholds from B1/B2's observed "normal" distribution (§2c)
  C2. Re-tune AutonomousHealthMonitorTest's threshold-boundary tests to match (§4)
  C3. Resolve Disturbance Report Finding 4 (does the severe-run anomaly reproduce post-fix?)

Stage D (requires A + C, still NOT recovery-behavior design):
  D1. Synthesize a second, post-validation readiness assessment — same format as the original,
      answering whether thresholds are now trustworthy enough to gate an actual design pass for
      F5 (vision backoff) and Layer 4 (pathfind-based supervisor). This is a documentation/synthesis
      step, matching how this project has always transitioned from "audit" to "design decision" —
      it does not itself design any recovery action.
```

Stage A has no ordering constraint relative to Stage B — both can run in parallel today. Stage C
cannot start before Stage B completes. Stage D cannot start before both A and C are done.

## Dependencies

```
Chassis-PID fix (external, hardware-blocked)
        │
        ├──► B1 (PID validation experiment)
        │        │
        │        ├──► B2 (disturbance rerun) ──┐
        │        └──► B3 (vision-noise exp.) ──┤
        │                                       ▼
        │                                  C1 (real thresholds)
        │                                       │
        │                                       ├──► C2 (retune existing tests)
        │                                       └──► C3 (resolve Finding 4)
        │
A1 (replay workflow) ─────────────────────────────┐
A2 (F1 signal test)  ──────────────────────────────┼──► D1 (post-validation readiness assessment)
A3 (F3 signal test)  ──────────────────────────────┘         │
                                                                ▼
                                                   (future, separate phase: F5/Layer-4 design —
                                                    explicitly NOT part of this plan)
```

## Acceptance criteria

- **Stage A complete when:** F1/F3 signals are proven correct by a passing test each, both logged via
  AdvantageKit (not `SmartDashboard`-only), and the replay-comparison workflow can take two `.wpilog`
  paths and reproduce the Disturbance Report's original results table from the original four logs
  (a self-check: if it can't reproduce known-good numbers, it isn't trustworthy for new ones).
- **Stage B complete when:** all four disturbance-harness subclasses have been re-run against the
  post-fix controller, and the control run's peak lateral error is small enough that it no longer
  dominates any plausible `TrackingDegraded` threshold (qualitatively: an order of magnitude below the
  original 3.10m, not a specific number this plan should not pre-commit to before seeing real data).
- **Stage C complete when:** `kTrackingDegradedLateralMeters`/`kTrackingDegradedLongitudinalMeters`
  are set from an actual observed distribution rather than the current 1.0m placeholder, with visible
  separation between "normal" and "smallest tested disturbance" error ranges, and
  `AutonomousHealthMonitorTest`'s threshold tests pass against the new constants without weakening
  `updateNeverSchedulesOrCancelsCommands()`'s guarantee.
- **Stage D complete when:** a written, evidence-based synthesis document exists (mirroring
  `Autonomous_Recovery_Readiness_Assessment.md`'s own format) stating explicitly whether the
  validated signals are now trustworthy enough to begin a *design* pass (not implementation) for F5
  and Layer 4 — the same "audit before design before implementation" sequencing this project has used
  for every recovery-adjacent decision so far.

## Risks

- **The chassis-PID fix might not fully resolve the divergence, only reduce it.** If Stage B's
  control run still shows meaningful (if smaller) error, Stage C's threshold derivation becomes
  harder, not automatic — the plan's "order of magnitude" acceptance criterion may need revisiting
  with the mentor rather than assumed.
- **`setSimulationWorldPose()`'s velocity-reset side effect (Finding 4's leading hypothesis) may
  resurface as a confound in Stage B/C3 even after the PID fix**, since it's a property of the
  injection method, not the controller. If Finding 4 doesn't cleanly resolve, Stage C3 may need its
  own follow-up rather than a clean answer.
- **Sim-only validation may not transfer to real hardware.** Every experiment in this plan (§2a-2d,
  and F1/F3's sim half) runs in MapleSim. The Readiness Assessment already flags real-hardware
  intake/shot detection (F1-hardware, F2) as LIMITATIONs requiring new sensors, not validated by
  anything in this plan — Stage A2 specifically validates only the sim signal, and this document does
  not claim otherwise.
- **Reusing `AutoRegressionTolerances`-derived thresholds risks importing existing flakiness.** The
  Readiness Assessment (§3.4) already flagged that `LtNeutralAutoRegressionTest`'s stall check runs on
  a razor's edge (0.025m-0.049m margins against a 0.050m threshold). If Stage C1 derives new
  thresholds using a similar methodology without deliberately building in more margin, the same
  flakiness class could propagate into a live, in-match monitor — which, unlike a test suite, cannot
  be safely reran until it happens to pass. Stage C1 should treat this as a design constraint, not
  just copy the test suite's existing tolerance file.
- **Scope creep risk: Stage A's F1/F3 work is signal validation, not the wiring the Recovery Audit's
  Layer 2 eventually calls for.** It would be easy to drift from "prove `hasGamePiece` flips
  correctly" into "wire it into `Superstructure.INTAKING`'s early-exit" mid-task. This plan explicitly
  scopes Stage A to signal validation only — wiring a validated signal into a state-machine decision is
  itself a Layer-2 implementation step requiring its own explicit go-ahead, per the same audit-first
  discipline this whole document is trying to preserve.
- **This plan itself could be read as pre-approving Stage B/C the moment the PID fix lands.** It is
  not — every stage above still requires the mentor's explicit go-ahead to begin, per this session's
  own instruction ("do not implement recovery behavior," "do not design recovery actions yet"). This
  document defines *what* validation would look like, not permission to start it.

## What this plan does not do

- Does not implement any code — no production file, no new test file, no threshold value change.
- Does not design F5's vision-backoff N/window value, or Layer 4's supervisor trigger — both remain
  explicit future design-pass items, unchanged from the Recovery Audit's and Readiness Assessment's
  own conclusions.
- Does not fix, retune, or schedule a fix for the chassis-PID divergence bug — that remains a
  separate, already-tracked, hardware-blocked item (`CLAUDE.md`'s "PID retune milestone" /
  Autonomous Velocity migration Phase 2).
- Does not commit to specific threshold numbers for Stage C — deliberately, since the whole point of
  Stage B/C is to derive them from real post-fix data rather than guess them now.
- Does not revise the Recovery Audit's 4-layer framework — this plan operates entirely within what
  that framework already calls "Layer 1" (tracking-error consumer) and the two independent Layer-2
  signals (F1/F3), and treats Layers 3/4 as still requiring their own future design pass, unchanged.
