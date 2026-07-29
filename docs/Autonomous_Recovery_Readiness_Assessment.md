# Autonomous Recovery Readiness Assessment

**Date:** 2026-07-28
**Purpose:** Determine whether the robot is ready for an autonomous recovery layer (an
`AutonomousHealthMonitor` or equivalent), by synthesizing three already-complete, audit-only
documents. This assessment **implements nothing**, **designs nothing** — the Recovery Audit
already sketched a 4-layer framework and this document does not revise it. The only job here is
readiness: what has to be true before any of that framework is safe to build.

**Inputs synthesized (no new investigation performed):**
- `docs/Autonomous_Command_Lifecycle_Audit.md` — command/NamedCommand termination correctness
  (both the original pass and its same-day continuation).
- `docs/Autonomous_Recovery_Audit.md` — 13 findings (F1-F13) on failure/disturbance detection gaps.
- `docs/Autonomous_Disturbance_Simulation_Report.md` — 4-trial MapleSim disturbance experiment
  (0/0.4/1.0/2.0m lateral shove) with real wpilog evidence.

Every claim below is a citation back to one of those three documents (or, transitively, to the
source files they already traced) — nothing here is a new code-reading pass.

---

## Verdict

**Not ready.** One confirmed, pre-existing, out-of-scope bug — the unclamped `kP=5`/`kP=3`
feedback term in `PPHolonomicDriveController` (first documented in `CLAUDE.md`'s sixteenth-session
findings, empirically reconfirmed by the Disturbance Report's own control run) — currently
produces **3.10m of lateral tracking error on a single undisturbed 6.35s path segment**
(Disturbance Report, Finding 1). Every threshold-based signal a recovery layer would need to read
(tracking error, vision-rejection streak) is downstream of this same instability. Building
threshold logic on top of it today would not distinguish "the robot was hit" from "the robot
always does this" — the single largest readiness gap identified across all three source documents.

This is not a new finding — it is the same bug `CLAUDE.md`'s "PID retune milestone" bullet already
tracks as blocked pending SysId/`Slot0` work — but this assessment is the first place it's been
evaluated specifically as a blocker for *recovery infrastructure*, not just for tracking accuracy.

---

## 1. What problems must be solved before recovery behavior is safe to implement?

**Primary blocker — chassis-PID divergence.** `PPHolonomicDriveController.calculateRobotRelative
Speeds()` sums feedforward and feedback with zero clamping (confirmed from decompiled
PathplannerLib 2026.1.2 source, cited in `CLAUDE.md`'s sixteenth-session findings). The Disturbance
Report's **zero-displacement control run** — no injected disturbance at all — grows from ~0.03m to
3.10m of lateral error over one path segment, with a visibly oscillatory (not monotonic, not
convergent) error trace. This reproduces the already-known, already-out-of-scope bug; it is not new.
What *is* new here is the implication for recovery: any "tracking degraded" or "stalled" threshold
a health monitor reads will trip on ordinary undisturbed operation, not just on real disturbances.
A monitor built today would alarm constantly and provide no signal, or would need a threshold set
so loose (to avoid false positives against 3m of baseline noise) that it would miss real,
smaller-magnitude disturbances entirely. **This must be fixed, or at minimum characterized and
bounded, before any threshold-based trigger can be trusted.**

**Secondary problem — the only disturbance-recovery evidence gathered so far is confounded by the
primary blocker.** Disturbance Report Finding 4 (the severe/2.0m run's tracking error collapsing to
near-zero instead of growing) is explicitly flagged by its own authors as unresolved — most likely
an artifact of `setSimulationWorldPose()`'s velocity-reset side effect interacting with the
already-chaotic PID behavior, not genuine disturbance tolerance. **Any threshold tuned against
today's disturbance-experiment data would be tuned against unreliable data.** The report's own
recommendation — re-run the identical experiment once the chassis-PID bug has a real fix — is a
precondition, not an optional follow-up.

**Tertiary problem — recovery actions need to be distinguishable from other interruptions.**
Neither source document flags this directly, but it falls out of combining them: `Robot.
wrapAutonomousForTelemetry()`'s `Auto/EndedInterrupted` (Lifecycle Audit, continuation #4) records
*that* a command was interrupted, not *why*. Today the only interrupters are `teleopInit()`'s mode
transition and `testInit()`'s `cancelAll()` (both benign, already-understood cases). The moment a
health monitor starts cancelling `FollowPathCommand`s or NamedCommands autonomously, that same
boolean will fire for a third reason with no way to tell them apart after the fact — nor will it be
distinguishable from a human clicking one of the still-live SmartDashboard command buttons
mid-auto (Lifecycle Audit's "lower-severity" finding on `RobotContainer.dashboard()`). Before a
monitor can act, its own actions need their own telemetry tag, or every future post-match analysis
of `Auto/EndedInterrupted` becomes ambiguous between "recovery intervened," "a human clicked
something," and "the match phase changed."

## 2. Which telemetry signals are trustworthy enough to drive decisions?

| Signal | Trustworthy today? | Basis |
|---|---|---|
| `Trajectory/ErrorLateralMeters`/`ErrorLongitudinalMeters`/`ErrorHeadingRadians` (`TrajectoryErrorTracker`) | **Measurement: yes. Decision-threshold: no.** | Computed correctly every cycle (Recovery Audit F4) — the arithmetic isn't in question. But as a *decision input* it's unusable until the chassis-PID baseline stops producing 3m of "normal" error (Disturbance Report Finding 1) — see §1. |
| `Vision/RejectedJumpMeters` | **Yes, as a per-cycle fact. No, as a standalone trigger.** | Accurately reports each rejection (Recovery Audit F5, Disturbance Report Finding 3 — 242 consecutive rejections measured precisely). But it's a raw per-cycle value with no consecutive-rejection counter or duration tracking today — a monitor would have to build that aggregation itself, and would inherit the same PID-driven false-positive risk if large tracking error (not a real hit) ever drives odometry far enough from vision's read to trip the same gate (see §3). |
| `Drive/AppliedVoltsPerModule`/`StatorCurrentAmpsPerModule` | **Yes, as raw telemetry.** | Logged live every cycle (Recovery Audit F4/F6), already used successfully for offline SysId curve-fitting and this same disturbance experiment's own current-vs-stall check (Disturbance Report Finding 4's ruled-out hypothesis). No live consumer exists yet, but the signal itself has already been cross-validated by an independent tool (`parse_akit_log.py`) multiple times this project. |
| `Auto/Running`/`Auto/EndedInterrupted` | **Yes, for what it currently claims — interrupted vs. not.** | Independently cross-validated against a second tool (Lifecycle Audit continuation #4, gate 3). Not trustworthy as a *success* signal, though — see §1's tertiary point and Disturbance Report Finding 2. |
| `hasGamePiece` (Intake, sim only) | **Yes, in sim.** | Populated from maple-sim's real ground-truth (`IntakeSimulation.getGamePiecesAmount()`, Recovery Audit F1). Never populated on real hardware — not trustworthy there because it doesn't exist there, not because it's wrong. |
| `FollowPathCommand`'s own completion state | **Trustworthy for what it is — "did the timer elapse" — untrustworthy as a proxy for success.** | Confirmed structurally (F7, decompiled source) and empirically (Disturbance Report Finding 2: identical `completed=true` at identical elapsed time across 0m-2m of displacement). Any monitor must treat command completion and tracking success as two separate signals, never one standing in for the other. |
| `SmartDashboard.putBoolean("Shooter/Index Stall", ...)` | **No.** | Dashboard-only (Recovery Audit F3) — never reaches AdvantageKit/wpilog, so it can't be reviewed post-match or read by anything that consumes `Logger` outputs the way every other signal above does. |

## 3. Which existing issues would create false recovery triggers?

1. **The chassis-PID divergence bug itself (§1) is the dominant false-trigger source.** A
   tracking-error threshold tuned to catch real disturbances would, on today's evidence, fire on
   essentially every autonomous run regardless of whether anything hit the robot — the control run
   alone crossed 3m with zero disturbance (Disturbance Report Finding 1), well past any reasonable
   "disturbed" threshold.
2. **The 1.0m vision innovation-gate threshold could plausibly be crossed by PID-driven odometry
   drift alone, not just a real hit** — not yet proven either way. The Disturbance Report's medium
   (1.0m) trial triggered zero rejections and landed "right at" the gate threshold as measured
   against the estimate's own drift at that instant (Finding 3) — meaning the gate's behavior is
   already sensitive to exactly the kind of estimate drift the PID bug could independently produce
   in a worse case. This wasn't isolated as its own experiment; flagged here as an open risk a
   post-PID-fix re-test (already recommended for Finding 4) should also check.
3. **Finding 4's anomalous "recovery"** (severe run's error collapsing to near-zero, most likely a
   `setSimulationWorldPose()` velocity-reset artifact colliding with the chaotic PID behavior, not
   real disturbance tolerance) is exactly the kind of data point that would mis-tune a threshold if
   used naively — it looks like a success case but isn't one. Any threshold-tuning pass that
   includes this data without accounting for the confound risks learning "severe disturbances
   recover fine," which is not established.
4. **This codebase's own regression-test thresholds already run on a razor's edge.**
   `LtNeutralAutoRegressionTest`'s stall check has failed with margins as tight as 0.027m-0.049m
   against a 0.050m threshold, repeatedly, across multiple sessions documented in `CLAUDE.md`
   (independently A/B-confirmed as pre-existing flakiness, not a regression, each time). This is
   direct evidence that copying `AutoRegressionTolerances`' existing values wholesale into a live
   detector (as F4/F6 propose reusing) risks importing the same knife-edge flakiness into a system
   that, unlike a test suite, cannot be safely reran until it happens to pass.
5. **A human pressing a live SmartDashboard command button during auto** (Lifecycle Audit's
   catalogued but unfixed lower-severity finding) now correctly cancels the conflicting
   Superstructure command (post-bug-#2-fix) — but that cancellation is indistinguishable from a
   monitor-triggered one in `Auto/EndedInterrupted`, per §1's tertiary point. A monitor reading that
   flag today could not tell "a mentor clicked Eject" from "the auto actually failed."
6. **A previously-uninvestigated config failure mode (F12)** — `configureAutoBuilder()`'s
   try/catch around `AutoBuilder.configure()` degrades silently on failure while
   `RobotContainer.java`'s subsequent `AutoBuilder.buildAutoChooser()` call would crash robot boot
   outright if that happened. Low likelihood, but if it ever did happen, any health monitor
   depending on `AutoBuilder`-sourced state would be reasoning about a system that already failed
   to initialize, not one that's merely disturbed.

## 4. What prerequisites are required before `AutonomousHealthMonitor`?

In dependency order:

1. **Fix or characterize-and-bound the chassis-PID divergence bug.** Blocks every threshold-based
   signal. This is already a tracked, separate work item (`CLAUDE.md`'s "PID retune milestone"),
   currently hard-blocked on the same physical-robot access as the Autonomous Velocity migration's
   Phase 2 SysId characterization. Recovery-layer work and that migration track converge here —
   this assessment does not create a new blocker, it identifies that the existing one also blocks
   recovery.
2. **Re-run the Disturbance Report's exact experiment once (1) lands**, to get a trustworthy
   before/after baseline — the report's own explicit recommendation for resolving Finding 4, and
   the only way to know whether real disturbance recovery looks any different from the current
   chaotic-baseline artifact.
3. **Wire `TrajectoryErrorTracker`'s existing getters into a live consumer (F4)** — the algorithm
   itself doesn't need inventing (`WpilogStallAnalyzer`/`AutoRegressionTolerances` already prove it
   out in a post-hoc context) — but its threshold should not be finalized until (1)/(2) establish
   what "normal" actually looks like. Safe to build the plumbing now; unsafe to tune and trust the
   threshold before then.
4. **Make a deliberate design decision on vision-lockout backoff (F5)** — N consecutive rejections,
   what window — via a brainstorming/design pass, not a blind implementation, per the Recovery
   Audit's own recommendation. Independent of (1)-(3) in mechanism, but should be evaluated against
   the same false-trigger risk named in §3.2 before being finalized.
5. **Add recovery-action-specific telemetry** distinguishing a monitor-initiated cancellation from
   a teleop transition, a `testInit()` cancel, or a human dashboard click — not previously flagged
   by name in any of the three source audits, but a direct consequence of combining the Lifecycle
   Audit's `Auto/EndedInterrupted` work with the Recovery Audit's proposed Layer 4 supervisor. Needs
   to exist before Layer 4 can act, or its actions become unattributable in every future wpilog.
6. **A full design/plan cycle for Layer 4** (the `pathfindToPose`/`pathfindThenFollowPath`-based
   supervisor) — the Recovery Audit already recommends this explicitly, calling it "the only layer
   that changes what the robot does, not just what it reports." This assessment agrees it should
   not be improvised, and additionally notes: since the recovery path itself would still run
   through `PPHolonomicDriveController`, its real-world effectiveness is capped by whatever (1)
   leaves in place — worth factoring into that future design conversation, not resolved here.

**Not prerequisites — can start independently, in parallel with all of the above:** F1 (wiring
sim's existing `hasGamePiece` into `Superstructure.INTAKING`) and F3 (wiring `Shooter.isJammed()`
into `Superstructure.SHOOTING`, mirroring `Intake`'s already-shipped pattern). Neither depends on
drivetrain tracking behavior at all — they're isolated mechanism-success signals, and F3 in
particular has a fully proven template to copy. These are the lowest-risk, highest-confidence
starting points if the mentor wants to make *any* recovery-adjacent progress before the chassis-PID
fix lands.

---

## Specific evaluation

### Existing drivetrain/path tracking instability

**Classification: BLOCKER before recovery.**
Confirmed by two independent methods — decompiled-source analysis (`CLAUDE.md`'s sixteenth-session
findings: `PPHolonomicDriveController` sums feedforward+feedback with zero clamping) and live
simulation evidence (Disturbance Report Finding 1: 3.10m of error on an undisturbed control run).
This is the single item every other classification below is conditioned on. It is not a new
problem this assessment introduces — it's the same bug `CLAUDE.md` already tracks as blocked on
physical-robot SysId access — but it is now additionally understood to block recovery-signal
trustworthiness, not only raw tracking accuracy.

### `TrajectoryErrorTracker` reliability

**Classification: can proceed in parallel (plumbing) / BLOCKER (threshold-tuning and trust).**
The tracker's own computation is reliable — it runs live every cycle, is already relied upon by a
proven post-hoc analyzer (`WpilogTrajectoryErrorReader`), and nothing in any of the three source
documents challenges its arithmetic. What's missing is a live consumer (F4) and, more importantly,
a threshold that means something — which cannot be set with confidence while the baseline itself is
chaotic (§1). Building the wiring now is safe and low-risk (it's read-only, changes no behavior);
trusting its output for a real decision is not, yet.

### Vision rejection lockout

**Classification: can proceed in parallel (design), BLOCKER-adjacent (implementation).**
The innovation gate itself is not broken — it does exactly what it was built for (Recovery Audit
F5: rejecting single bad MegaTag2 frames). The gap is the missing backoff after a *sustained,
repeated* rejection streak, which the Disturbance Report empirically confirmed with real numbers
(242 consecutive rejections, zero backoff, Finding 3). The backoff design itself (N/window) doesn't
technically require the chassis-PID fix to design or even implement — but per §3.2, its trigger
threshold interacts with exactly the kind of pose drift the PID bug could independently cause, so
finalizing and tuning it before (1)/(2) risks the same false-trigger class as the tracking-error
threshold.

### PathPlanner recovery primitives

**Classification: can proceed in parallel (prototyping), future enhancement (production
auto-trigger wiring).**
`AutoBuilder.pathfindToPose()`/`pathfindThenFollowPath()` (F8) and `FollowPathCommand`'s
replan-on-reinit (F9) are real, already-shipped, already-deployed (a populated `navgrid.json`
exists) capabilities that nothing in this repo currently calls. Nothing about experimenting with
them or building a design for Layer 4 requires the chassis-PID fix first — they're orthogonal
library features. But since any path they generate would still be tracked by the same
`PPHolonomicDriveController`, wiring them into an *automatic, production* recovery trigger before
(1) is fixed risks recovering onto a path the robot then diverges from just as badly as the
original — the Recovery Audit's own framing of Layer 4 as "the only layer that changes robot
behavior" is exactly why this assessment treats its production activation as gated, even though
prototyping is not.

### Mechanism success detection

**Classification: can proceed in parallel (F1-sim, F3) / future enhancement (F1-hardware, F2).**
Intake success (F1, sim-only) and shooter jam recovery (F3) are both fully independent of
drivetrain/tracking concerns — pure subsystem-internal signals, one already populated in sim and
merely unread, the other with a proven, shippable template (`Intake.setRoller()`'s existing
jam-recovery pattern) ready to port. These are the safest, most immediately actionable items in the
entire recovery backlog and do not need to wait on anything above. Real-hardware intake detection
(F1) and shot-success detection (F2) are both hardware-gated LIMITATIONs, not code tasks — future
enhancements pending a sensor that doesn't exist on this robot today, not blocked by any software
prerequisite.

---

## Summary table

| Item | Classification |
|---|---|
| Chassis-PID divergence bug (unclamped feedback) | **Blocker** |
| Disturbance-experiment re-run post-PID-fix | **Blocker** (sequenced after the above) |
| `TrajectoryErrorTracker` live-consumer plumbing (F4) | Can proceed in parallel |
| `TrajectoryErrorTracker` threshold tuning/trust | **Blocker** |
| Vision lockout backoff — design/N-window decision (F5) | Can proceed in parallel |
| Vision lockout backoff — final tuning/trust | Blocker-adjacent (validate against PID fix) |
| Recovery-action-attribution telemetry | **Blocker** (new item, identified by this assessment) |
| `pathfindToPose`/`pathfindThenFollowPath` prototyping (F8) | Can proceed in parallel |
| `FollowPathCommand` replan-on-reinit adoption (F9) | Can proceed in parallel |
| Layer 4 supervisor — production auto-trigger wiring | Future enhancement, gated on PID fix |
| Intake success wiring, sim (F1) | Can proceed in parallel — **recommended first** |
| Shooter jam wiring (F3) | Can proceed in parallel — **recommended first** |
| Intake success detection, real hardware (F1) | Future enhancement (hardware-gated) |
| Shot success detection (F2) | Future enhancement (hardware-gated) |
| Config-load silent-degrade hardening (F12) | Future enhancement (low likelihood) |

---

## What this assessment did not do

- Did not modify any production code, test, or configuration file.
- Did not design or revise the Recovery Audit's proposed 4-layer framework — that design already
  exists and is not second-guessed here beyond noting where its own layers depend on the
  chassis-PID fix.
- Did not pick the vision-backoff N/window value or any tracking-error threshold — both remain
  judgment calls for a future design pass, same as the Recovery Audit already concluded.
- Did not re-run or re-verify any of the three source documents' own experiments; this is a
  synthesis pass over already-completed, already-cited evidence.
- Did not touch drivetrain velocity architecture (`DriveRequestType`/`Slot0`/SysId), the
  still-unmerged `feature/autonomous-completion-trigger-framework` worktree, or Controls Academy —
  all out of scope, consistent with every source document it draws from.
