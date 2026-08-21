# Autonomous Observability Phase 0.5 — Verification Audit

**Status update 2026-07-30:** This audit is a 2026-07-28 snapshot. Its verification of Phase 0.5
behavior-neutrality remains valid, but its repeated characterization of the old 3.10 m sim
divergence and physical-access dependency is superseded by the corrected pose-reset baseline.

**Date:** 2026-07-28
**Purpose:** Independently verify, after the fact and without re-implementing anything, that the
committed Phase 0.5 ("Autonomous Observability Layer") work matches its own stated scope in
`docs/Autonomous_Recovery_Readiness_Assessment.md`, actually behaves the way its commit message and
session notes claim, and is safe to leave as-is. This document performs **no code changes** — it is
a read-only audit, run via `/bootstrap` at the mentor's explicit request.

---

## 1. Git state

- Branch: `refactor/hybrid`
- `HEAD`: `d848ea5` ("Health checks in place")
- `git status --short`: **clean** — no dirty or untracked files.
- Phase 0.5 landed as two mentor-authored commits (not assistant-initiated), same pattern as the
  earlier `8cdcb9a` lifecycle-audit commit:
  - `44d369e` "Auto and work" — the command-lifecycle-continuation, Recovery Audit, and Disturbance
    Simulation work from earlier the same day (`docs/Autonomous_Recovery_Audit.md`,
    `docs/Autonomous_Disturbance_Simulation_Report.md`, `Robot.java`'s `Auto/Running`/
    `Auto/EndedInterrupted` telemetry, the throwaway `frc.robot.recovery` disturbance-test package,
    etc.). **Not** Phase 0.5 itself — it's the audit/evidence work Phase 0.5 was built on top of.
  - `d848ea5` "Health checks in place" — **this is Phase 0.5.** Confirmed by `git show --stat`
    against the file list `docs/claudex/sessions/2026-07-28.md`'s own "Update — Phase 0.5" section
    names as touched: `Robot.java`, `RobotContainer.java`, `Vision.java`,
    `TrajectoryErrorTracker.java`, `RobotAutoTerminationTelemetryTest.java`, plus new
    `docs/Autonomous_Recovery_Readiness_Assessment.md`, `AutonomousHealthMonitor.java`,
    `AutonomousHealthMonitorTest.java`, `TrajectoryErrorTrackerTest.java`. Exact match — nothing
    extra, nothing missing.

## 2. Files changed (Phase 0.5 / commit `d848ea5`)

| File | Type | Change |
|---|---|---|
| `src/main/java/frc/robot/auto/AutonomousHealthMonitor.java` | New (production) | Supplier-injected health aggregator; read-only. |
| `src/main/java/frc/robot/Robot.java` | Modified (production) | New `AutoEndReason` enum + `pendingCancelReason` static field; tagged at `teleopInit()`/`testInit()`; consumed in the existing `wrapAutonomousForTelemetry()` `finallyDo`. |
| `src/main/java/frc/robot/RobotContainer.java` | Modified (production) | Constructs `autonomousHealthMonitor`, wires `trajectoryErrorTracker::getSecondsSinceLastFreshTargetPose` / `vision::getLastRejectedJumpMeters` into it, calls `.periodic()` from `trajectoryTrackerPeriodic()`. |
| `src/main/java/frc/robot/subsystems/vision/Vision.java` | Modified (production) | New `lastRejectedJumpMeters` field + `getLastRejectedJumpMeters()` getter, set alongside the existing `Vision/RejectedJumpMeters` log call. |
| `src/main/java/frc/robot/utility/TrajectoryErrorTracker.java` | Modified (production) | New `timestampSecondsSupplier` constructor param (call site updated in `RobotContainer.java`); new `getSecondsSinceLastFreshTargetPose()` getter. |
| `src/test/java/frc/robot/auto/AutonomousHealthMonitorTest.java` | New (test) | 14 tests. |
| `src/test/java/frc/robot/utility/TrajectoryErrorTrackerTest.java` | New (test) | 4 tests. |
| `src/test/java/frc/robot/RobotAutoTerminationTelemetryTest.java` | Modified (test) | Extended with a third phase asserting `Auto/EndReason` attribution. |
| `docs/Autonomous_Recovery_Readiness_Assessment.md` | New (doc) | The synthesis document Phase 0.5 was scoped against. |
| `claude.md` | Modified (doc) | Session-handoff pointer update only — no behavioral content. |
| `docs/claudex/history.md`, `docs/claudex/sessions/2026-07-28.md` | Modified (doc) | Session bookkeeping. |

No file outside this list changed in `d848ea5`. Confirmed via `git show --stat d848ea5` directly,
not from session-note prose.

## 3. Telemetry added

New AdvantageKit outputs, all logged only from `periodic()`/logging call sites (never from `update()`,
which does no I/O):

- `Auto/Health/TrackingDegraded` (bool) — `|lateral error| > 1.0m` OR `|longitudinal error| > 1.0m`.
- `Auto/Health/Stalled` (bool) — rolling 1.0s window, `<0.05m` moved, gated on
  `secondsSinceLastFreshTargetPose() <= 0.5s` (never fires during an intentional stationary phase).
- `Auto/Health/VisionUnhealthy` (bool) — `≥5` consecutive cycles with a nonzero rejected-jump value.
- `Auto/Health/RecoveryReason` (string) — one of `NONE`/`TRACKING_DEGRADED`/`STALLED`/
  `VISION_UNHEALTHY`, priority-ordered in that listed order.
- `Auto/EndReason` (string) — one of `NATURAL_COMPLETION`/`TELEOP_INTERRUPTION`/`TEST_CANCELLATION`/
  `RECOVERY_CANCELLATION` (reserved, unused — no code path sets it yet)/`UNKNOWN_INTERRUPTION`.

All four `Auto/Health/*` thresholds (`kTrackingDegradedLateralMeters`/`kTrackingDegradedLongitudinalMeters`
= 1.0m, `kStallWindowSeconds` = 1.0s, `kStallTranslationThresholdMeters` = 0.05m,
`kStalePathSetpointGraceSeconds` = 0.5s, `kVisionUnhealthyConsecutiveRejections` = 5) are declared
`static final` in `AutonomousHealthMonitor.java` and documented in its own class javadoc as
placeholders, not tuned values — matching the Readiness Assessment's own conclusion (§4.3) that
tuning must wait on the chassis-PID fix.

## 4. Production behavior impact assessment

**Assessed impact: zero.** Verified three independent ways, not just asserted:

1. **Structural:** `AutonomousHealthMonitor.java` contains no reference to `CommandScheduler`, no
   `.cancel()`, `.schedule()`, or any WPILib Command-lifecycle API anywhere in the file (confirmed by
   direct read of the full source — grep-equivalent manual check, 177 lines). Every method is either
   a pure computation (`update()` and its four `updateX()` helpers) or a `Logger.recordOutput` call
   (`log()`). There is no code path from this class into anything that could alter what the robot
   does.
2. **Test-enforced:** `AutonomousHealthMonitorTest.updateNeverSchedulesOrCancelsCommands()` schedules
   a real command, calls `update()` 20 times with deliberately adverse (degraded/stalled/unhealthy)
   synthetic inputs, and asserts the command is still scheduled afterward. This test passed in this
   session's own run (see §5).
3. **Call-site review:** `RobotContainer.trajectoryTrackerPeriodic()`'s only new line is
   `autonomousHealthMonitor.periodic()`, appended after the pre-existing
   `trajectoryErrorTracker.periodic()` call, itself invoked from `Robot.java`'s existing periodic
   loop (unchanged call site, not audited as new). Nothing consumes `AutonomousHealthMonitor`'s
   query methods (`isTrackingDegraded()`, `isStalled()`, `isVisionUnhealthy()`, `getRecoveryReason()`)
   anywhere in `src/main` — confirmed by their absence from any file's diff or any grep hit outside
   the class's own definition and its test.

**`Robot.java`'s `AutoEndReason` change is additive-only, not decision-altering.** `pendingCancelReason`
is written at `teleopInit()`/`testInit()` (both pre-existing call sites, unchanged apart from the one
new assignment line each) and read once, inside the `finallyDo` callback `wrapAutonomousForTelemetry()`
already installs — that callback already ran and already logged `Auto/EndedInterrupted` before this
change; the only new behavior is an additional log line (`Auto/EndReason`) computed from state that
was already available. No command is scheduled, cancelled, or altered by this code.

**`Vision.java` and `TrajectoryErrorTracker.java` changes are pure getters over already-computed
values** — `lastRejectedJumpMeters` mirrors a value already being logged every cycle;
`getSecondsSinceLastFreshTargetPose()` derives from timestamps already being tracked
(`hasFreshTargetPose`/an existing `logAndClearFreshness()` path). Neither changes
`fuseMeasurements()`'s accept/reject decision or the trajectory-error computation itself — confirmed
by reading the diff hunks directly (§2 above; full diff reviewed, not summarized from memory).

**Conclusion: this phase makes zero changes to autonomous robot behavior.** Every new code path is
either read-only telemetry or a getter over an existing value. This matches the mentor's explicit
Phase 0.5 instruction ("zero behavior change") and the Readiness Assessment's own framing of
observability as safe to build ahead of the chassis-PID fix.

## 5. Regression evidence (run fresh this session, not reused from prior session notes)

All gates re-run from the current `d848ea5` HEAD, `JAVA_HOME` pointed at the WPILib JDK 17:

- **Gate 1 (`./gradlew compileJava`):** BUILD SUCCESSFUL.
- **Gate 2 (`python SKILLS/run_headless_sim.py --run-seconds 12`):** PASS — 12s of disabled-mode
  periodic loops, no exceptions.
- **Targeted Phase 0.5 tests** (`AutonomousHealthMonitorTest`, `TrajectoryErrorTrackerTest`,
  `RobotAutoTerminationTelemetryTest`): **19/19 passed** (14 + 4 + 1), confirmed via the generated
  JUnit XML reports' `tests=`/`failures=` attributes, not console text alone.
- **Full suite (`./gradlew test`):** **40/41 passed.** The one failure —
  `LtNeutralAutoRegressionTest.regressionCheck()`, `"LT Neutral stalled at t=5.16s: moved 0.045m over
  the preceding 0.98s (threshold 0.050m)... expected: <false> but was: <true>"` — matches, message
  for message and margin for margin, the same pre-existing razor-thin-margin flaky stall check
  documented repeatedly across this project's session history (0.025m–0.049m against the same
  0.050m threshold, independently A/B-confirmed as pre-existing on multiple prior occasions, most
  recently in this same day's own Phase 0.5 implementation session via `git stash`/rerun). No new
  failure signature appeared. This is not re-litigated as a regression from Phase 0.5.

No `IntakeJamRecoveryTest` hang occurred in this session's full-suite run (it passed cleanly) — the
previously-documented `SimHooks.stepTiming()` JNI race is intermittent, not reproduced this run, and
is unrelated to Phase 0.5's files regardless.

## 6. Scope conformance vs. `docs/Autonomous_Recovery_Readiness_Assessment.md`

Checked against the assessment's own §4 ("prerequisites... in dependency order") and its
"can proceed in parallel" carve-outs:

| Assessment item | Phase 0.5 status | Conforms? |
|---|---|---|
| §4.3 "Wire `TrajectoryErrorTracker`'s existing getters into a live consumer (F4)... safe to build the plumbing now; unsafe to tune and trust the threshold before then." | Done exactly as scoped: `AutonomousHealthMonitor` consumes `getLateralErrorMeters()`/`getLongitudinalErrorMeters()`/`getSecondsSinceLastFreshTargetPose()`; thresholds left as explicit, documented placeholders. | **Yes.** |
| §4.5 "Add recovery-action-specific telemetry distinguishing a monitor-initiated cancellation from a teleop transition, a `testInit()` cancel, or a human dashboard click." | `AutoEndReason` enum added with a `RECOVERY_CANCELLATION` case reserved but **unused** (no code sets it — correct, since no recovery layer exists yet to trigger it). `TELEOP_INTERRUPTION`/`TEST_CANCELLATION` are wired and tested. | **Yes** — exactly the "needs to exist before Layer 4 can act" plumbing, not Layer 4 itself. |
| Vision-rejection aggregation (§2 table, "would have to build that aggregation itself") | `consecutiveVisionRejections` counter + `kVisionUnhealthyConsecutiveRejections` threshold added inside `AutonomousHealthMonitor`, not inside `Vision.java` itself — `Vision.java` only exposes the raw per-cycle value via `getLastRejectedJumpMeters()`. | **Yes**, and arguably cleaner than the assessment's phrasing implied (keeps `Vision.java` free of aggregation logic). |
| "Not prerequisites — can start independently" (F1 sim `hasGamePiece`, F3 `Shooter.isJammed()`) | **Not touched.** Neither `Superstructure.INTAKING` nor `Superstructure.SHOOTING` reference these signals; `Intake.java`/`Shooter.java` are absent from `d848ea5`'s diff entirely. | **Correctly out of scope** — the mentor's Phase 0.5 request didn't ask for these, and this audit confirms they weren't opportunistically bundled in. |
| "Layer 4 supervisor... should not be improvised" / production auto-trigger wiring | Not touched — no `pathfindToPose`/`pathfindThenFollowPath` call added anywhere; `AutonomousHealthMonitor`'s outputs have zero consumers. | **Correctly out of scope.** |
| "Fix or characterize-and-bound the chassis-PID divergence bug" (§4.1, the historical primary blocker) | **Not touched**, and correctly so — Phase 0.5 was never scoped to fix it. The later pose-reset correction invalidated the old divergence evidence; threshold trust still requires a fresh evidence pass. | **Correctly out of scope**, not a gap in this phase. |

**No deviations found.** Every file Phase 0.5 touched maps to an item the Readiness Assessment
explicitly marked "can proceed in parallel," and nothing marked "blocker" was touched.

## 7. Known limitations (carried forward, not resolved by this audit)

- **The four `Auto/Health/*` thresholds are placeholders, not tuned values.** They should not be
  trusted to distinguish "disturbed" from "normal" until the corrected-baseline evidence pass and
  mentor review establish appropriate thresholds.
- **Nothing consumes `Auto/Health/*` or `Auto/EndReason` yet.** No alerting, no dashboard widget, no
  decision logic. This was correctly out of scope for Phase 0.5 and remains unimplemented.
- **`RECOVERY_CANCELLATION` is a reserved, currently-dead enum value** — no code path sets it, by
  design, since no recovery layer exists yet to trigger it. Not a bug; a placeholder for the future
  Layer 4 work the assessment describes.
- **`LtNeutralAutoRegressionTest`'s stall-check flakiness is unrelated to this phase** but remains
  unresolved as its own item — this audit re-confirms (§5) it is not new or worsened, not that it is
  fixed.
- **The vision-lockout backoff design (F5's N/consecutive-rejections/window question) was not
  revisited** — `kVisionUnhealthyConsecutiveRejections = 5` is this phase's own placeholder pick, not
  a value derived from a dedicated design pass the Readiness Assessment (§4.4) says should happen
  separately.

## 8. Verdict: Ready / Not Ready for Phase 1

Two different "Phase 1" concepts exist in this repo's history and this audit is explicit about which
one it's answering:

- The Autonomous **Velocity migration** track's own "Phase 1" (sim-only SysId workflow validation) is
  already complete and unrelated to this thread — not what this section evaluates.
- The question this audit answers is: **is it safe to move past observability into the next phase of
  the recovery-layer work** — i.e., start having `AutonomousHealthMonitor`'s signals actually
  influence robot behavior (cancel a command, trigger a `pathfindToPose` recovery, tighten/loosen a
  vision gate)?

**Verdict: NOT READY.**

This is not a new conclusion — it is a re-confirmation of the Readiness Assessment's own verdict,
checked against current evidence rather than assumed still true:

1. The assessment's original primary blocker was the pre-pose-fix sim divergence; that evidence is
   superseded. Phase 0.5 still does not authorize behavior changes, and its placeholder thresholds
   require a fresh evidence pass before they can drive a decision.
2. Phase 0.5 itself is complete, correctly scoped, and verified clean in this session (§4-§6) — there
   is nothing further to do *within* Phase 0.5. But completing the plumbing does not unblock what the
   plumbing was explicitly not allowed to do yet (drive a decision).
3. No new evidence gathered this session changes the Readiness Assessment's threshold-trustworthiness
   analysis — the same four placeholder thresholds are still placeholders, and the same "would fire on
   every normal run" risk (assessment §3.1) still applies unchanged.

**Do not, without explicit further instruction:**
- Wire any `AutonomousHealthMonitor` query into command cancellation, `pathfindToPose`, or any other
  behavior-altering logic.
- Treat the four `Auto/Health/*` thresholds as tuned.
- Set `AutoEndReason.RECOVERY_CANCELLATION` from any new code path (that would imply a recovery layer
  exists, which it does not).

**Safe to do in parallel, per the Readiness Assessment's own carve-out (§4, "not prerequisites"),** if
the mentor wants further recovery-adjacent progress before the chassis-PID fix lands: F1 (wire sim's
`hasGamePiece` into `Superstructure.INTAKING`) and F3 (wire `Shooter.isJammed()` into
`Superstructure.SHOOTING`) — both isolated mechanism-success signals, independent of drivetrain
tracking behavior, neither touched by this phase.
