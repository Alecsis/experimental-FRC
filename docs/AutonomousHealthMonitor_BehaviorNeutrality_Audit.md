# AutonomousHealthMonitor — Behavior-Neutrality Re-Verification Audit

**Date:** 2026-07-28
**Purpose:** Re-verify, independently and from current source (not by trusting the prior audit's
conclusions), that `AutonomousHealthMonitor` and everything wired to it are still purely
observability — no command scheduling/cancellation, no altered accept/reject or control-loop
decisions anywhere downstream. Read-only audit; no code, test, Gradle, or config file was modified.

A prior audit already covers this exact question at commit `d848ea5`:
`docs/Autonomous_Observability_Phase0_5_Audit.md`. This document does not restate its content —
it independently re-derives the same conclusion from the current `HEAD`, and reports whether
anything has drifted since.

---

## 1. Git state

- Branch: `AI-Agents` (identical history to `refactor/hybrid`).
- `HEAD`: `b8f6fbc` ("Logs") — one commit ahead of `d848ea5`, the commit the prior audit verified.
- `git status --short`: clean, no dirty or untracked files.
- **`git diff --stat d848ea5 HEAD -- src/` returns empty** — zero bytes of `src/` have changed
  since the prior audit's verified-clean snapshot. The intervening commit (`b8f6fbc`) touched only
  `.claude/commands/*.md`, `SKILLS/*-agent.md`, `claude.md`, and two `docs/claudex/` bookkeeping
  files (confirmed via `git show --stat b8f6fbc`) — none of which are `AutonomousHealthMonitor` or
  any file it's wired through.

This means the source-level facts below are, byte-for-byte, the same code the prior audit read —
but they were re-derived independently in this session rather than copied from that document, per
this project's evidence standard.

## 2. `AutonomousHealthMonitor.java` itself

Full file read (`src/main/java/frc/robot/auto/AutonomousHealthMonitor.java`, 177 lines):

- No import, reference, or call to `CommandScheduler`, `Command.schedule()`, `Command.cancel()`, or
  any WPILib Command-lifecycle API anywhere in the file.
- `update()` (lines 82-87) calls only its four private `updateX()` helpers, each of which reads a
  supplier and writes a private field — no I/O, no side effects outside `this`.
- `periodic()` (lines 90-93) is `update()` + `log()`; `log()` (lines 154-159) is four
  `Logger.recordOutput` calls and nothing else.
- The four public query getters (`isTrackingDegraded()`, `isStalled()`, `isVisionUnhealthy()`,
  `getRecoveryReason()`, lines 161-175) are plain field returns.

## 3. Wiring — `RobotContainer.java`

- Construction (`RobotContainer.java:87-93`): `autonomousHealthMonitor` is built from six
  suppliers, all read-only (`trajectoryErrorTracker::getLateralErrorMeters`,
  `::getLongitudinalErrorMeters`, `::getSecondsSinceLastFreshTargetPose`,
  `() -> drivetrain.getState().Pose`, `Timer::getFPGATimestamp`, `vision::getLastRejectedJumpMeters`).
- Only call site: `trajectoryTrackerPeriodic()` (`RobotContainer.java:164-167`) calls
  `autonomousHealthMonitor.periodic()` immediately after `trajectoryErrorTracker.periodic()`. This
  method is invoked from `Robot.robotPeriodic()` **after** `CommandScheduler.getInstance().run()`
  (`Robot.java:119-123`) — confirmed by reading the call order directly, not inferred from a
  comment.
- **Consumer search:** `grep -rn "isTrackingDegraded|isStalled|isVisionUnhealthy|getRecoveryReason|autonomousHealthMonitor" src/` returns hits only inside `AutonomousHealthMonitor.java` itself,
  its wiring in `RobotContainer.java` (construction + the one `.periodic()` call), and
  `AutonomousHealthMonitorTest.java`. Nothing in `src/main` reads any of the four query getters —
  there is no decision logic anywhere downstream of this class's output.

## 4. `Robot.java` — `AutoEndReason` attribution

- `pendingCancelReason` is set at `teleopInit()` (`Robot.java:192`) and `testInit()`
  (`Robot.java:211`), and consumed once inside `wrapAutonomousForTelemetry()`'s `finallyDo`
  callback (`Robot.java:174-184`) — a callback that was already installed and already logging
  `Auto/Running`/`Auto/EndedInterrupted` before Phase 0.5; the only new behavior is one additional
  `Logger.recordOutput("Auto/EndReason", ...)` line computed from state that was already available.
- `AutoEndReason.RECOVERY_CANCELLATION` is declared but **still unused** — `grep -n "RECOVERY_CANCELLATION" src/main/java/frc/robot/Robot.java` shows it only in the enum declaration and the
  ternary's implicit fallthrough (`interrupted ? pendingCancelReason : NATURAL_COMPLETION`), never
  assigned by any code path. No recovery layer exists yet to set it — this is intentional, matching
  the prior audit's finding, and unchanged.
- No command is scheduled, cancelled, or altered by any of this — `pendingCancelReason` is pure
  bookkeeping for a `Logger.recordOutput` call.

## 5. `Vision.java` — `getLastRejectedJumpMeters()`

- `lastRejectedJumpMeters` (`Vision.java:74`) is assigned once, at `fuseMeasurements():244`, from
  `rejectedJumpMeters` — a local variable already fully computed by the existing accept/reject loop
  above it (`Vision.java:202-243`).
- Read the full body of `fuseMeasurements()` directly: the innovation-gate rejection (`jumpMeters >
  Constants.kMaxVisionJumpMeters`, line 228) and the `drivetrain.addVisionMeasurement(...)` accept
  path (line 239-240) are unchanged — `lastRejectedJumpMeters`'s assignment is the only new line in
  the method, and it does not participate in the branch condition anywhere.
- `getLastRejectedJumpMeters()` (`Vision.java:252-254`) is a plain field return.

## 6. `TrajectoryErrorTracker.java` — `getSecondsSinceLastFreshTargetPose()`

- `lastFreshTargetPoseTimestampSeconds` is written only inside `logAndClearFreshness()`
  (`TrajectoryErrorTracker.java:116-125`), gated on the pre-existing `hasFreshTargetPose` flag.
- The error-computation logic in `periodic()` (lines 78-114: heading/longitudinal/lateral error,
  path-tangent tracking) has no new branches and does not read
  `lastFreshTargetPoseTimestampSeconds` anywhere — confirmed by reading the full method body.
- `getSecondsSinceLastFreshTargetPose()` (lines 145-150) is a pure derived getter (`now - last`, or
  `+Infinity` if never set).

## 7. Regression evidence (run fresh this session)

`JAVA_HOME` pointed at the WPILib JDK 17 (`/c/Users/Public/wpilib/2026/jdk`), all from current
`HEAD` (`b8f6fbc`):

- **Gate 1 (`./gradlew compileJava`):** BUILD SUCCESSFUL.
- **Gate 2 (`python SKILLS/run_headless_sim.py --run-seconds 12`):** PASS — 12s disabled-mode
  periodic loops, no exceptions.
- **Targeted tests** (`AutonomousHealthMonitorTest`, `TrajectoryErrorTrackerTest`,
  `RobotAutoTerminationTelemetryTest`): **19/19 passed** (14 + 4 + 1), confirmed via console
  PASSED lines for every individual test method. `AutonomousHealthMonitorTest.
  updateNeverSchedulesOrCancelsCommands()` — the test that schedules a real command, hammers
  `update()` 20× with deliberately adverse synthetic inputs, and asserts the command is still
  scheduled afterward — passed.
- Full `./gradlew test` was **not** re-run this session (out of scope for this narrow question,
  and this project's own history documents a real, previously-root-caused
  `SimHooks.stepTiming()`/`IntakeJamRecoveryTest` JNI-hang class that makes full-suite runs
  unpredictably slow) — the targeted set above is the complete set of tests that exercise
  `AutonomousHealthMonitor` or its collaborators, so it fully answers this audit's question without
  needing the full suite.

## 8. Verdict

**Still behavior-neutral. No drift since the prior Phase 0.5 audit (`d848ea5`).**

- Zero `src/` changes since the commit the prior audit verified (§1).
- Independent re-read of `AutonomousHealthMonitor.java` and all four files it touches confirms: no
  `CommandScheduler` interaction, no consumer of its query getters anywhere in production code, and
  every collaborator change (`Vision.java`, `TrajectoryErrorTracker.java`, `Robot.java`'s
  `AutoEndReason`) is a pure additive getter/telemetry line over a value the pre-existing logic
  already computed — none of them participate in any accept/reject, control, or scheduling
  decision.
- All applicable gates pass fresh from current `HEAD`.

## 9. Known limitations (carried forward, not this audit's concern to resolve)

Unchanged from `docs/Autonomous_Observability_Phase0_5_Audit.md` §7 — repeated here only so this
document is self-contained:

- The four `Auto/Health/*` thresholds are still placeholders, not tuned.
- Nothing consumes `Auto/Health/*` or `Auto/EndReason` yet — no alerting, no dashboard, no decision
  logic. That remains correctly out of scope.
- `RECOVERY_CANCELLATION` remains a reserved, currently-dead enum value by design.
- The chassis-PID divergence bug (the prerequisite for trusting any of these signals) remains
  unfixed and hard-blocked on physical-robot access, per `CLAUDE.md`.

**No fix or change is recommended by this audit** — its only question ("is this still
behavior-neutral") is answered yes, with fresh evidence.
