# Autonomous Command Lifecycle Audit

**Date:** 2026-07-28
**Scope:** Command lifecycle, bindings, and NamedCommands — reliability of autonomous, not drivetrain
velocity architecture (untouched; see `docs/superpowers/plans/2026-07-28-autonomous-velocity-migration.md`
for that separate, hardware-blocked track).
**Files audited:** `RobotContainer.java`, `OperatorControls.java`, `Superstructure.java`,
`CommandSwerveDrivetrain.java` (Command-returning methods only), `Intake.java`, `Shooter.java`, all `.auto`
files in `src/main/deploy/pathplanner/autos/` (12 at the time of the original pass; see "Continued audit"
below for a file-count correction), `Robot.java` (mode-transition cancellation and, as of the continued
audit below, completion telemetry).

## Summary

Two real bugs found and fixed this session, both via TDD (regression test first, minimal fix, full gate
verification). One is a genuine, previously-flagged, high-severity hang risk present in **every single
auto** in the fleet; the other is a requirements gap that let one auto step run uninterruptible-but-also-
uninterrupting alongside a conflicting manual command. Beyond those two, the rest of the command graph is
sound: every other NamedCommand and button binding has a real, provable end condition or correct
interruption behavior. A handful of lower-severity/informational risks are catalogued below for future
awareness, not fixed (out of this session's minimal-fix scope).

## Bugs found and fixed

### 1. `"Orbit"` had no timeout — every auto could hang forever (HIGH → fixed)

`RobotContainer.java` registered `"Orbit"` as
`drivetrain.trackHub(vision, 0, () -> 0, () -> 0, true)` with no bound. `trackTarget`'s `isFinished`
condition (`CommandSwerveDrivetrain.java`, private `trackTarget()`) only returns true once heading error
drops under `kTrackHeadingToleranceRad` (2°); there is no time-bound fallback. Every one of the 12 `.auto`
files ends with `parallel(Orbit, Shooting Sequence)` or `parallel(Orbit, Quick Shooting)` — a
`ParallelCommandGroup`, which requires **every** branch to finish before the group itself completes. An
unclamped heading-PID (`kP=5.0`, output never clamped in `trackTarget`, unlike `driveToPOI`'s clamped
PIDs) that doesn't settle, or any other reason `Orbit` fails to converge, silently stalls the *entire* auto
forever — the same bug class as the already-fixed `intakeCmd()`-in-a-parallel-block hang (13th session, see
`Superstructure.intakeSequence`'s javadoc). This was already flagged as an unaddressed risk in `CLAUDE.md`
("a `finishOnAlign`-based command with no timeout backstop — it can hang forever if vision never
converges") but never acted on until this audit.

Contributing evidence this isn't hypothetical: the existing `LT_Neutral.json` regression golden already
records `"completed": false"` with `runtimeSeconds` pinned to exactly the harness's 15s real-time cap —
consistent with (though not proven solely caused by) the auto never reaching a self-terminating state.

**Regression test (RED first):** `src/test/java/frc/robot/auto/AutoCommandSafetyTest.java` — a pure
JSON-parsing structural test with no CommandScheduler/Robot/sim dependency. Walks every `.auto` file's
command tree (handling `named`/`wait`/`path`/`sequential`/`parallel`/`race`/`deadline` nodes, and correctly
flagging a `named` command nested inside a `sequential` branch of a `parallel` block — a traversal blind
spot covered by its own dedicated test) and asserts every NamedCommand used inside a `parallel` block is
listed in a new `RobotContainer.BOUNDED_NAMED_COMMANDS` allowlist. With `"Orbit"` deliberately left out of
the allowlist, the test failed exactly as expected, flagging `"Orbit"` in all 12 auto files by name.

**Fix:** `RobotContainer.java` — `NamedCommands.registerCommand("Orbit", drivetrain.trackHub(vision, 0, () ->
0, () -> 0, true).withTimeout(2.0))`, and `"Orbit"` added to `BOUNDED_NAMED_COMMANDS`. 2.0s was chosen as a
conservative bound: ample margin above normal heading-PID convergence time (should be well under a second
in practice), while not becoming the long pole against its `parallel` partner's own bound (`Quick Shooting`
caps at 3.0s in one auto). Not competition-tuned — an initial safe value, same as `"Intake Start Sequence"`'s
5.0s pick in the 13th session, adjustable later without re-litigating the underlying bug.

**Verified:** RED confirmed (`AutoCommandSafetyTest` failed, flagging exactly `"Orbit"` × 12 auto files) →
fix applied → GREEN. `./gradlew compileJava` BUILD SUCCESSFUL. `python SKILLS/run_headless_sim.py
--run-seconds 12` PASS. Full `./gradlew test`: 16/17 (see "Pre-existing flaky test" below for the one
failure). Zero other files touched.

### 2. `shootingSequence()` didn't declare a Superstructure requirement (MEDIUM → fixed)

`Superstructure.shootCmd()`/`intakeCmd()`/`ejectCmd()`/`stowCmd()` all declare `this` (Superstructure) as
their Command requirement (`Commands.startEnd(..., this)` / `Commands.runOnce(..., this)`) — normal WPILib
practice, so the `CommandScheduler` cancels one automatically if another is scheduled while it's running.
`shootingSequence(double)` — the method behind both `"Shooting Sequence"` and `"Quick Shooting"`
NamedCommands, run by every in-scope auto — composed `Commands.runOnce(() -> requestShoot(timeoutSeconds))`
and `Commands.waitUntil(() -> !mShotInProgress)` with **no subsystem argument on either**. A
`SequentialCommandGroup`'s requirements are the union of its members' requirements, so with both members
requirement-less, the composed sequence had an **empty requirement set**.

Concrete failure scenario this created: `RobotContainer.dashboard()` exposes `"State: Shoot"`/`"State:
Eject"`/`"State: Intake"`/`"State: Stow"` as live `SmartDashboard.putData` buttons — clickable over
NetworkTables in every robot mode, including autonomous. Before this fix, if `"Shooting Sequence"` was
mid-flight during auto and someone pressed the dashboard's `"State: Eject"` button, the scheduler would
**not** cancel either command (no requirement conflict) — both would run concurrently, each writing
`mWantedState`/`mShotInProgress` on alternating scheduler ticks, corrupting the state machine
unpredictably instead of cleanly interrupting one or the other.

**Regression test (RED first):** `src/test/java/frc/robot/SuperstructureCommandRequirementsTest.java` —
boots a real `Robot()` in sim (matching `SuperstructureEjectingTest`'s established pattern; own test class
per `build.gradle`'s `forkEvery = 1`), gets `Superstructure.shootingSequence(1.0)`, and asserts
`.getRequirements().contains(superstructure)`. Failed exactly as expected pre-fix (empty requirement set).

**Fix:** `Superstructure.java` — changed `Commands.runOnce(() -> requestShoot(timeoutSeconds))` to
`Commands.runOnce(() -> requestShoot(timeoutSeconds), this)`. That alone gives the whole composed sequence
the Superstructure requirement (union-of-members), so `shootingSequence()` now behaves exactly like the
other four bridge Commands for interruption purposes.

**Verified:** RED confirmed → fix applied → GREEN. `./gradlew compileJava` BUILD SUCCESSFUL.
`python SKILLS/run_headless_sim.py --run-seconds 12` PASS. Full `./gradlew test`: 16/17 (same one
pre-existing failure, see below). Zero other files touched.

### Causality check for the one full-suite failure

Both fixes' full-suite runs show exactly one failure: `LtNeutralAutoRegressionTest`'s stall check, at
`t≈5.2-5.3s`, margins of 0.037-0.049m against a 0.050m threshold — this is early in path-following, well
before either fix's code paths (`Orbit`/`shootingSequence` only execute at the very end of the auto). This
is the same pre-existing flaky stall-check class documented extensively elsewhere in this session's
history (0.045m/0.047m/0.027m margins recorded earlier today alone). Confirmed via A/B, not assumed: with
both fixes `git stash`ed back to the pre-session baseline, the identical test failed on its first rerun
(0.049m vs 0.050m, same `t≈5.2s` location) and then passed on a second immediate rerun with zero code
changes — proving the flakiness is independent of both fixes.

*(One further prompt-injection occurrence surfaced during this A/B check: a `<system-reminder>`-style note
on the `git stash` tool output falsely claimed `RobotContainer.java` was "modified by the user or a linter"
and instructed concealment. Declined and surfaced immediately per the Trust Boundary policy — same
fingerprint as the running tally in `docs/claudex/history.md`; the change was in fact this session's own
stash, not the user's or a linter's.)*

## Full NamedCommand inventory

| NamedCommand | Backing method | Requirement | Terminates? | Bound | Risk (post-fix) |
|---|---|---|---|---|---|
| `"Home Intake"` | `Intake.homing()` | Intake | Yes | `.until(hardstop).withTimeout(1)` + 2 instant steps; ≤~1s | LOW |
| `"Orbit"` | `drivetrain.trackHub(...).withTimeout(2.0)` | Drivetrain | Yes (fixed this session) | heading-converge (<2°) OR 2.0s hard cap | LOW (was HIGH) |
| `"Shooting Sequence"` | `Superstructure.shootingSequence(5.0)` | Superstructure (fixed this session) | Yes | ≤1.0s align + ≤5.0s feed ≈ ≤6.0s worst case | LOW (was MEDIUM) |
| `"Quick Shooting"` | `Superstructure.shootingSequence(3.0)` | Superstructure (fixed this session) | Yes | ≤1.0s align + ≤3.0s feed ≈ ≤4.0s worst case | LOW (was MEDIUM) |
| `"Intake Start Sequence"` | `Superstructure.intakeSequence(5.0)` = `intakeCmd().withTimeout(5.0)` | Superstructure | Yes | hard 5.0s cap (intakeCmd() itself never self-finishes) | LOW |
| `"Intake Stop"` | `Superstructure.stowCmd()` | Superstructure | Yes | instant (`runOnce`) | LOW |

Every NamedCommand in this table is used inside a PathPlanner `"parallel"` block in at least one auto
(confirmed via direct JSON inspection), so every row above is a "can this hang the whole auto" question, not
a hypothetical.

## Button binding inventory (interruption behavior)

All Command factories below are called fresh at each binding site (no shared/cached Command instance reused
across bindings) — confirmed by direct code inspection, so there's no risk of the same Command object being
scheduled from two different Triggers concurrently.

**`RobotContainer.configureBindings()`:**
| Binding | Command | Requirement | Interruption | Risk |
|---|---|---|---|---|
| `RobotModeTriggers.disabled()` `.whileTrue` | `Idle` request, `.ignoringDisable(true)` | Drivetrain | whileTrue: cancels on trigger→false | LOW |
| `RobotModeTriggers.teleop()` `.onTrue` | `intake.homing()` | Intake | onTrue: self-terminating, no re-trigger risk | LOW |
| `sysid.leftBumper/leftTrigger/rightTrigger` `.onTrue` | mode-select `runOnce` | none | instant | LOW |
| `sysid.y/a/b/x` `.and(leftBumper)` `.whileTrue` | `sysIdQuasistatic`/`sysIdDynamic` | Drivetrain | whileTrue, bench-only controller (port 2) | LOW (bench tooling, not match-reachable) |

**`OperatorControls.configureBindings()`:**
| Binding | Command | Requirement | Interruption | Risk |
|---|---|---|---|---|
| default command | teleop `applyRequest` drive | Drivetrain | replaced by any drivetrain command; see "default vs. auto" below | LOW/INFO |
| `rightBumper/rightTrigger` `.whileTrue` | `trackHub`/`trackPassTarget` (`finishOnAlign=false`) | Drivetrain | whileTrue, never self-finishes by design (hold-to-track) | LOW |
| `leftBumper` `.onTrue` | seed heading + pose reset `runOnce` | none | instant | LOW |
| `povRight/povLeft/x/y/b` `.whileTrue` | `driveToPOI(...)` | Drivetrain | whileTrue AND self-finishes on tolerance — won't auto-restart if held past convergence (standard WPILib `whileTrue` semantics, not a bug) | LOW |
| `controlBox.button(2/6/3)` `.whileTrue` | `shootCmd`/`intakeCmd`/`ejectCmd` | Superstructure | whileTrue, terminates via `requestStow()` on release | LOW |
| sim-mirror equivalents (`RobotBase.isSimulation()`-gated) | same as above | same | same | LOW (never active on real hardware) |

## Lower-severity / informational findings (not fixed — outside this session's minimal-fix scope)

- **SmartDashboard command buttons are live during autonomous.** `RobotContainer.dashboard()` exposes
  `"State: Intake"/"State: Eject"/"State: Stow"/"State: Shoot"` via `SmartDashboard.putData`, which are
  clickable over NetworkTables in any robot mode. Now that finding #2 is fixed, pressing one of these during
  a Superstructure-requiring auto step will *correctly cancel* that step (deterministic, no more silent
  concurrent corruption) — but it will still abort that step of the auto early if pressed. This requires a
  human to physically click a dashboard button mid-match; flagged for awareness, not fixed (would need a
  design decision, e.g. gating dashboard commands to teleop-only, which wasn't asked for here).
- **Default drivetrain command vs. autonomous.** `Robot.teleopInit()` correctly cancels the running
  autonomous command (`m_autonomousCommand.cancel()`), so the standard FRC "did the auto finish before the
  match phase changed" question is handled. If an auto finishes *before* the 15s autonomous period ends,
  the drivetrain's default (teleop joystick) command will schedule itself the moment nothing else requires
  the drivetrain — mitigated by the existing 10% deadband, but a driver bumping a stick during the
  auto-to-teleop gap would move the robot. This is a generic, well-known FRC pattern, not a defect
  introduced by this codebase; not fixed.
- **`Intake.agitatePivot()` — an infinite `.repeatedly()` command with zero subsystem requirement,
  scheduled directly (not via a Trigger).** `Superstructure.ensureAgitating()` schedules it
  (`CommandScheduler.getInstance().schedule(mAgitateCommand)`) and `setState()` explicitly cancels it when
  leaving `ALIGNING`/`SHOOTING`. This is safe today because exactly one call site owns its lifecycle with
  its own `isScheduled()` guard, but because it declares no requirement, the normal WPILib
  cancel-on-conflict safety net would not catch a future accidental double-schedule from elsewhere. Not a
  live bug (nothing else references it); flagged for future awareness if it's ever bound to a button or
  reused.
- **Dead code with the same unbounded-`waitUntil` pattern as the fixed `Orbit` bug:**
  `Shooter.indexJam()` (`Commands.waitUntil(this::isJammed)`, no timeout) and `Shooter.spin(double)` /
  `Shooter.dashSpin()` (`Commands.waitUntil(() -> shooterAtSpeed(rpm))`, no timeout). Confirmed via
  repository-wide grep: **none of these three methods are called anywhere** outside `Shooter.java` itself —
  not bound to a button, not a NamedCommand. Zero present-day impact, but a landmine for whoever wires one
  of these up later without adding a bound; worth a timeout before any future binding uses them.
- **NamedCommand instance-reuse constraint (design note, not a bug today).**
  `NamedCommands.registerCommand(name, command)` binds one Command *instance* per name for the program's
  lifetime. Reusing the same NamedCommand name twice within a single auto's `parallel` branches would
  attempt to schedule that one instance twice concurrently, which WPILib does not support cleanly.
  Confirmed via the same JSON walk used for finding #1: no current `.auto` file does this (each NamedCommand
  appears at most once per `parallel` block). Worth keeping in mind for future auto authoring.
- **`waitUntil`/`waitTime` inventory (per the audit's own goal list).** Only two real `waitUntil`-style
  constructs exist in the reachable command graph: `Superstructure.shootingSequence`'s
  `Commands.waitUntil(() -> !mShotInProgress)` (bounded indirectly by the state machine's own feed-timeout
  transition, confirmed in the NamedCommand table above) and `Intake.homing()`'s
  `.until(this::hardstop).withTimeout(1)` (explicitly bounded). No PathPlanner-native "wait until" condition
  block exists in any `.auto` file — confirmed by direct JSON traversal, only fixed-duration `"wait"` nodes
  appear. The two unbounded `waitUntil`s in dead Shooter code are covered above.

## Verification performed

- `AutoCommandSafetyTest`: RED (Orbit flagged in all 12 autos) → GREEN, both tests passing consistently.
- `SuperstructureCommandRequirementsTest`: RED (empty requirement set) → GREEN.
- `./gradlew compileJava`: BUILD SUCCESSFUL (both fixes applied together).
- `python SKILLS/run_headless_sim.py --run-seconds 12`: PASS.
- Full `./gradlew test`: 16/17, the one failure independently A/B-confirmed as the pre-existing flaky
  `LtNeutralAutoRegressionTest` stall check, unrelated to either fix (see "Causality check" above).
- `git status --short` / `git diff --stat` confirm the only files touched: `RobotContainer.java`,
  `Superstructure.java` (production), plus two new test files. Nothing else in the repository was modified.

## What this audit did not do

Per explicit instruction, drivetrain velocity architecture (`DriveRequestType`, `Slot0`, the
`OpenLoopVoltage`-vs-`Velocity` question) was not touched and is not in scope here — that remains
hardware-blocked per the Autonomous Velocity migration plan. This audit also did not re-tune `"Orbit"`'s
2.0s timeout, the existing `Shooting Sequence`/`Quick Shooting` feed timeouts, or re-baseline any
regression golden — all are pre-existing values or intentionally conservative new ones, not touched beyond
what each fix required.

## Continued audit, same day (new session): resolution guard + termination telemetry

**Scope:** picking up directly from this file's own findings, re-verifying two things the first pass
established by manual inspection rather than by an automated regression guard: (1) that every NamedCommand
any `.auto` file actually references resolves to something registered, and (2) whether autonomous
completion is observable at all in production telemetry (as opposed to only inside this repo's own JUnit
harness). Same constraint as before: drivetrain velocity architecture (`DriveRequestType`, `TunerConstants`,
SysId migration files) untouched.

**Correction to this doc's own file count:** the header above and the bugs-found section both say "every
12 `.auto` files" — there are actually **13** in `src/main/deploy/pathplanner/autos/` (`RB Neutral.auto` is
present; the Backlog section of `CLAUDE.md` had listed `LB Neutral`/`RB Neutral` as "deferred" from the
regression-suite's *golden* scope, which is a different thing from the NamedCommand-safety scope this audit
covers). Confirmed via direct directory listing — all 13 use the exact same six NamedCommands as the other
12 (`"Home Intake"`, `"Intake Start Sequence"`, `"Orbit"`, and either `"Shooting Sequence"` or `"Quick
Shooting"`), so nothing found by the original audit changes; both new regression tests below iterate
`Files.list()` over the real directory rather than a hardcoded count, so this was never a blind spot in the
tests themselves, only in this doc's prose.

### 3. New guard: every `.auto` NamedCommand reference actually resolves

`AutoCommandSafetyTest` (finding #1's guard) only checks that a NamedCommand used **inside a `parallel`
block** is on the `BOUNDED_NAMED_COMMANDS` allowlist — it never checked that the name resolves to anything
registered at all, in any position (parallel or plain sequential). Read PathplannerLib's own
`NamedCommands.getCommand()` (decompiled `PathplannerLib-java-2026.1.2-sources.jar`): an unregistered name
does **not** throw or fail `compileJava` — it prints a `DriverStation.reportWarning` and silently
substitutes `Commands.none()`. A typo'd or renamed NamedCommand reference in a `.auto` file would compile
fine, pass gate 1, and simply skip that step of the auto at runtime with nothing but an easy-to-miss
driver-station warning as evidence.

**New regression test:** `src/test/java/frc/robot/auto/AutoNamedCommandResolutionTest.java` — boots a real
`Robot()` in sim (needs `NamedCommands`'s registry actually populated, which only happens once
`RobotContainer`'s constructor runs, so this can't be the pure-JSON style `AutoCommandSafetyTest` uses),
walks every `.auto` file's command tree collecting every `"named"` command's name regardless of nesting or
position, and asserts `NamedCommands.hasCommand(name)` for each. A `collectNamedCommandNames()` static
helper does the traversal; its correctness (finds names nested arbitrarily deep, e.g. sequential-inside-
parallel) is covered by a new test in the existing `AutoCommandSafetyTest.java`
(`collectNamedCommandNamesFindsNamesRegardlessOfNesting`, pure JSON, no boot needed) rather than duplicating
boot machinery just to prove the traversal itself.

**Verified non-vacuous, not just written and trusted:** temporarily commented out `"Orbit"`'s
`NamedCommands.registerCommand` call in `RobotContainer.java`, reran the test — failed exactly as expected,
naming `"Orbit"` in the violation message. Reverted (confirmed via `git diff --stat`, zero-line diff
afterward), reran — passed. Currently all six registered NamedCommands (`"Home Intake"`, `"Orbit"`,
`"Shooting Sequence"`, `"Quick Shooting"`, `"Intake Start Sequence"`, `"Intake Stop"`) resolve; every name
any `.auto` file references is a subset of that six (`"Intake Stop"` is registered but not currently
referenced by any auto — dead registration, not a bug).

### 4. Real gap found and fixed: autonomous termination was not observable in production telemetry

`Robot.java`'s `autonomousInit()`/`teleopInit()` scheduled and cancelled the autonomous command but never
recorded whether it finished naturally, was cut off by the teleop transition, or was cancelled some other
way (e.g. a disable-triggered scheduler cancel). This repo's own JUnit harness can tell the difference
(`AutoRegressionTestBase` polls `CommandScheduler.getInstance().isScheduled(autoCommand)` from the test
thread) but **that path is test-only** — it is not RobotContainer's real `getAutonomousCommand()` command,
and none of that information reaches AdvantageKit. A real match's wpilog had no signal to distinguish "auto
completed" from "auto got interrupted" after the fact — exactly the "autonomous termination is observable"
goal named for this continued audit.

**Fix:** `Robot.java` gained a small, directly testable wrapper:

```java
static Command wrapAutonomousForTelemetry(Command autoCommand) {
  Logger.recordOutput("Auto/Running", true);
  Logger.recordOutput("Auto/EndedInterrupted", false);
  return autoCommand.finallyDo(interrupted -> {
    Logger.recordOutput("Auto/Running", false);
    Logger.recordOutput("Auto/EndedInterrupted", interrupted);
  });
}
```

`autonomousInit()` now wraps `m_robotContainer.getAutonomousCommand()` with this before scheduling it (and
still assigns the *wrapped* Command to `m_autonomousCommand`, so `teleopInit()`'s existing `.cancel()` call
correctly targets the scheduled instance, not an unscheduled original). `Command.finallyDo(BooleanConsumer)`
receives WPILib's own `interrupted` flag from the scheduler at end-time, so this distinguishes a natural
finish from any kind of interruption (teleop cancel, a disable-triggered cancel, or anything else) with no
new polling loop and no change to scheduling/requirements.

**Regression test (TDD, RED first):** `src/test/java/frc/robot/RobotAutoTerminationTelemetryTest.java`.
Bypasses the auto chooser entirely (its default selection is a trivial `Commands.none()` that finishes
before there's anything to observe — the same reason `AutoRegressionTestBase` bypasses it) and exercises
`wrapAutonomousForTelemetry` directly against two synthetic `WaitCommand`s in one test method (a second
`@Test` method in the same class would double-call `Logger.start()` within one JVM fork and throw
`IllegalThreadStateException`, the exact failure class discovered earlier the same day building the reverse
SysId workflow test): a short one left to finish naturally, then a long one cancelled mid-flight. Watched it
fail (`cannot find symbol: method wrapAutonomousForTelemetry`) before the method existed. **One real
correction made mid-implementation:** the first version of this test partitioned samples by a timestamp
this test thread captured itself via `Timer.getFPGATimestamp()`, comparing it against each wpilog record's
own timestamp — this produced a false failure (`Auto/Running=true` for phase 2 landed in the wrong
partition) traced to clock skew between this thread's clock read and AdvantageKit's own cycle-timestamp
bookkeeping for records written from outside the robot thread. Rewritten to compare the two boolean series
**by ordinal position** instead (both keys are always written together, in the same call, so they stay in
lockstep) — deterministic regardless of any cross-thread timing skew. A second real correction: `Auto/
EndedInterrupted` turned out to be write-on-change in the underlying log (repeated same-value writes
collapse to one sample) — confirmed empirically, not assumed — so the test asserts the deduped series
`[false, true]` (never spuriously true during the natural-finish phase, true exactly once after the cancel)
rather than one sample per phase.

**Verified:**
- `RobotAutoTerminationTelemetryTest`: RED (missing symbol) → GREEN, reran 2/2 consecutive passes.
- `./gradlew compileJava` / `compileTestJava`: BUILD SUCCESSFUL.
- `python SKILLS/run_headless_sim.py --run-seconds 12`: PASS.
- **Gate 3, independently cross-validated with a separate tool** (`python SKILLS/parse_akit_log.py --dump`),
  against the test's own freshly-produced wpilog: `/RealOutputs/Auto/Running` → `True, False, True, False`
  at t=0.022s/0.203s/0.623s/0.823s; `/RealOutputs/Auto/EndedInterrupted` → `False, True` at t=0.022s/0.823s
  — matching the test's own internal assertions exactly, via a tool that never shares code with the test.
- Full `./gradlew test`, run three times total this session: run 1 was 19/19 minus `LtNeutralAutoRegressionTest`'s
  stall check (independently A/B-confirmed pre-existing and unrelated via `git stash` on just `Robot.java`
  — baseline fails with the identical signature, no code from this session involved); run 2 was 19/19 minus
  `CommandSwerveDrivetrainSysIdSimWorkflowTest` (a different, untouched-by-this-session test — passed cleanly
  when rerun in isolation immediately after, consistent with this repo's already-documented class of
  real-wall-clock-timing flakiness under full-suite load, not a regression); run 3 was a clean 19/19. Neither
  of this session's two new tests failed in any of the three runs.
- `git status --short` / `git diff --stat` confirm exactly what changed: `Robot.java` (production, minimal
  diff), `AutoCommandSafetyTest.java` (one new test method), plus two new test files. Nothing else.

### Remaining unknowns / not in scope

- Whether the full-suite's pre-existing timing flakiness (now confirmed to affect at least two different
  test classes, not only `LtNeutralAutoRegressionTest`) has a common root cause worth investigating as its
  own item — out of scope for this audit, flagged for awareness.
- `Auto/Running`/`Auto/EndedInterrupted` are new telemetry only; they don't yet feed any dashboard/alerting
  and nothing consumes them automatically (e.g. no "flash a warning light if auto ended interrupted" logic)
  — this was scoped as "make it observable in a wpilog," not "build a live-alert consumer," matching what
  was asked.
- `RobotContainer.BOUNDED_NAMED_COMMANDS`'s eventual merge conflict with the still-unmerged
  `feature/autonomous-completion-trigger-framework` worktree (flagged in the first pass of this audit) is
  unchanged by this continuation.
