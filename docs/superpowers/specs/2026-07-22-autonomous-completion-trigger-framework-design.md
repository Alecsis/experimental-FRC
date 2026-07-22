# Autonomous Completion-Trigger Framework — Design

**Status:** Approved by mentor 2026-07-22, proceeding to implementation plan.
**Scope:** Software-only, fully executable and testable in simulation before September hardware access. No physical robot required.

## Problem

Three prior investigation-only audits (`docs/Autonomous_Architecture_Audit.md`, `docs/Path_Following_Tuning_Readiness_Audit.md`, `docs/SysId_Characterization_Checklist.md`) converge on the same headline finding: **the autonomous stack has no event-driven or sensor-driven feedback — every mechanism sequence is a fixed-duration guess.** All 13 `.auto` files start `Home Intake → wait 0.125s → parallel(path, Intake Start Sequence)` and rely exclusively on hand-picked timeouts (`intakeSequence(5.0)`, `shootingSequence(5.0)`/`shootingSequence(3.0)`, `mAlignTimeoutSeconds=1.0`) — none derived from a measured event. Benchmarked explicitly against 6328/Mechanical Advantage's event/trigger-driven autonomous composition style in the architecture audit, this is "the clearest specific place this codebase is furthest from a strong peer benchmark."

The full fix for this — note/fuel-presence sensing — is out of reach before September: it requires a hardware decision (beam-break vs. color sensor vs. current-signature) the mentor has deliberately not made yet, and MapleSim does not model game pieces, so a synthetic "note detected" signal cannot be honestly validated end-to-end in simulation.

What *is* available now, software-only and fully sim-testable, is the **framework** the eventual sensor will plug into: a generic "run until a condition is met, or a timeout elapses" primitive, with an initial library of pose/path-based conditions that are real (not synthetic) signals already available in simulation today.

## Goals

- Replace the ad hoc fixed-timeout pattern with one composable, testable primitive, usable by every current and future auto.
- Ship condition suppliers today that use only signals genuinely simulated (drivetrain pose, PathPlanner path-following state) — no invented sensor.
- Architect the primitive so a future note-presence signal (whatever technology is chosen in September) plugs in as "supply a real `BooleanSupplier`" — zero framework changes required later.
- As a side effect, formally close the previously-identified NamedCommand-hang bug class (audit item 1: `intakeCmd()` registered inside a `"parallel"` block, hung every auto until fixed via `intakeSequence(double)`) — any command built through the new primitive is self-terminating by construction.
- Zero behavior change to any existing auto unless a call site is explicitly opted in.

## Non-goals

- Note/fuel-presence sensing itself — explicitly deferred to September, needs a hardware decision.
- Retuning any existing auto's timeout values.
- Expanding the auto regression suite to the remaining 12 autos (tracked separately in `CLAUDE.md`'s backlog).
- Any change to `Superstructure`'s `SuperstructureState` enum or its `periodic()` arbitration logic.

## Design

### 1. `Superstructure.java` — generalize the existing timeout bridges

Verified against the actual current implementations (not assumed) — the two existing bridges are **not** structurally symmetric, which matters for how each one generalizes safely:

```java
// Current: intakeCmd() is Commands.startEnd(this::requestIntake, this::requestStow, this)
public Command intakeSequence(double timeoutSeconds) {
    return intakeCmd().withTimeout(timeoutSeconds);
}

// Current: NOT built on shootCmd()/startEnd -- calls the bounded requestShoot(timeoutSeconds)
// overload directly (distinct from shootCmd()'s unbounded requestShoot()) and waits on the
// state machine's own mShotInProgress flag, which Superstructure.periodic()'s SHOOTING case
// clears via requestStow() when mFeedTimeoutSeconds elapses.
public Command shootingSequence(double timeoutSeconds) {
    return Commands.sequence(
        Commands.runOnce(() -> requestShoot(timeoutSeconds)),
        Commands.waitUntil(() -> !mShotInProgress));
}
```

`intakeCmd()` is a plain `startEnd` command, so `.until(doneEarly)` composes safely — whichever race participant ends it, `intakeCmd()`'s own `end()` always fires `requestStow()`. `shootingSequence` has no such handler: it is a `Commands.sequence(...)` with no `end()`-time cleanup at all, so an external `.until(doneEarly)` wrapper that cancels it *before* `mFeedTimeoutSeconds` elapses would never call `requestStow()`, leaving `mShotInProgress` stuck `true` and the state machine wedged in `ALIGNING`/`SHOOTING` — a real regression, not a hypothetical one. The generalized form must therefore fold the early-exit condition into the same `waitUntil` and add explicit cleanup via WPILib's `finallyDo(BooleanConsumer)` decorator (confirmed API: "executed after the command's `end()` method is called," covering both normal completion and interruption — `docs/software/commandbased/command-compositions.rst`), so cleanup fires exactly once regardless of which condition ends it:

```java
public Command intakeSequence(double timeoutSeconds) {
    return intakeSequence(() -> false, timeoutSeconds);       // preserves current behavior exactly
}

public Command intakeSequence(BooleanSupplier doneEarly, double timeoutSeconds) {
    return intakeCmd().until(doneEarly).withTimeout(timeoutSeconds)
        .finallyDo(interrupted -> Logger.recordOutput("Superstructure/IntakeSequenceEndReason",
            (doneEarly.getAsBoolean() ? EndReason.CONDITION_MET : EndReason.TIMED_OUT).name()));
}

public Command shootingSequence(double timeoutSeconds) {
    return shootingSequence(() -> false, timeoutSeconds);     // preserves current behavior exactly
}

public Command shootingSequence(BooleanSupplier doneEarly, double timeoutSeconds) {
    return Commands.sequence(
            Commands.runOnce(() -> requestShoot(timeoutSeconds)),
            Commands.waitUntil(() -> !mShotInProgress || doneEarly.getAsBoolean()))
        .finallyDo(interrupted -> {
            boolean earlyExit = doneEarly.getAsBoolean();
            requestStow();   // no-op if the internal feed-timeout branch already called it
            Logger.recordOutput("Superstructure/ShootSequenceEndReason",
                (earlyExit ? EndReason.CONDITION_MET : EndReason.TIMED_OUT).name());
        });
}
```

Both existing single-argument overloads become thin callers of the new two-argument form with `() -> false` — i.e. "never finish early," which is exactly today's behavior, and `EndReason` always resolves to `TIMED_OUT` in that degenerate case (also exactly today's implicit behavior, now made visible). This is a backward-compatible generalization: every existing `NamedCommands.registerCommand(...)` call in `RobotContainer.java` keeps compiling and behaving identically until a call site is explicitly updated to pass a real condition. `requestStow()` is idempotent (just reasserts `mWantedState = STOWED; mShotInProgress = false;`), so the redundant call in the internal-timeout case is harmless.

### 2. New `utility/AutoTriggers.java` — condition supplier library

Pure `BooleanSupplier` factories over already-public, already-simulated signals. No vendor types beyond what `CommandSwerveDrivetrain`'s existing public API exposes (consistent with Rule 1's hardware isolation — this file never touches `com.ctre.*`/`com.revrobotics.*` directly).

- `withinToleranceOf(Pose2d target, double meters)` — from `drivetrain.getState().Pose`.
- `pathFollowingComplete()` — sourced from a new boolean exposed by `CommandSwerveDrivetrain` (see below).

### 3. `CommandSwerveDrivetrain.java` — expose path-active state via the existing push seam

`configureAutoBuilder()` already registers `PathPlannerLogging.setLogActivePathCallback(...)` (used today to log `Odometry/Trajectory`). Per the precedent established in `docs/claudex/architecture.md`'s "Observability instrumentation pattern" section ("look for an existing push callback/hook before adding a new vendor import"), this same callback is extended to also update a package-visible boolean (path list non-empty ⇒ following; empty ⇒ complete/idle). No new PathPlanner import, no new vendor coupling.

### 4. `RobotContainer.java` — no structural change

Existing `NamedCommands.registerCommand(...)` calls are unaffected. Optionally, once this lands, one or two registrations may be updated to pass a real `AutoTriggers` condition instead of the implicit `() -> false` — that adoption is a separate, later, low-risk change, not part of this design's landing diff. A small `Set<String>` allowlist of "known-bounded" NamedCommand names is added beside the registration block, consumed by the new structural test (§5).

### 5. New test: `src/test/java/frc/robot/auto/AutoCommandSafetyTest.java`

Parses every `.auto` file's JSON and asserts every NamedCommand referenced inside a `"parallel"` block is present in the allowlist from §4. Pure static JSON parsing — no `CommandScheduler`, no sim, no `forkEvery`/singleton concerns (matches the "Auto regression testing pattern" section's own precedent about JVM/singleton isolation, but this test needs none of that machinery since it never constructs a robot). This is the structural guard that prevents the original `intakeCmd()`-hang bug class from recurring under a *different* NamedCommand in the future.

Verified by direct inspection of all 13 `.auto` files (not assumed): exactly four distinct NamedCommand names ever appear inside a `"parallel"` block across the whole deploy — `Intake Start Sequence`, `Orbit`, `Shooting Sequence`, `Quick Shooting`. `Home Intake` and `Intake Stop` are the other two registered NamedCommands but never appear inside a `"parallel"` block (used sequentially), so the allowlist only needs those four entries.

### 6. New test: `src/test/java/frc/robot/SuperstructureConditionalSequenceTest.java`

Unit test in the style of the existing `SuperstructureEjectingTest`. Two cases:
- Condition flips true before the timeout ⇒ command finishes early via `.until(...)`.
- Condition never flips ⇒ command finishes via `.withTimeout(...)` at the expected time, matching today's behavior exactly.

### 7. Extend `LtNeutralAutoRegressionTest`

Add one assertion that the new end-reason telemetry (§8) is logged and takes a sane value for this one already-existing golden run — a cheap end-to-end smoke check using the harness that already exists. No new auto, no new golden.

### 8. Telemetry

- `Superstructure/IntakeSequenceEndReason` / `Superstructure/ShootSequenceEndReason` — logged enum/string, `CONDITION_MET` vs `TIMED_OUT`. Also directly closes a separately-identified audit finding (`Autonomous_Architecture_Audit.md` §2/§5 roadmap item 2) that the `ALIGNING→SHOOTING` timeout-vs-genuine-completion split is currently invisible in logs — this generalizes that same visibility to the intake side.
- `Drive/IsFollowingAutoPath` (boolean) — the path-active flag from §3, reusable beyond this feature.

## Affected classes

| Class | Change |
|---|---|
| `Superstructure.java` | Generalize `intakeSequence`/`shootingSequence` to accept an optional `BooleanSupplier`; add end-reason telemetry |
| `utility/AutoTriggers.java` (new) | Condition supplier library |
| `CommandSwerveDrivetrain.java` | Expose `Drive/IsFollowingAutoPath` via the existing `PathPlannerLogging` callback |
| `RobotContainer.java` | Add the bounded-NamedCommand allowlist constant; no behavioral change |
| `src/test/java/frc/robot/auto/AutoCommandSafetyTest.java` (new) | Structural safety test |
| `src/test/java/frc/robot/SuperstructureConditionalSequenceTest.java` (new) | Conditional-sequence unit test |
| `src/test/java/frc/robot/auto/LtNeutralAutoRegressionTest.java` | One added telemetry assertion |

## Complexity & risk

- **Complexity: Low-Medium.** Additive method overloads, a handful of pure functions, one existing-seam reuse. No change to `Superstructure`'s state machine or `periodic()` arbitration.
- **Risk: Low.** Every existing call site is unaffected until explicitly opted in — default behavior is byte-for-byte identical to today, so this cannot regress `LT Neutral`'s (already-flagged-stale, per `CLAUDE.md`) golden by itself. `AutoCommandSafetyTest` needs no scheduler/sim machinery, so it carries no `forkEvery`/singleton risk.
- **Tech debt: net negative (reduces it).** Formalizes the ad hoc timeout pattern that already caused one production bug (the `intakeCmd()` hang), and is the exact seam September's note sensor needs — that later work becomes "supply a real `BooleanSupplier` to an existing overload," not a second plumbing project.

## Alternatives considered

1. **Structural NamedCommand safety test alone** (audit roadmap item 1, no framework generalization). Real value, zero risk, but only prevents recidivism of one already-fixed bug — doesn't improve autonomous capability. Subsumed by this design: any command built through the new primitive is bounded by construction, so the safety property comes for free rather than as a separate bolt-on.
2. **Expand the auto regression suite to the remaining 12 autos.** Highest raw coverage value, but mechanical per-auto toil applying an already-built harness, not a framework capability change — explicitly out of scope per the requesting mentor's "not a single auto" framing. Independent backlog item, unaffected by this design.
3. **`VisionIOSim` noise/dropout injection + trust-filter exercise.** Valuable, but scoped to `Vision`, not autonomous sequencing; its payoff (confidence-aware behavior) is a separate, larger downstream item.
4. **Note-presence sensing directly.** Correct long-term diagnosis (the audit's own "single most consequential reliability gap"), but requires a hardware decision not yet made and isn't simulatable today (no game-piece physics in MapleSim). This design is the portion of that fix that's available now, without faking sensor physics.
