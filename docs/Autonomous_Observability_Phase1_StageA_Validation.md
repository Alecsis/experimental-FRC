# Autonomous Observability Phase 1, Stage A — Signal Validation

**Date:** 2026-07-28
**Purpose:** Document that Stage A of `docs/Autonomous_Observability_Phase1_Plan.md` — validating
the F1 (`Intake.hasGamePiece()`) and F3 (`Shooter.isJammed()`) mechanism-success signals identified
in `docs/Autonomous_Recovery_Audit.md` — is complete. **This document validates signals only. It
does not authorize, design, or implement any recovery behavior.** Neither signal is wired into
`AutonomousHealthMonitor`, `Superstructure`, or any command-cancellation/behavior-altering logic.
Both remain exactly what the Readiness Assessment (`docs/Autonomous_Recovery_Readiness_Assessment.md`,
§4, "not prerequisites — can start independently") scoped them as: isolated, drivetrain-independent
mechanism signals, safe to validate ahead of the chassis-PID fix that blocks everything else in that
assessment.

**Status update 2026-07-30:** The signal-validation work is complete and remains valid. The phrase
"chassis-PID fix that blocks everything else" is historical wording from the pre-pose-fix assessment;
the corrected sim baseline removed that blocker, but no recovery behavior is authorized by this document.

---

## 1. Purpose (expanded)

The Recovery Audit (F1, F3) and the Readiness Assessment (§2's signal-trust table, §4's prerequisite
ordering) both identified `hasGamePiece`/`isJammed` as real, low-risk, high-confidence signals that
could be validated independently of the drivetrain-tracking blocker. Validating a signal means:
proving it reads correctly under a controlled condition, not wiring it into a decision. This document
records that validation — new getters/test hooks, two new JUnit tests, and full-suite regression
evidence — and explicitly draws the line at signal correctness, leaving "what should happen when the
signal fires" as future, separately-scoped work.

## 2. F1 — Intake `hasGamePiece()` validation

**Addition:** `Intake.java:161-165` (`public boolean hasGamePiece()`) — a direct passthrough to
`inputs.hasGamePiece`, an `IntakeIOInputs` field that was already populated by `IntakeIOSim` and
already AutoLogged, but had no getter anywhere in the codebase before this change (confirmed via
grep — no prior reference existed).

**Simulation injection path:** `Intake.java:167-172` adds package-private
`provideGamePieceForTest()`, which delegates to a new `IntakeIOSim.addGamePieceForTest()`
(`IntakeIOSim.java:161-166`). That method calls IronMaple's own
`IntakeSimulation.addGamePieceToIntake()` directly — a plain counter increment against the
simulation's existing capacity model, no new field/collision physics involved. Both methods return
`false` (rather than throwing) if the lazily-constructed `IntakeSimulation` doesn't exist yet, which
is the same defensive pattern already used elsewhere in `IntakeIOSim`.

**Validation test:** `src/test/java/frc/robot/subsystems/intake/IntakeGamePieceSignalTest.java` (new,
1 test). Boots a real `Robot()` via `startCompetition()`/`SimHooks`, asserts `hasGamePiece()` reads
`false` before any piece is provided, injects one via `provideGamePieceForTest()`, steps sim time, and
asserts `hasGamePiece()` now reads `true`. Deliberately placed in package
`frc.robot.subsystems.intake` for package-private access to the test hook — same convention as the
pre-existing `IntakeJamRecoveryTest`.

**Limitations (explicitly not addressed by this work):**
- **Sim-only.** `hasGamePiece()` reads `inputs.hasGamePiece`, which `IntakeIOReal` never sets — on
  real hardware this always reads `false` today. No sensor exists on the physical robot to back this
  signal (matches the Readiness Assessment's own §2 classification: "Yes, in sim... Never populated
  on real hardware — not trustworthy there because it doesn't exist there, not because it's wrong").
- **No `Superstructure` consumer added.** Neither `Superstructure.INTAKING` nor any other state reads
  `hasGamePiece()`. It is validated as a correct, readable signal — not yet used to end an intake
  sequence, gate a NamedCommand, or influence any decision.

## 3. F3 — Shooter `isJammed()` validation

**Visibility change:** `Shooter.java:118-122` — `isJammed()` was `private` (referenced only by the
unused, dead-code-adjacent `indexJam()` per the Autonomous Command Lifecycle Audit's lower-severity
findings) and is now package-private, matching `Intake`'s already-established test-access pattern
(`Intake.java:168-177`).

**Test override pattern:** `Shooter.java` gained `forceJamConditionForTest(double, double)` and
`clearJamOverrideForTest()` (byte-for-byte copy of `Intake`'s own jam-test approach — a boolean flag
plus two forced-value fields, consumed at the top of `periodic()` before `Logger.processInputs` runs)
to force `indexStatorCurrentAmps`/`indexVelocityRadsPerSec` to arbitrary values, bypassing
`ShooterIOSim`'s inability to model a physical jam (no load/obstruction physics).

**AdvantageKit logging addition:** `Shooter.java`'s `periodic()` gained
`Logger.recordOutput("Shooter/Jammed", isJammed())`, alongside the pre-existing
`SmartDashboard.putBoolean("Shooter/Index Stall", ...)` call. This closes the exact gap the Recovery
Audit's F3 and the Readiness Assessment's §2 signal-trust table both flagged: `isJammed()`'s only
prior output was dashboard-only, meaning it "never reaches AdvantageKit/wpilog, so it can't be
reviewed post-match or read by anything that consumes `Logger` outputs the way every other signal
does." `Shooter/Jammed` is now a real wpilog/AdvantageKit output.

**Validation test:** `src/test/java/frc/robot/subsystems/shooter/ShooterJamSignalTest.java` (new, 1
test, new package `frc.robot.subsystems.shooter` under `src/test`). Boots a real `Robot()`, asserts
`isJammed()` reads `false`, forces a jam condition (`100.0` A stator current, `0.0` rad/s velocity —
`isJammed()`'s real trip condition), asserts `isJammed()` now reads `true`, clears the override, and
asserts it reads `false` again.

**Limitation (explicitly not addressed by this work):** no automatic jam recovery was added.
`isJammed()` is validated as a correctly-computed, now-loggable signal — nothing consumes it to
trigger a recovery action, retry, or state change. `Superstructure.SHOOTING`'s transition logic is
unmodified.

## 4. Verification evidence

**TDD RED, from the prior implementation session (`CLAUDE.md`'s Stage A/F1+F3 Next-up bullet, not
re-reproduced in this documentation-only session per its own no-code-changes scope):** both new test
files were written first and watched fail with 8 real compiler errors — missing `hasGamePiece()`,
missing `provideGamePieceForTest()`, missing `addGamePieceForTest()`, `isJammed()`'s prior `private`
access, and the missing `forceJamConditionForTest`/`clearJamOverrideForTest` hooks — before any
production code was added.

**GREEN, re-verified fresh in this session** (`JAVA_HOME` pointed at the WPILib JDK 17, current
`refactor/hybrid` working tree):

- **Gate 1 (`./gradlew compileJava`):** BUILD SUCCESSFUL.
- **Gate 2 (`python SKILLS/run_headless_sim.py --run-seconds 12`):** PASS — robot init reached, 12s
  of disabled-mode periodic loops, no exceptions.
- **Targeted + related tests** (`./gradlew test --tests ...`, six classes), confirmed via the
  generated JUnit XML `tests=`/`failures=` attributes, not console text alone — **6/6 passed, 0
  failures**:

  | Test class | Result |
  |---|---|
  | `frc.robot.subsystems.intake.IntakeGamePieceSignalTest` (new) | 1/1 passed |
  | `frc.robot.subsystems.shooter.ShooterJamSignalTest` (new) | 1/1 passed |
  | `frc.robot.subsystems.intake.IntakeJamRecoveryTest` (related — pre-existing jam-test pattern this work mirrors) | 1/1 passed |
  | `frc.robot.SuperstructureEjectingTest` (related — Superstructure state-transition regression) | 1/1 passed |
  | `frc.robot.SuperstructureCommandRequirementsTest` (related — confirms the command-requirements fix from the Lifecycle Audit is undisturbed) | 1/1 passed |
  | `frc.robot.RobotLifecycleTest` (related — general boot/lifecycle regression) | 1/1 passed |

- **Full suite (`./gradlew test`):** **43 tests completed, 1 failed.**

## 5. Known limitation — the one full-suite failure

The single failure is `frc.robot.auto.LtNeutralAutoRegressionTest.regressionCheck()`:

```
LT Neutral stalled at t=5.48s: moved 0.049m over the preceding 0.98s (threshold 0.050m), still
within 0.50s of the last fresh path setpoint (t=5.32s) ==> expected: <false> but was: <true>
```

This is unrelated to Stage A for two independent reasons:

1. **It is not in the diff.** `Intake.java`, `IntakeIOSim.java`, and `Shooter.java` (the three files
   this work touched) have no relationship to `LtNeutralAutoRegressionTest`'s drivetrain-tracking
   stall check — that test exercises `CommandSwerveDrivetrain`/`PPHolonomicDriveController` path
   following, a completely separate subsystem from intake/shooter jam signals.
2. **It matches the project's already-documented flaky signature exactly.** `CLAUDE.md`'s
   session history and the Readiness Assessment (§3.4) both record this same test failing repeatedly
   with margins as tight as 0.027m–0.049m against the identical 0.050m threshold, independently
   A/B-confirmed as pre-existing flakiness on multiple prior occasions (most recently the Phase 0.5
   audit's own full-suite run, 0.045m vs 0.050m). This run's 0.049m margin is consistent with that
   same razor-thin-margin class, not a new failure signature.

## 6. Scope boundary

Explicitly confirmed by reading the current diff and the class definitions directly, not assumed:

- **`AutonomousHealthMonitor` is unchanged.** Neither `hasGamePiece()` nor `isJammed()` appears
  anywhere in `src/main/java/frc/robot/auto/AutonomousHealthMonitor.java`. Its four `Auto/Health/*`
  thresholds and its `RecoveryReason` computation are exactly as Phase 0.5 left them.
- **No thresholds changed.** This work introduces no new threshold, tolerance, or tuning constant —
  `hasGamePiece()`/`isJammed()` are boolean passthroughs over existing sensor/simulation state.
- **No recovery actions implemented.** Nothing cancels a command, retries an intake, triggers a
  `pathfindToPose` reroute, or alters `Superstructure`'s state machine based on either signal.
- **No autonomous behavior changed.** `Superstructure.INTAKING` and `Superstructure.SHOOTING`'s
  `periodic()` transition logic is byte-for-byte unmodified by this diff — confirmed via
  `git diff -- src/main/java/frc/robot/subsystems/Superstructure.java` returning empty. The only
  behavioral-surface change anywhere in this diff is the new `Logger.recordOutput("Shooter/Jammed",
  ...)` call, which writes a log entry and nothing else.

This matches the Readiness Assessment's own framing of F1/F3 as "the safest, most immediately
actionable items in the entire recovery backlog" precisely because they don't touch drivetrain
tracking or the `AutonomousHealthMonitor` decision surface at all. The next item to close out Stage A
per `docs/Autonomous_Observability_Phase1_Plan.md`'s own acceptance criteria is the replay-based
before/after comparison workflow (not started); Stage B/C remain hard-blocked on physical-robot
access, unaffected by this work.
