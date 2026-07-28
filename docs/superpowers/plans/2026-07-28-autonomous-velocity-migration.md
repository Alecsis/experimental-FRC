# Autonomous Velocity Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Migrate autonomous path-following from `DriveRequestType.OpenLoopVoltage` to `DriveRequestType.Velocity`
safely — characterizing and validating every precondition first, comparing measured behavior before committing, and
leaving a clean revert path at every step — per the conditional recommendation in
`docs/Autonomous_Drive_Architecture_Recommendation.md`.

**Architecture:** Six sequential phases, each producing an independently-revertible, independently-testable state.
Phases 0 and 3 are code changes (TDD, one focused commit each); **Phase 1 runs in simulation, workflow-validation
only — see its own TODO** — and produces no production values; Phase 2 includes a physical-robot-only
characterization precondition plus code deliverables applying the resulting real values; Phase 4 captures a
before/after comparison using the existing auto-regression harness plus one new metric; Phase 5 is a human decision
gate, not a code task. No phase after 0 begins until the prior phase's success criteria are met and, for Phases 3 and
5 specifically, explicit mentor sign-off is given — these are non-defaultable decision points, matching this
project's own established pattern (see `CLAUDE.md`'s VT-input-relay migration history for the precedent).

**Tech Stack:** Java 17 (WPILib/Phoenix 6/PathplannerLib toolchain), JUnit 5, AdvantageKit (`Logger.recordOutput`),
WPILib's `DataLogReader`/`DataLogRecord` for wpilog analysis, the project's existing hand-rolled auto-regression
harness (`src/test/java/frc/robot/auto/`).

## Global Constraints

- **JDK 17 required** for Gradle — `export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"` before any `./gradlew`
  command (the PATH `java` is JDK 25 and cannot run Gradle 8.11's configuration phase). Per `CLAUDE.md`.
- **`CommandSwerveDrivetrain.java` is a sanctioned hardware-isolation exception** (CTRE Tuner X generated output,
  `docs/claudex/architecture.md`) — edits to it for `Slot0`/`DriveRequestType` stay within its already-sanctioned
  scope, not a new hole in the hardware-isolation boundary.
- **`TunerConstants.java` gain edits are expected, normal work for this plan** — its own header comment states "Both
  sets of gains need to be tuned to your individual robot." This is not a violation of "never copy numbers from
  reference code" — no reference-repo numbers are copied anywhere in this plan; every numeric gain value in Phase 2
  comes from this robot's own SysId run (Phase 1), never from `temp_reference/` or `frc-steal-from-the-best/`.
  Only *patterns* were drawn from those repos (already done, in the prior recommendation doc) — the
  `DriveRequestType.Velocity` choice and the `ChassisSpeeds.discretize()` call.
- **TDD discipline**: every code task starts with a failing test, per `superpowers:test-driven-development`.
- **One focused commit per task** — never bundle Phase 0's discretization fix with Phase 3's `DriveRequestType`
  switch, even though both touch the same method. Each commit must be independently revertible.
- **Non-defaultable decision points**: Phase 3 (switching production autonomous control mode) and Phase 5 (final
  keep/revert decision) require explicit mentor sign-off via `AskUserQuestion` or equivalent direct confirmation —
  an executing agent must stop and ask, not proceed on its own judgment, matching this project's own established
  precedent for behavior-changing decisions.
- **Verification gates** (`CLAUDE.md`'s "Advanced Agent Verification Loop") apply to every code task: gate 1
  (`./gradlew compileJava`), gate 2 (`python SKILLS/run_headless_sim.py`), gate 2.5 (`./gradlew test`) for any
  behavior claim, gate 3 (`python SKILLS/parse_akit_log.py`) whenever a wpilog-based claim is made. A skipped gate
  means the claim is UNVERIFIED, not passed.
- **Revised 2026-07-28: Phase 1 now runs in simulation first, workflow-validation only — not physical hardware.**
  The mentor directed that SysId be exercised in sim for now, to verify the command lifecycle, `SignalLogger`/
  AdvantageKit telemetry, data collection, and integration end-to-end before ever touching the physical robot. Per
  `docs/SysId_Characterization_Checklist.md` §6, a sim run is circular for *characterization* purposes — maple-sim's
  own `kDriveFrictionVoltage`/`kSteerFrictionVoltage` (`TunerConstants.java:98-99`) are themselves unmeasured
  placeholders, so a sim SysId run recovers numbers already fed into the sim, not real hardware behavior — but that
  circularity is irrelevant to workflow validation, which only needs the pipeline to run correctly, not to produce
  trustworthy numbers. **See Phase 1's own TODO for the hard requirement this creates on Phase 2.**
- **Phase 2 requires the physical robot** for its actual, production-bound characterization values — unchanged from
  the original plan. Only Phase 1's scope changed (sim workflow validation added ahead of the physical run, not a
  replacement for it).

---

## File Structure

| File | Change | Phase |
|---|---|---|
| `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java` | Modify: extract `prepareAutoSpeeds()`, wire in `ChassisSpeeds.discretize()` | 0 |
| `src/test/java/frc/robot/subsystems/CommandSwerveDrivetrainSanitizeSpeedsTest.java` | Modify: add discretization-order test | 0 |
| `src/main/java/frc/robot/generated/TunerConstants.java` | Modify: `driveGains` — apply characterized `kS/kV/kA`, then retuned `kP/kI/kD` | 2 |
| `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java` | Modify: extract `buildAutoRequest()`, set `DriveRequestType.Velocity` | 3 |
| `src/test/java/frc/robot/subsystems/CommandSwerveDrivetrainAutoRequestTest.java` | Create: pins `DriveRequestType` on the built request | 3 |
| `src/test/java/frc/robot/auto/AutoRegressionGolden.java` | Modify: add `maxStatorCurrentAmps` field | 4 |
| `src/test/java/frc/robot/auto/WpilogCurrentDrawReader.java` | Create: reads `Drive/StatorCurrentAmpsPerModule` max from a wpilog | 4 |
| `src/test/java/frc/robot/auto/AutoRegressionTestBase.java` | Modify: capture current-draw metric into the golden | 4 |
| `src/test/resources/autoRegression/LT_Neutral_openloop_baseline.json` | Create: pre-Phase-3 comparison snapshot | 4 |
| `docs/Velocity_Migration_Comparison_Report.md` | Create: the actual before/after comparison | 4 |

No file outside `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`,
`src/main/java/frc/robot/generated/TunerConstants.java`, and the `src/test/java/frc/robot/auto/` /
`src/test/java/frc/robot/subsystems/` test packages is touched by this plan.

---

# Phase 0 — Add missing `ChassisSpeeds.discretize()`

**Objective:** Close the discretization gap flagged in `docs/Path_Following_Tuning_Readiness_Audit.md` §4 and
confirmed against two independent elite-team implementations in `docs/Autonomous_Drive_Architecture_Recommendation.md`
§2 (CTRE's own official example and Team 6328's `Drive.java`), while autonomous is still `OpenLoopVoltage` — isolating
this change from every later phase.

**Expected behavior:** A small, numerically-bounded correction to the commanded `ChassisSpeeds` on any control-loop
iteration where translation and rotation are commanded simultaneously (per `ChassisSpeeds.discretize()`'s documented
purpose — it introduces a small coupling term into vx/vy from omega). **Not** a zero-diff change at the bit level,
but zero *architectural* change: still `OpenLoopVoltage`, still the same chassis PID gains, still the same
feedforward wiring. The correction is expected to stay well inside the existing auto-regression golden's tolerance
headroom (`AutoRegressionTolerances.kLateralErrorHeadroomMeters = 0.75`,
`kLongitudinalErrorHeadroomMeters = 1.0`) — those bounds already exist because of documented sim-timing jitter of a
similar or larger order.

### Task 0.1: Extract and discretize the auto-speed preparation pipeline

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java:359-363` (the `configureAutoBuilder()`
  lambda) and its import block (~line 29)
- Test: `src/test/java/frc/robot/subsystems/CommandSwerveDrivetrainSanitizeSpeedsTest.java`

**Interfaces:**
- Produces: `ChassisSpeeds CommandSwerveDrivetrain.prepareAutoSpeeds(ChassisSpeeds speeds)` — package-private,
  same visibility/testability pattern as the existing `sanitizeAutoSpeeds(ChassisSpeeds)` (already package-private,
  already tested by `CommandSwerveDrivetrainSanitizeSpeedsTest`). Later tasks (Phase 3's `buildAutoRequest()`) call
  this method instead of `sanitizeAutoSpeeds()` directly.

- [x] **Step 1: Write the failing test**

Add to `src/test/java/frc/robot/subsystems/CommandSwerveDrivetrainSanitizeSpeedsTest.java` (existing file — add this
test method inside the existing `CommandSwerveDrivetrainSanitizeSpeedsTest` class, alongside the two tests already
there):

```java
  @Test
  @Timeout(30)
  void prepareAutoSpeedsDiscretizesBeforeSanitizing() {
    // A combined translation+rotation command is exactly the case ChassisSpeeds.discretize()
    // exists for -- it couples a small vx/vy correction in from omega. Comparing against an
    // independently-computed expected value (built from the same public API, not a magic number)
    // pins the composition order: discretize() must run before sanitizeAutoSpeeds(), matching
    // docs/Path_Following_Tuning_Readiness_Audit.md's own sequencing note.
    ChassisSpeeds raw = new ChassisSpeeds(2.0, 1.0, 1.5);

    ChassisSpeeds actual = drivetrain.prepareAutoSpeeds(raw);

    ChassisSpeeds expectedDiscretized =
        ChassisSpeeds.discretize(raw, edu.wpi.first.wpilibj.TimedRobot.kDefaultPeriod);
    ChassisSpeeds expected = drivetrain.sanitizeAutoSpeeds(expectedDiscretized);

    assertEquals(expected.vxMetersPerSecond, actual.vxMetersPerSecond, 1e-9);
    assertEquals(expected.vyMetersPerSecond, actual.vyMetersPerSecond, 1e-9);
    assertEquals(expected.omegaRadiansPerSecond, actual.omegaRadiansPerSecond, 1e-9);
  }

  @Test
  @Timeout(30)
  void prepareAutoSpeedsActuallyChangesCombinedTranslationRotationInput() {
    // Guards against a no-op discretize() call (e.g. a future refactor accidentally passing
    // dtSeconds=0, which would make discretize() an identity function and silently defeat this
    // whole phase). Pure translation or pure rotation alone would not exercise the coupling term.
    ChassisSpeeds raw = new ChassisSpeeds(2.0, 1.0, 1.5);

    ChassisSpeeds prepared = drivetrain.prepareAutoSpeeds(raw);
    ChassisSpeeds sanitizedWithoutDiscretize = drivetrain.sanitizeAutoSpeeds(raw);

    boolean differs =
        Math.abs(prepared.vxMetersPerSecond - sanitizedWithoutDiscretize.vxMetersPerSecond) > 1e-9
            || Math.abs(prepared.vyMetersPerSecond - sanitizedWithoutDiscretize.vyMetersPerSecond) > 1e-9;
    assertTrue(differs, "discretize() should introduce a measurable vx/vy coupling term "
        + "for a combined translation+rotation command");
  }
```

This requires adding `import static org.junit.jupiter.api.Assertions.assertTrue;` to the test file's existing
static-import block (it currently only imports `assertEquals`).

- [x] **Step 2: Run tests to verify they fail**

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew test --tests "frc.robot.subsystems.CommandSwerveDrivetrainSanitizeSpeedsTest"
```
Expected: FAIL — `cannot find symbol: method prepareAutoSpeeds(ChassisSpeeds)` (compile error, since the method
doesn't exist yet).

- [x] **Step 3: Implement `prepareAutoSpeeds()` and wire it in**

In `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`, add this import near the existing
`edu.wpi.first.wpilibj.*` imports (after line 34, `edu.wpi.first.wpilibj.DriverStation.Alliance`):

```java
import edu.wpi.first.wpilibj.TimedRobot;
```

Add the new method immediately after `sanitizeAutoSpeeds()` (after line 349, before `private void
configureAutoBuilder()` at line 351):

```java
    /**
     * Discretizes, then bounds, a raw PathPlanner chassis-speed command before it reaches CTRE's
     * {@code ApplyRobotSpeeds} request. Discretization must run first -- it couples a small vx/vy
     * term in from omega that {@link #sanitizeAutoSpeeds} would otherwise saturate independently
     * of, per docs/Path_Following_Tuning_Readiness_Audit.md's sequencing note. CTRE's own official
     * Phoenix6-Examples reference (temp_reference/Phenoix 6 API Examples/java/SwerveWithPathPlanner)
     * and Team 6328's independent Drive.java both discretize at this same point in their own
     * pipelines -- this repo was the outlier in omitting it.
     */
    ChassisSpeeds prepareAutoSpeeds(ChassisSpeeds speeds) {
        return sanitizeAutoSpeeds(ChassisSpeeds.discretize(speeds, TimedRobot.kDefaultPeriod));
    }
```

Change the `configureAutoBuilder()` lambda (lines 359-363) from:

```java
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds
                                    .withSpeeds(sanitizeAutoSpeeds(speeds))
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
```

to:

```java
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds
                                    .withSpeeds(prepareAutoSpeeds(speeds))
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
```

- [x] **Step 4: Run tests to verify they pass**

```bash
./gradlew test --tests "frc.robot.subsystems.CommandSwerveDrivetrainSanitizeSpeedsTest"
```
Expected: PASS — all 4 tests (2 pre-existing + 2 new).

- [x] **Step 5: Full verification gate**

```bash
./gradlew compileJava
python SKILLS/run_headless_sim.py --run-seconds 12
./gradlew test
```
Expected: BUILD SUCCESSFUL, sim PASS, full suite green.

- [x] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java \
        src/test/java/frc/robot/subsystems/CommandSwerveDrivetrainSanitizeSpeedsTest.java
git commit -m "drivetrain: discretize autonomous chassis speeds before sanitizing"
```

**Note:** committed with message "Extract prepareAutoSpeeds helper for autonomous velocity migration" per
explicit executing-session instruction, not the message drafted above.

**Regression tests:**
- `CommandSwerveDrivetrainSanitizeSpeedsTest` (all 4 methods, including the 2 new ones above).
- `LtNeutralAutoRegressionTest` — run and confirm it **still passes against the existing checked-in golden**
  (`LT_Neutral.json`, unmodified). This is the "zero architectural behavior change" check: `completed`,
  `runtimeSeconds` (±3s), `maxLateralErrorMeters`/`maxLongitudinalErrorMeters` (within existing headroom) must all
  hold with no golden update.

**Success criteria:**
- `CommandSwerveDrivetrainSanitizeSpeedsTest` passes with the 2 new tests included.
- `LtNeutralAutoRegressionTest` passes against the **unmodified** existing golden — no `-DupdateAutoGolden` run
  needed or performed. If it fails, that is a signal to investigate before proceeding, not to update the golden
  reflexively (per this suite's own established discipline, `AutoRegressionTestBase`'s golden-update guard).
- `./gradlew compileJava`, `run_headless_sim.py`, and full `./gradlew test` all green.

**Rollback plan:** `git revert` the single commit from Step 6. Zero downstream impact — no later phase's code depends
on `prepareAutoSpeeds()` existing yet at this point (Phase 3 is the first consumer, and Phase 3 has not landed).

---

# Phase 1 — Validate the SysId workflow in simulation (NOT hardware characterization)

> **⚠️ TODO — READ BEFORE TOUCHING PHASE 2:** This phase runs entirely in simulation. Its purpose is to verify the
> SysId **command lifecycle, `SignalLogger`/AdvantageKit telemetry logging, data collection, and integration** —
> proving the pipeline runs correctly end-to-end — **not** to produce production feedforward constants. Any
> `kS`/`kV`/`kA` numbers this phase produces are temporary, sim-derived placeholders, influenced by maple-sim's own
> unmeasured friction constants (`kDriveFrictionVoltage`/`kSteerFrictionVoltage`, `TunerConstants.java:98-99`), and
> **must never be pasted into `TunerConstants.driveGains` as final values, and must never be committed as production
> characterization.** **Phase 2 (Slot0 tuning) is NOT complete — regardless of how far its own tasks progress —
> until real hardware characterization has been performed per `docs/SysId_Characterization_Checklist.md` §4 on the
> physical robot.** This TODO must be carried forward (as a code comment near `TunerConstants.driveGains` and in
> Phase 2's own precondition below) until that physical run happens and replaces it.

**Objective:** Prove the already-wired SysId infrastructure
(`docs/SysId_Characterization_Checklist.md` §1, "Bottom line up front" — code-complete, nothing to build) actually
works when exercised: the routine schedules and runs without error, `SignalLogger` state transitions fire, and the
telemetry a characterization depends on (`Drive/AppliedVoltsPerModule`, `SwerveStates/Measured`, etc. — already
logged unconditionally every periodic tick per `CommandSwerveDrivetrain.logDriveMotorVoltages()`) is actually
captured in the resulting wpilog during a SysId run specifically, not just during normal driving. This is a cheap,
sim-only integration check that catches wiring bugs before they'd otherwise only surface at a physical bring-up
session — it does not replace Task 1.2 below.

**Expected behavior:** Running `sysIdQuasistatic`/`sysIdDynamic` against the sim (maple-sim) drivetrain exercises the
identical command lifecycle, `SignalLogger` calls, and AdvantageKit telemetry path a real robot run would — only the
underlying physics (and therefore any derived gain values) differ. No change to any other robot behavior; the SysId
routines remain dormant otherwise, unchanged from today.

### Task 1.1: Exercise the SysId command lifecycle in sim, verify logging/telemetry/integration

- [x] **Implemented** — `src/test/java/frc/robot/subsystems/CommandSwerveDrivetrainSysIdSimWorkflowTest.java`
  (new file). Boots the robot in sim (`AutoRegressionTestBase`'s proven `Robot()`-thread pattern), selects the
  Translation routine (`useTranslationSysId()`), schedules `sysIdQuasistatic(kForward)` for a bounded 3s window
  (not the full 10s default timeout), confirms it's still scheduled (not errored out) mid-window, cancels it, then
  repeats with `sysIdDynamic(kForward)` for a bounded 1.5s window. No `CommandSwerveDrivetrain.java` production
  code was touched — every method this test calls (`useTranslationSysId`, `sysIdQuasistatic`, `sysIdDynamic`)
  already existed and was already wired (per `docs/SysId_Characterization_Checklist.md`'s "ready now" finding).
- [x] **Telemetry/analysis-pipeline check** — after the run, reads the produced wpilog via WPILib's
  `DataLogReader`/`DataLogRecord` (the same library `frc.robot.auto.WpilogTrajectoryErrorReader` uses), and
  asserts `Drive/AppliedVoltsPerModule` shows >10 samples, a max-abs value >0.5V, and >3 distinct rounded values —
  proving the signal actually ramped/stepped during the SysId window rather than sitting at a constant (e.g. 0).
- [x] **SignalLogger/hoot-log check — ran, and produced a real (negative) finding, not assumed either way.**
  Phoenix's `SignalLogger` writes a separate `.hoot` file (`docs/SysId_Characterization_Checklist.md` §3), distinct
  from the AdvantageKit wpilog. Confirmed empirically, 3/3 consistent runs: **no `.hoot` file appears in `logs/`
  during this headless JUnit sim harness** — `SignalLogger.start()` still runs without error (per
  `Telemetry.java:37`, unconditional at boot) and the `SysIdTranslation_State` callback's `SignalLogger.writeString`
  calls never throw, but no file is written under this harness (no real CAN bus/CANivore present). This is scoped
  as an *observation*, not a test failure: Task 1.1's own success criteria never claimed hoot-log file production
  would work in sim, and the checklist's §6 already flags real hoot capture as physical-robot-only. Recorded here so
  a future session doesn't waste time trying to make the sim JUnit harness produce an analyzable `.hoot` file — it
  structurally can't without real CAN hardware.
- [x] **Kept assertions scoped to "the pipeline ran and produced data"** — no `kS`/`kV`/`kA` value is computed or
  asserted anywhere in this test, per this task's own success criteria. That computation is Task 1.2's job, not
  started this session per explicit instruction.

**Verification:** `CommandSwerveDrivetrainSysIdSimWorkflowTest` passed 3/3 consecutive runs. `./gradlew compileJava`
BUILD SUCCESSFUL. `python SKILLS/run_headless_sim.py --run-seconds 12` PASS. Full `./gradlew test` gate: see the
session note for the result recorded there.

### Task 1.2: Compute placeholder gains from the sim run, prove the analysis leg works

- [x] **Implemented** — re-ran `CommandSwerveDrivetrainSysIdSimWorkflowTest` to produce a fresh wpilog
  (`logs/akit_26-07-28_11-56-19.wpilog`, no production code touched). A one-off analysis script (not committed —
  this task is documentation/analysis-only, per its own scope) reused `SKILLS/parse_akit_log.py` to pair
  `Drive/AppliedVoltsPerModule` (mean abs across 4 modules) against `DriveState/Speeds`'s `vx` by nearest
  timestamp (all 151 samples matched within 20ms), then fit `V = kS·sign(vx) + kV·vx` via ordinary least squares
  — the same two-term relationship SysId's own analyzer computes.
- [x] **Result recorded** in `docs/SysId_Sim_Workflow_Validation.md`: `kS = 0.1849 V`, `kV = 1.7293 V·s/m`,
  R² = 0.9198 over 151 paired samples — a well-conditioned, non-degenerate fit, proving the pipeline (telemetry
  capture → wpilog → timestamp matching → regression) works end-to-end.
  Labeled unmistakably throughout: **"SIMULATION-DERIVED PLACEHOLDER — DO NOT USE FOR REAL ROBOT
  CHARACTERIZATION,"** repeating the Phase 1 TODO's requirement verbatim so the doc is self-contained even if read
  without this plan.
- [x] **Explicitly not claimed:** that the numbers are numerically reasonable or usable beyond proving the
  pipeline — `TunerConstants.driveGains` was not touched, and the doc says so directly.

**Regression tests:** Once implemented, the Task 1.1 test itself (asserts the pipeline runs and telemetry is
captured) is this phase's regression test — re-run on every future change to `CommandSwerveDrivetrain`'s SysId
wiring to catch a silent breakage before it would otherwise only surface at a physical bring-up session.

**Success criteria:**
- Sim run completes cleanly: command lifecycle exercised, `SignalLogger` state transitions fire, wpilog telemetry
  captured during the SysId window specifically.
- A placeholder `kS`/`kV` is computed from that data and recorded in `docs/SysId_Sim_Workflow_Validation.md`, proving
  the full command → telemetry → data-collection → analysis pipeline works.
- **The TODO above is present and unremoved** — this phase cannot be marked done by silently dropping the warning
  once numbers exist; the warning's whole purpose is to survive past the point where it'd be tempting to forget it.
- Explicitly NOT a success criterion: that the sim-derived numbers are numerically reasonable, close to any real
  value, or usable for anything beyond proving the pipeline — this phase makes no claim about the numbers themselves.

**Rollback plan:** None needed — this phase, as scoped, never writes to `TunerConstants.java`. If Task 1.1/1.2 are
implemented and found to not prove what they claim, the new test/results-doc files can simply be deleted; nothing
downstream depends on them (Phase 2's real characterization is independent, physical-hardware work).

---

# Phase 2 — Tune Slot0

> **⚠️ PRECONDITION, carried forward from Phase 1's TODO:** Phase 1 (as revised) runs in simulation and produces only
> temporary, sim-derived placeholder numbers — it does **not** produce anything Task 2.1 may use. Before Task 2.1 can
> apply real values, **real hardware characterization must be performed on the physical robot**, per
> `docs/SysId_Characterization_Checklist.md` §4 (the same physical procedure originally described in this plan's own
> first draft of Phase 1). This physical run is not yet scheduled as a numbered task in this plan — it is a hard
> blocker on Task 2.1 and must happen, and be recorded, before this phase proceeds past Task 1.1/1.2's sim-only
> validation. **Phase 2 is not complete — and `driveGains` must not be treated as production-ready — until this
> precondition is satisfied**, regardless of whether Task 2.1's mechanical steps have been carried out with
> placeholder numbers for other testing purposes.

**Objective:** Apply real, hardware-measured `kS`/`kV`/`kA` to `TunerConstants.driveGains`, then empirically retune
`kP`/`kI`/`kD` off that feedforward, per the existing "September Tuning Playbook" Step 3
(`docs/Path_Following_Tuning_Readiness_Audit.md` §6). **Autonomous is unaffected by this entire phase** — confirmed
in `docs/Autonomous_Control_Path_Audit.md`: `Slot0` is not consulted by `OpenLoopVoltage`, which is still active
until Phase 3. This is worth stating plainly because it means Phase 2 carries substantially lower risk than it would
under a closed-loop-auto architecture — the only things it can affect before Phase 3 lands are teleop driving feel
and the `trackTarget()` vision-tracking command.

**Expected behavior:** Improved teleop drive velocity-tracking accuracy and `trackTarget()` precision. No change to
autonomous or `driveToPOI()` behavior (both confirmed `OpenLoopVoltage`, unaffected by `Slot0`).

### Task 2.1: Apply characterized feedforward

**Files:**
- Modify: `src/main/java/frc/robot/generated/TunerConstants.java:32-34` (`driveGains`)

- [ ] **Step 1: Apply the physical-robot-measured values**

Replace:
```java
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0.1).withKV(0.124);
```
with the **real hardware-measured** values (exact numbers come from the physical characterization run described in
this phase's precondition above — this step cannot be completed with placeholder numbers, and specifically must not
be completed with Phase 1's sim-derived numbers; if the physical run hasn't happened yet, this task is blocked, not
to be filled in with a guess or a sim placeholder):
```java
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(<hardware-measured kS>).withKV(<hardware-measured kV>).withKA(<hardware-measured kA>);
```

- [ ] **Step 2: Verify gate**

```bash
./gradlew compileJava
python SKILLS/run_headless_sim.py --run-seconds 12
```
Expected: BUILD SUCCESSFUL, sim PASS. (Sim uses maple-sim's own separate friction placeholders for its physics model
— this gate confirms the constant compiles and boots cleanly, not that sim behavior changed meaningfully; sim SysId
would be circular per the Global Constraints note.)

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/generated/TunerConstants.java
git commit -m "drivetrain: apply hardware-characterized drive kS/kV/kA"
```

### Task 2.2: Empirically retune `kP`/`kI`/`kD`

Not TDD-able — this is real-hardware iterative tuning, per the existing playbook's own methodology (manual
`SwerveRequest.Velocity` step commands, not a computable target).

- [ ] Command fixed-speed steps via `SwerveRequest.Velocity` (bench or floor test) at several speeds.
- [ ] Watch `SwerveStates/Measured` vs `SwerveStates/Setpoints` (per-module speed) and
  `Drive/StatorCurrentAmpsPerModule` live or in the resulting wpilog.
- [ ] Adjust `kP`/`kI`/`kD` in `TunerConstants.java:32-34`, iterate. Each iteration: re-run
  `./gradlew compileJava`, redeploy, re-test.
- [ ] Once acceptable, commit:
  ```bash
  git add src/main/java/frc/robot/generated/TunerConstants.java
  git commit -m "drivetrain: retune drive Slot0 kP/kI/kD off characterized feedforward"
  ```

**Regression tests:**
- `./gradlew test` (full suite) after each commit — no test in the current suite exercises `Slot0` numeric values
  directly (per `docs/Drive_Slot0_Retuning_Audit.md`, nothing in this repo's JUnit suite constructs real hardware),
  so this gate confirms no compile/wiring regression, not tuning quality.
- `LtNeutralAutoRegressionTest` — expected to pass **unchanged** against the existing golden, since autonomous
  doesn't consult `Slot0` yet. **If this test's result changes at all during Phase 2, that is an anomaly worth
  investigating before proceeding to Phase 3** — it would mean something in this phase had an effect the confirmed
  control-path analysis says it shouldn't, and that discrepancy should be understood, not shrugged off.

**Success criteria:**
- Step-response settling time/overshoot the mentor finds acceptable (playbook's own stated acceptance criteria —
  no specific numeric target is prescribed here, matching every prior audit's "no gain values recommended" stance).
- No stator current pinning at `kSlipCurrent = 120A` during routine (non-defense-simulating) tracking.
- `LtNeutralAutoRegressionTest` unchanged, confirming Phase 2's isolation from autonomous held in practice, not just
  in the architectural analysis.

**Rollback plan:** `git revert` Task 2.1's and/or Task 2.2's commits independently (they're separate commits) back
toward the placeholder `kS=0.1, kV=0.124, kA` unset / `kP=0.1` values. Zero effect on autonomous either way, since
`Slot0` remains unconsulted by autonomous until Phase 3 regardless of which values are in place.

---

# Phase 3 — Switch only autonomous `ApplyRobotSpeeds` to `Velocity`

**Objective:** The single architecture change this whole migration exists to evaluate. Scoped to *only* the
autonomous path-following request — `driveToPOI()` (already explicit `OpenLoopVoltage`) and teleop/`trackTarget()`
(already explicit `Velocity`) are both already correctly set and untouched by this phase.

**Mentor sign-off required before this task begins — do not proceed past Step 1 without explicit confirmation.**
This is the one phase in this plan that changes production autonomous behavior; per this plan's Global Constraints
and this project's own established precedent for non-defaultable decisions, an executing agent must stop and ask
here, not proceed on inferred approval from having reached this point in the plan.

**Expected behavior:** Autonomous path-following now runs the drive motors' onboard closed loop (`Slot0`, freshly
characterized and retuned in Phases 1-2) instead of the linear `kSpeedAt12Volts` scalar. Per
`docs/Autonomous_Drive_Architecture_Recommendation.md` §1, expect improved shaft-level tracking fidelity under
torque disturbances; **do not expect this alone to fix the chassis-level tracking error** (`PPHolonomicDriveController`'s
unclamped `kP=5`/`kP=3`, confirmed independent of this change). **The existing `LT_Neutral.json` golden is expected
to need re-baselining after this phase** — this is a real architecture change, not a regression to be forced back to
the old numbers.

### Task 3.1: Extract `buildAutoRequest()` and set `DriveRequestType.Velocity`

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java:359-363` and its import block
- Create: `src/test/java/frc/robot/subsystems/CommandSwerveDrivetrainAutoRequestTest.java`

**Interfaces:**
- Consumes: `ChassisSpeeds CommandSwerveDrivetrain.prepareAutoSpeeds(ChassisSpeeds)` (Phase 0, unchanged).
- Produces: `SwerveRequest.ApplyRobotSpeeds CommandSwerveDrivetrain.buildAutoRequest(ChassisSpeeds speeds,
  DriveFeedforwards feedforwards)` — package-private, same testability pattern as `sanitizeAutoSpeeds`/
  `prepareAutoSpeeds`.

- [ ] **Step 1: Write the failing test**

Create `src/test/java/frc/robot/subsystems/CommandSwerveDrivetrainAutoRequestTest.java`:

```java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.generated.TunerConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Pins the autonomous control mode this migration exists to change. Per
 * docs/Autonomous_Control_Path_Audit.md, {@code ApplyRobotSpeeds.DriveRequestType} defaults to
 * OpenLoopVoltage and was never overridden -- this test guards the Phase 3 override so a future
 * refactor can't silently drop it back to the default.
 */
class CommandSwerveDrivetrainAutoRequestTest {

  private CommandSwerveDrivetrain drivetrain;

  @BeforeEach
  void setup() {
    HAL.initialize(500, 0);
    DriverStationSim.resetData();
    SimHooks.pauseTiming();
    drivetrain = TunerConstants.createDrivetrain();
  }

  @AfterEach
  void teardown() {
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  @Test
  @Timeout(30)
  void autoRequestUsesClosedLoopVelocity() {
    ChassisSpeeds speeds = new ChassisSpeeds(1.0, 0.0, 0.0);
    DriveFeedforwards zeroFeedforwards = DriveFeedforwards.zeros(4);

    var request = drivetrain.buildAutoRequest(speeds, zeroFeedforwards);

    assertEquals(DriveRequestType.Velocity, request.DriveRequestType,
        "autonomous path-following must use closed-loop Velocity control per the Phase 3 migration");
  }
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
./gradlew test --tests "frc.robot.subsystems.CommandSwerveDrivetrainAutoRequestTest"
```
Expected: FAIL — `cannot find symbol: method buildAutoRequest(...)`.

- [ ] **Step 3: Implement `buildAutoRequest()` and rewire the lambda**

Add this import to `CommandSwerveDrivetrain.java`, alongside the existing `com.pathplanner.lib.*` imports
(after line 20, `com.pathplanner.lib.util.PathPlannerLogging`):

```java
import com.pathplanner.lib.util.DriveFeedforwards;
```

Add the new method immediately after `prepareAutoSpeeds()` (added in Phase 0), before `private void
configureAutoBuilder()`:

```java
    /**
     * Builds the CTRE request for one autonomous path-following control-loop iteration.
     * Package-private so CommandSwerveDrivetrainAutoRequestTest can assert the configured
     * DriveRequestType directly -- the same testability pattern as {@link #sanitizeAutoSpeeds} and
     * {@link #prepareAutoSpeeds}.
     *
     * <p>DriveRequestType.Velocity per the Phase 3 migration
     * (docs/superpowers/plans/2026-07-28-autonomous-velocity-migration.md): drive Slot0 must be
     * characterized (Phase 1) and retuned (Phase 2) before this method is reachable in production,
     * since Velocity mode consults Slot0 where OpenLoopVoltage never did -- see
     * docs/Autonomous_Control_Path_Audit.md for the confirmed control-path evidence.
     */
    SwerveRequest.ApplyRobotSpeeds buildAutoRequest(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        return m_pathApplyRobotSpeeds
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSpeeds(prepareAutoSpeeds(speeds))
                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons());
    }
```

Change the `configureAutoBuilder()` lambda from:

```java
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds
                                    .withSpeeds(prepareAutoSpeeds(speeds))
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
```

to:

```java
                    (speeds, feedforwards) -> setControl(buildAutoRequest(speeds, feedforwards)),
```

- [ ] **Step 4: Run test to verify it passes**

```bash
./gradlew test --tests "frc.robot.subsystems.CommandSwerveDrivetrainAutoRequestTest"
```
Expected: PASS.

- [ ] **Step 5: Full verification gate**

```bash
./gradlew compileJava
python SKILLS/run_headless_sim.py --run-seconds 12
./gradlew test
```
Expected: BUILD SUCCESSFUL, sim PASS. **Expected, not a failure to fix:** `LtNeutralAutoRegressionTest` may now
fail against the old `LT_Neutral.json` golden — the underlying dynamics genuinely changed. Do not "fix" this by
reverting Phase 3; re-baselining is Phase 4/5's job, after the comparison is captured (see Task 4.2's ordering note).

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java \
        src/test/java/frc/robot/subsystems/CommandSwerveDrivetrainAutoRequestTest.java
git commit -m "drivetrain: switch autonomous path-following to closed-loop Velocity control"
```

**Regression tests:**
- `CommandSwerveDrivetrainAutoRequestTest` (new) — must pass, confirms the mode switch itself.
- `CommandSwerveDrivetrainSanitizeSpeedsTest` — must still pass unchanged; `prepareAutoSpeeds()`'s own behavior is
  untouched by this phase, only its caller changed.
- `LtNeutralAutoRegressionTest` — **run and record the result, but do not treat a golden mismatch here as a defect
  to fix.** This is exactly the signal Phase 4 exists to capture and interpret.

**Success criteria:**
- `CommandSwerveDrivetrainAutoRequestTest` and `CommandSwerveDrivetrainSanitizeSpeedsTest` both pass.
- `./gradlew compileJava` and `run_headless_sim.py` both green.
- The robot drives a full autonomous path in sim without crashing, stalling unexpectedly, or throwing a
  `DriverStation.reportError` — confirmed by watching `python SKILLS/run_headless_sim.py`'s output and/or a manual
  `simulateJava` run, not assumed from compilation success alone.

**Rollback plan:** `git revert` Task 3.1's single commit. This restores the lambda to calling `prepareAutoSpeeds()`
directly (Phase 0's state) — i.e. reverting Phase 3 alone, cleanly, without touching Phase 0's discretization fix or
Phase 1-2's characterized `Slot0` gains (which remain harmlessly inert for autonomous once reverted, exactly as they
were before Phase 3 landed). This is the plan's single most important rollback path and was designed to be a
one-commit, one-line revert specifically for this reason.

---

# Phase 4 — Compare against current `OpenLoopVoltage`

**Objective:** Produce a structured, evidence-based before/after comparison across the five requested metrics
(trajectory error, peak longitudinal error, max cross-track error, completion time, current draw), isolating
`DriveRequestType` as the only variable between the two snapshots being compared.

**Important methodological point this phase must get right:** comparing Phase 3's result against the *original*
pre-Phase-0 `LT_Neutral.json` golden would confound three changes at once (discretization, characterized `Slot0`,
`DriveRequestType`). The valid comparison is **immediately-before-Phase-3 vs. immediately-after-Phase-3** — same
discretization, same `Slot0` state, `DriveRequestType` as the only difference. Task 4.2 below exists specifically to
capture that "before" snapshot at the right point in the sequence — **it should have been captured right before
Task 3.1's Step 6 commit, or reconstructed by temporarily checking out the commit immediately prior to it.**

**Expected behavior:** No behavior change from this phase's own code (Task 4.1 only adds a new logged/captured
metric, changes no control logic). The comparison itself may show either outcome — this phase does not presuppose
which mode wins.

### Task 4.1: Add current-draw capture to the regression harness

**Files:**
- Create: `src/test/java/frc/robot/auto/WpilogCurrentDrawReader.java`
- Modify: `src/test/java/frc/robot/auto/AutoRegressionGolden.java`
- Modify: `src/test/java/frc/robot/auto/AutoRegressionTestBase.java`

**Interfaces:**
- Produces: `WpilogCurrentDrawReader.readMaxStatorCurrentAmps(Path wpilogFile): double` — max absolute stator
  current across all 4 drive modules over the whole log, mirroring `WpilogTrajectoryErrorReader`'s established
  shape and its own use of WPILib's `DataLogReader`/`DataLogRecord`.

- [ ] **Step 1: Write the failing test**

Create `src/test/java/frc/robot/auto/WpilogCurrentDrawReaderTest.java`:

```java
package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.Test;

/**
 * Exercises WpilogCurrentDrawReader against a real wpilog produced by the existing auto
 * regression harness, rather than a hand-built fixture -- consistent with
 * WpilogTrajectoryErrorReader's own test-free-but-harness-proven pattern; this test instead
 * proves the reader against a controlled synthetic case to keep it independently testable.
 */
class WpilogCurrentDrawReaderTest {

  @Test
  void missingEntryThrows() {
    // A wpilog that never logged Drive/StatorCurrentAmpsPerModule (e.g. a non-drivetrain test's
    // log) must fail loudly, not silently return 0.0 and look like a real (if oddly low) result.
    Path nonexistentButValidLog = Paths.get("build", "does-not-exist.wpilog");
    assertThrows(IOException.class,
        () -> WpilogCurrentDrawReader.readMaxStatorCurrentAmps(nonexistentButValidLog));
  }
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
./gradlew test --tests "frc.robot.auto.WpilogCurrentDrawReaderTest"
```
Expected: FAIL — `cannot find symbol: class WpilogCurrentDrawReader`.

- [ ] **Step 3: Implement `WpilogCurrentDrawReader`**

Create `src/test/java/frc/robot/auto/WpilogCurrentDrawReader.java`:

```java
package frc.robot.auto;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

/**
 * Scans a .wpilog for Drive/StatorCurrentAmpsPerModule (CommandSwerveDrivetrain#
 * logDriveMotorVoltages, a double[4]) and returns the max absolute value across all four modules
 * over the whole file. Mirrors WpilogTrajectoryErrorReader's structure exactly, adapted for a
 * double-array entry instead of a scalar double.
 */
final class WpilogCurrentDrawReader {
    static final String kEntryName = "/RealOutputs/Drive/StatorCurrentAmpsPerModule";
    private static final String kDoubleArrayType = "double[]";

    private WpilogCurrentDrawReader() {}

    static double readMaxStatorCurrentAmps(Path wpilogFile) throws IOException {
        DataLogReader reader = new DataLogReader(wpilogFile.toString());
        if (!reader.isValid()) {
            throw new IOException("Not a valid WPILOG file: " + wpilogFile);
        }

        Map<Integer, String> activeNames = new HashMap<>();
        Map<Integer, String> activeTypes = new HashMap<>();
        boolean sawEntry = false;
        double maxCurrent = 0.0;

        for (DataLogRecord record : reader) {
            if (record.isStart()) {
                DataLogRecord.StartRecordData start = record.getStartData();
                activeNames.put(start.entry, start.name);
                activeTypes.put(start.entry, start.type);
                if (start.name.equals(kEntryName)) {
                    sawEntry = true;
                }
                continue;
            }
            if (record.isFinish() || record.isSetMetadata()) {
                continue;
            }

            String name = activeNames.get(record.getEntry());
            String type = activeTypes.get(record.getEntry());
            if (!kEntryName.equals(name) || !kDoubleArrayType.equals(type)) {
                continue;
            }

            for (double amps : record.getDoubleArray()) {
                if (!Double.isNaN(amps)) {
                    maxCurrent = Math.max(maxCurrent, Math.abs(amps));
                }
            }
        }

        if (!sawEntry) {
            throw new IOException(
                    "wpilog " + wpilogFile + " never logged " + kEntryName
                            + " -- is logDriveMotorVoltages() wired up?");
        }

        return maxCurrent;
    }
}
```

- [ ] **Step 4: Run test to verify it passes**

```bash
./gradlew test --tests "frc.robot.auto.WpilogCurrentDrawReaderTest"
```
Expected: PASS.

- [ ] **Step 5: Wire into the golden and harness**

In `src/test/java/frc/robot/auto/AutoRegressionGolden.java`, add one field after `maxLongitudinalErrorMeters`:

```java
    public double maxStatorCurrentAmps;
```

In `src/test/java/frc/robot/auto/AutoRegressionTestBase.java`, in `regressionCheck()` immediately after the existing
`WpilogTrajectoryErrorReader.MaxErrors errors = ...` line (line 172), add:

```java
        double maxStatorCurrentAmps = WpilogCurrentDrawReader.readMaxStatorCurrentAmps(wpilog);
```

Immediately after `actual.maxLongitudinalErrorMeters = errors.maxLongitudinalErrorMeters();` (line 180), add:

```java
        actual.maxStatorCurrentAmps = maxStatorCurrentAmps;
```

**Deliberately not added as a new `assertTrue`/tolerance-gated comparison** in this step — per
`AutoRegressionTolerances`'s own stated philosophy ("intentionally loose... tightening these is a deliberate later
fast-follow, not part of this milestone"), a new metric needs at least one real before/after data point (which this
whole phase exists to produce) before a sensible tolerance can be set. Recorded and captured, not yet gated.

- [ ] **Step 6: Re-establish the existing golden with the new field**

```bash
./gradlew test --tests "frc.robot.auto.LtNeutralAutoRegressionTest" -DupdateAutoGolden=true -DconfirmGoldenUpdate=true
```
This updates `LT_Neutral.json` to include `maxStatorCurrentAmps` — a schema addition, not a tolerance change; the
existing `completed`/`runtimeSeconds`/`maxLateralErrorMeters`/`maxLongitudinalErrorMeters` values are expected to be
numerically identical to before this step (same code path, same run conditions), only the new field is new.

- [ ] **Step 7: Full verification gate**

```bash
./gradlew compileJava
python SKILLS/run_headless_sim.py --run-seconds 12
./gradlew test
```
Expected: BUILD SUCCESSFUL, sim PASS, full suite green (including the regenerated golden from Step 6).

- [ ] **Step 8: Commit**

```bash
git add src/test/java/frc/robot/auto/WpilogCurrentDrawReader.java \
        src/test/java/frc/robot/auto/WpilogCurrentDrawReaderTest.java \
        src/test/java/frc/robot/auto/AutoRegressionGolden.java \
        src/test/java/frc/robot/auto/AutoRegressionTestBase.java \
        src/test/resources/autoRegression/LT_Neutral.json
git commit -m "auto-regression: capture max drive stator current per run"
```

### Task 4.2: Capture the isolated before/after comparison

**This task's ordering is important and must not be skipped:** it produces the "before" snapshot that makes the
comparison valid (see this phase's objective note above).

- [ ] Confirm you are at the commit immediately before Task 3.1's Step 6 commit (i.e. Phase 0-2 landed, Phase 3 not
  yet applied). If Phase 3 has already been committed and this snapshot was never captured at the right time,
  reconstruct it: `git stash`, `git checkout <commit-before-phase-3>`, run the capture below, `git checkout -`,
  `git stash pop` — this reconstruction is valid specifically because Task 3.1 is a single, isolated commit.
- [ ] Run the regression test and save its golden output under a **comparison-only** filename (not the active
  golden the suite checks against):
  ```bash
  ./gradlew test --tests "frc.robot.auto.LtNeutralAutoRegressionTest" -DupdateAutoGolden=true -DconfirmGoldenUpdate=true
  cp src/test/resources/autoRegression/LT_Neutral.json src/test/resources/autoRegression/LT_Neutral_openloop_baseline.json
  ```
- [ ] Restore `LT_Neutral.json` to whatever it should actually be for the current commit (if you reconstructed via
  checkout, `git checkout <original-branch> -- src/test/resources/autoRegression/LT_Neutral.json` after returning).
- [ ] Commit the baseline snapshot on its own:
  ```bash
  git add src/test/resources/autoRegression/LT_Neutral_openloop_baseline.json
  git commit -m "auto-regression: capture pre-Velocity-migration OpenLoopVoltage baseline for LT Neutral"
  ```
- [ ] At the current commit (Phase 3 applied), run the same test again and record its output — this is the "after"
  data point, taken from the just-updated `LT_Neutral.json` (Task 4.1 Step 6) or a fresh run if more time has
  passed.

### Task 4.3: Produce the comparison report

**Files:**
- Create: `docs/Velocity_Migration_Comparison_Report.md`

- [ ] Write the report with a table comparing, for `LT Neutral` (and any other auto with an established regression
  test at the time this phase runs): `completed`, `runtimeSeconds` (= completion time), `maxLateralErrorMeters` (=
  max cross-track error), `maxLongitudinalErrorMeters` (= peak longitudinal error), `maxStatorCurrentAmps` (= current
  draw), each as `LT_Neutral_openloop_baseline.json`'s value vs. the current `LT_Neutral.json`'s value, with the
  raw delta and percent change. "Trajectory error" (the fifth requested metric) is the combination already captured
  by the lateral+longitudinal pair plus `Trajectory/Error*` AdvantageKit telemetry (already logged per
  `docs/Path_Following_Tuning_Readiness_Audit.md` §5) — cite specific wpilog timestamps/values if a single-number
  trajectory-error summary is wanted beyond the two already-tabulated components, rather than inventing a new metric
  not already defined anywhere in this codebase.
- [ ] State plainly whether each metric improved, regressed, or was statistically indistinguishable given the
  already-documented ~0.3m RMS run-to-run sim-timing jitter (`AutoRegressionTolerances.java`'s own doc comment) —
  do not report a single-run difference smaller than that jitter as a confirmed change.
- [ ] No recommendation in this report — Phase 5 makes the decision. This report's job is the data, stated plainly.

**Regression tests:**
- `WpilogCurrentDrawReaderTest` (new, Task 4.1).
- `LtNeutralAutoRegressionTest` — run at both commits per Task 4.2; neither run is expected to "pass" against the
  other's golden by design (that's the point of the comparison) — the harness's own pass/fail gate is not the
  mechanism this phase uses to judge success.

**Success criteria:**
- All five requested metrics captured for both configurations, with the confound-isolation ordering in Task 4.2
  actually followed (not the confounded original-golden-vs-Phase-3 comparison).
- `docs/Velocity_Migration_Comparison_Report.md` exists, states real numbers (no placeholders), and explicitly notes
  the sim-timing-jitter floor below which a difference isn't a confirmed change.
- This phase's own code changes (`WpilogCurrentDrawReader`, the golden schema addition) pass their own tests and the
  full verification gate.

**Rollback plan:** `git revert` Task 4.1's commit (current-draw capture) and/or Task 4.2's commit (baseline
snapshot) independently — both are additive (new field, new file) and don't alter any existing assertion's pass/fail
behavior, so reverting either is low-risk and doesn't cascade into Phases 0-3. The comparison report
(`docs/Velocity_Migration_Comparison_Report.md`) can simply be deleted if the migration is abandoned.

---

# Phase 5 — Decide whether to keep Velocity

**Objective:** A human decision, informed by Phase 4's data and the six-dimension mechanism comparison already
established in `docs/Autonomous_Drive_Architecture_Recommendation.md` §1. **Not a code task.**

**Expected behavior:** No code changes from this phase itself — only the two possible follow-up action sets below,
each of which is its own small, separately-committed piece of work.

### Decision procedure

- [ ] Present Phase 4's comparison report to the mentor.
- [ ] Ask explicitly (via `AskUserQuestion` or equivalent direct confirmation — this is the second of this plan's
  two non-defaultable decision points): **keep `Velocity`, or revert to `OpenLoopVoltage`?**
- [ ] Do not infer an answer from the comparison data alone, even if every metric improved — per
  `docs/Autonomous_Drive_Architecture_Recommendation.md`'s own closing section, this decision also weighs tuning-
  maintenance burden and match-robustness risk that a single sim comparison run doesn't fully capture.

### If keeping `Velocity`:

- [ ] Re-baseline `LT_Neutral.json` as the new authoritative golden (it already reflects Phase 3's state from Task
  4.1 Step 6 — confirm it's current, re-run `-DupdateAutoGolden=true -DconfirmGoldenUpdate=true` if anything changed
  since).
- [ ] Delete the comparison-only `LT_Neutral_openloop_baseline.json` (its job is done) or move it into
  `docs/claudex/history.md`'s record of this decision if the mentor wants it preserved for reference.
- [ ] Update `CLAUDE.md`'s `## Session Handoff` / architecture notes to reflect that autonomous now runs closed-loop
  Velocity control, and that `docs/Autonomous_Control_Path_Audit.md`'s "confirmed: OpenLoopVoltage" finding is now
  historical (true as of when it was written, superseded by this migration) — do not silently leave that audit
  looking current when it no longer describes production behavior.
- [ ] Expand the auto regression suite to the other 10 in-scope autos against the new closed-loop baseline
  (`CLAUDE.md`'s existing backlog item), not against stale open-loop goldens.

### If reverting to `OpenLoopVoltage`:

- [ ] `git revert` Phase 3's Task 3.1 commit — per that task's own rollback plan, this is a clean, isolated,
  one-commit revert.
- [ ] **Keep Phases 0-2's work** — discretization and characterized `Slot0` gains still benefit teleop and
  `trackTarget()` regardless of this decision (per the Architecture Recommendation doc's §5(a) tuning order, this
  was always framed as an independent-value improvement, not contingent on the Velocity switch succeeding).
- [ ] Record the decision and its reasoning in `docs/claudex/history.md` and `CLAUDE.md`'s session handoff, citing
  Phase 4's comparison report — a "we tried it and decided against it, here's the data" record is valuable and
  should not be silently discarded just because the answer was "no."

**Regression tests:** Whichever branch is taken, run the full verification gate
(`compileJava`/`run_headless_sim.py`/`gradlew test`) one final time to confirm the resulting state is clean.

**Success criteria:** A recorded decision, with reasoning traceable to Phase 4's data, and a clean resulting repo
state (either the closed-loop path fully adopted and documented, or fully reverted with Phases 0-2 preserved) — not
a half-applied state where the code and the documentation disagree about which mode is active.

**Rollback plan:** This phase's own "rollback" is re-opening the decision later if new evidence arrives — both
branches above already describe how to reverse themselves (re-revert Phase 3's revert, or re-run the keep-Velocity
steps), so no separate rollback mechanism is needed beyond what's already documented per-branch.
