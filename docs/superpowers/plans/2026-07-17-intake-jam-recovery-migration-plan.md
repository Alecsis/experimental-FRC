# Intake Jam-Detection Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move jam detection/recovery from the one-off `Intake.intakeJamReverse()` Command into `Intake.setRoller()` itself (mechanism-level, 1678-style), so any caller gets automatic protection, then route `RobotContainer`'s `"Intake Start Sequence"`/`"Intake Stop"` NamedCommands through `Superstructure` — closing the Backlog item that's been blocked since the Superstructure v2 refactor.

**Architecture:** `setRoller()` becomes the single interception point for jam handling: an `INTAKE` request while jammed substitutes a brief `EJECT` recovery pulse before resuming; a `STOP`/`EJECT` request always aborts any in-progress pulse immediately. `Superstructure.periodic()` already calls `setRoller(Roller.INTAKE)` every tick during `INTAKING`, so this makes recovery automatic there with zero changes to `Superstructure.java`. Full design and rationale: `docs/superpowers/specs/2026-07-17-intake-jam-recovery-migration-design.md`.

**Tech Stack:** WPILib 2026.2.1 (JDK 17 toolchain), AdvantageKit 26.0.2, JUnit 5, Gradle 8.11.

## Global Constraints

- `JAVA_HOME` must point at the WPILib JDK 17 (`/c/Users/Public/wpilib/2026/jdk`) before any `./gradlew` invocation.
- No vendor hardware imports — this plan only touches `Intake.java`, `RobotContainer.java`, and a new test file, none of which are `*IOReal.java` files.
- One `@Test` method per test class in `src/test/java` — `build.gradle`'s `forkEvery = 1` forks a fresh JVM per class, not per method, and `Intake` is a Singleton with no reset hook (same constraint `RobotLifecycleTest`/`SuperstructureEjectingTest` already document).
- The new test class must live in package `frc.robot.subsystems.intake` (same package as `Intake`) to get package-private access to the test-only jam-override hook added in Task 1 — it does not need the full `frc.robot` package the other two tests use, but it does still need to boot the real `Robot` (see Task 2's Interfaces note on why).
- Final claim of completion requires: `./gradlew compileJava` exit 0, then `./gradlew test` exit 0 with all three test classes (`RobotLifecycleTest`, `SuperstructureEjectingTest`, `IntakeJamRecoveryTest`) reported PASSED, per `CLAUDE.md`'s Advanced Agent Verification Loop (gates 1 and 2.5). Run `python SKILLS/run_headless_sim.py` (gate 2) after Task 3, since that task changes `RobotContainer` construction.
- Keep every task's commit in a compilable state — this drives the task ordering below (the NamedCommands fix must land before `intakeJamReverse()` is deleted, since `RobotContainer.java` references it directly).

---

### Task 1: Mechanism-level jam recovery + test hook (`Intake.java`)

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/intake/Intake.java`

**Interfaces:**
- Consumes: nothing new.
- Produces:
  - `Intake.setRoller(Roller)` — same signature, new jam-interception behavior. `Intake.getInstance()`, `Superstructure.periodic()`, and everything else that already calls it needs no changes.
  - `Intake.forceJamConditionForTest(double statorCurrentAmps, double velocityRadsPerSec): void` — package-private, consumed by Task 2's test.
  - `Intake.clearJamOverrideForTest(): void` — package-private, consumed by Task 2's test.

- [x] **Step 1: Add the `Timer` import and jam-recovery/test-override fields**

In `src/main/java/frc/robot/subsystems/intake/Intake.java`, add to the import block (after the existing `edu.wpi.first.units.measure.Angle` import, alphabetically before `smartdashboard`):

```java
import edu.wpi.first.wpilibj.Timer;
```

Add a new constant next to the existing jam-threshold constants (around line 24-27):

```java
  private static final double kJamStatorCurrentAmps = 80;
  private static final double kJamVelocityThresholdRadPerSec = RotationsPerSecond.of(5).in(RadiansPerSecond);
  private static final double kJamRecoveryPulseSeconds = 0.3;
  private static final double kHardstopStatorCurrentAmps = 65;
  private static final double kHomingVoltage = 3.0;
```

Add new fields next to `currentRoller` (around line 45):

```java
  private Roller currentRoller = Roller.STOP;
  private boolean jamRecoveryActive = false;
  private double jamRecoveryStartTimestamp = 0.0;

  private boolean testJamOverrideActive = false;
  private double testJamStatorCurrentAmps;
  private double testJamVelocityRadsPerSec;
```

- [x] **Step 2: Rewrite `setRoller()` to intercept jams, add `applyRoller()`**

Replace the existing `setRoller()` (currently lines 103-111):

```java
  public void setRoller(Roller state) {
    if (state == Roller.STOP) {
      io.setRollerVoltage(0);
    } else {
      io.setRollerVelocity(RotationsPerSecond.of(state.getRPM() / 60.0).in(RadiansPerSecond));
    }
    currentRoller = state;
    Logger.recordOutput("Intake/TargetState", state);
  }
```

with:

```java
  public void setRoller(Roller state) {
    if (state != Roller.INTAKE) {
      // STOP or EJECT always wins immediately, aborting any in-progress recovery pulse.
      jamRecoveryActive = false;
      applyRoller(state);
      return;
    }

    if (!jamRecoveryActive && isJammed()) {
      jamRecoveryActive = true;
      jamRecoveryStartTimestamp = Timer.getFPGATimestamp();
    }

    if (jamRecoveryActive) {
      if (Timer.getFPGATimestamp() - jamRecoveryStartTimestamp < kJamRecoveryPulseSeconds) {
        applyRoller(Roller.EJECT); // recovery pulse in progress
        return;
      }
      jamRecoveryActive = false; // pulse elapsed, fall through to resume INTAKE
    }

    applyRoller(Roller.INTAKE);
  }

  private void applyRoller(Roller state) {
    if (state == Roller.STOP) {
      io.setRollerVoltage(0);
    } else {
      io.setRollerVelocity(RotationsPerSecond.of(state.getRPM() / 60.0).in(RadiansPerSecond));
    }
    currentRoller = state;
    Logger.recordOutput("Intake/TargetState", state);
    Logger.recordOutput("Intake/JamRecoveryActive", jamRecoveryActive);
  }
```

- [x] **Step 3: Add the test-only jam-override hook**

Add these two methods directly below `isJammed()` (currently lines 118-121):

```java
  /** Test-only: forces isJammed()'s inputs, bypassing physics IntakeIOSim can't model (no
   *  load/obstruction). Package-private -- only for JUnit tests in this package. */
  void forceJamConditionForTest(double statorCurrentAmps, double velocityRadsPerSec) {
    testJamOverrideActive = true;
    testJamStatorCurrentAmps = statorCurrentAmps;
    testJamVelocityRadsPerSec = velocityRadsPerSec;
  }

  /** Test-only: stops overriding sensor readings, resumes real IntakeIOSim physics. */
  void clearJamOverrideForTest() {
    testJamOverrideActive = false;
  }
```

- [x] **Step 4: Apply the test override inside `periodic()`**

Replace the start of `periodic()` (currently lines 224-227):

```java
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
```

with:

```java
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    if (testJamOverrideActive) {
      inputs.rollerStatorCurrentAmps = testJamStatorCurrentAmps;
      inputs.rollerVelocityRadsPerSec = testJamVelocityRadsPerSec;
    }
    Logger.processInputs("Intake", inputs);
```

- [x] **Step 5: Static gate — compile clean**

Run:
```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
cd "C:/Users/xdm/Claude/experimental-FRC" && ./gradlew compileJava
```
Expected: `BUILD SUCCESSFUL`. (`intakeJamReverse()` and `intake()` still exist and still compile at this point — they're deleted in Task 4, after `RobotContainer.java` stops referencing them in Task 3.)

- [x] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/intake/Intake.java
git commit -m "intake: move jam recovery into setRoller(), add test-only override hook"
```

---

### Task 2: `IntakeJamRecoveryTest` — the behavior proof

**Files:**
- Create: `src/test/java/frc/robot/subsystems/intake/IntakeJamRecoveryTest.java`

**Interfaces:**
- Consumes: `Intake.getInstance()`, `Intake.setRoller(Roller)`, `Intake.getRollerState()`, `Intake.forceJamConditionForTest(double, double)`, `Intake.clearJamOverrideForTest()` (all from Task 1).
- Produces: nothing consumed elsewhere — terminal proof for this migration's jam-recovery mechanism.

This test still boots the real `Robot` via `startCompetition()`, even though it calls `Intake.setRoller()` directly rather than going through `Superstructure`/`CommandScheduler`, because `IntakeIOSim.updateInputs()` (`IntakeIOSim.java:111`) reaches into the static field `RobotContainer.drivetrain`, which is only populated once `RobotContainer` — and therefore the full `Robot` — has been constructed. Booting the real robot also ensures `Intake.periodic()` actually runs each tick (via `CommandScheduler.run()`'s subsystem loop), which is what applies the test-override values to `inputs` before `isJammed()`/`setRoller()` reads them from the test thread.

- [x] **Step 1: Write the test**

**Revised 2026-07-17 during Task 2's implementation** — the original version of this step called `Intake.setRoller()` directly, deliberately bypassing `Superstructure`. That failed deterministically: `Superstructure` is constructed as part of the required `Robot` boot regardless, sits in its default `OFF` state for the test's whole duration (nothing ever moves it), and `Superstructure.periodic()`'s `OFF` case unconditionally calls `intake.setRoller(Roller.STOP)` every tick — resetting `jamRecoveryActive` between the test's own calls. The design doc's Context finding #2 named this exact race but assumed Task 3 closes it; Task 3 only makes `Superstructure` the sole writer *during `INTAKING`*, which this test never entered. Fixed by routing through `Superstructure.requestIntake()`/`requestStow()` instead, matching how this logic is actually driven in production and turning every assertion into a stable state rather than a race against Superstructure's next tick.

```java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Robot;
import frc.robot.subsystems.superstructure.Superstructure;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Third test class in this repo, deliberately in package frc.robot.subsystems.intake so it gets
 * package-private access to Intake's forceJamConditionForTest()/clearJamOverrideForTest() test
 * hooks. Same real-startCompetition()-loop harness and single-@Test-method-per-class reasoning as
 * RobotLifecycleTest/SuperstructureEjectingTest.
 *
 * Drives INTAKING through Superstructure rather than calling Intake.setRoller() directly:
 * Superstructure is registered and its periodic() runs every tick regardless (constructed as
 * part of the required Robot boot), and its OFF/STOWED cases unconditionally call
 * intake.setRoller(Roller.STOP) -- a direct-call test that never requests INTAKING would have
 * that call fought/overwritten by Superstructure on every intervening tick. Routing through
 * Superstructure.requestIntake()/requestStow() instead matches the only way this logic is
 * actually driven in production and avoids the race entirely.
 */
class IntakeJamRecoveryTest {
  private Robot robot;
  private Thread robotThread;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    robot = new Robot();
    robotThread = new Thread(robot::startCompetition, "IntakeJamRecoveryTest-competition");
    robotThread.setDaemon(true);
    robotThread.start();
    SimHooks.waitForProgramStart();
  }

  @AfterEach
  void teardown() throws InterruptedException {
    try {
      robot.endCompetition();
      robotThread.join(1000);
      assertFalse(robotThread.isAlive(),
          "startCompetition() loop should have exited within 1s of endCompetition()");
      robot.close();
    } finally {
      SimHooks.resumeTiming();
    }
  }

  @Test
  @Timeout(30)
  void jamTriggersRecoveryPulseThenResumesAndAbortsOnStop() {
    assertTrue(robotThread.isAlive(),
        "startCompetition() loop should still be running after waitForProgramStart()");

    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.1);

    Intake intake = Intake.getInstance();
    Superstructure superstructure = Superstructure.getInstance(null);

    superstructure.requestIntake();
    SimHooks.stepTiming(0.1);

    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "INTAKING with no jam should just command the roller INTAKE");

    // Force a jam (high stator current, near-zero velocity -- isJammed()'s real condition).
    // Superstructure.periodic() re-calls setRoller(Roller.INTAKE) every tick during INTAKING,
    // so the recovery pulse fires automatically, the same way it will in production.
    intake.forceJamConditionForTest(100.0, 0.0);
    SimHooks.stepTiming(0.1);

    assertEquals(Intake.Roller.EJECT, intake.getRollerState(),
        "a jam detected during INTAKING should trigger an immediate recovery pulse");

    // Clear the jam (simulates the obstruction clearing during the pulse) and step past the
    // pulse duration (0.3s) -- this is now a stable end state, since nothing will re-trigger a
    // pulse once isJammed() reads false again.
    intake.clearJamOverrideForTest();
    SimHooks.stepTiming(0.4);

    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
        "after the pulse elapses and the jam clears, INTAKE should resume and stay resumed");

    // Force a second jam, confirm a new pulse fires, then abort mid-pulse via a real caller-level
    // stow request (matches a driver releasing the intake button / auto moving on).
    intake.forceJamConditionForTest(100.0, 0.0);
    SimHooks.stepTiming(0.1);
    assertEquals(Intake.Roller.EJECT, intake.getRollerState(),
        "second jam should trigger a new recovery pulse");

    superstructure.requestStow();
    SimHooks.stepTiming(0.1);

    assertEquals(Intake.Roller.STOP, intake.getRollerState(),
        "requestStow() mid-pulse must abort the recovery pulse immediately, not let it finish");

    intake.clearJamOverrideForTest();
  }
}
```

**Known risk to verify empirically, not by further reasoning:** `IntakeIOSim`'s roller uses closed-loop velocity control over a real `DCMotorSim`. A normal 0→800 RPM spin-up briefly produces high current at low velocity — the same electrical signature `isJammed()` looks for. If the very first assertion (baseline INTAKE, no forced jam) is flaky or fails because the roller hasn't cleared the 5 RPS threshold yet when checked, that's a real spin-up transient, not a broken test — increase the `SimHooks.stepTiming(0.1)` right after `requestIntake()` (more settle time before checking) rather than changing the assertion itself.

- [x] **Step 2: Run the test and verify it passes**

Run:
```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
cd "C:/Users/xdm/Claude/experimental-FRC" && ./gradlew test --tests "frc.robot.subsystems.intake.IntakeJamRecoveryTest"
```
Expected: `BUILD SUCCESSFUL`, 1 test PASSED.

- [x] **Step 3: Negative control — prove the assertions actually discriminate**

Temporarily change the first assertion's expected value, e.g.:
```java
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
```
(where it currently expects `Intake.Roller.EJECT`). Run the same command from Step 2 again. Expected: FAILS with a real `AssertionFailedError: ... expected: <INTAKE> but was: <EJECT>`. Revert the change back to `Intake.Roller.EJECT` and re-run to confirm green again.

- [x] **Step 4: Commit**

```bash
git add src/test/java/frc/robot/subsystems/intake/IntakeJamRecoveryTest.java
git commit -m "intake: add jam-recovery behavior proof"
```

---

### Task 3: NamedCommands fix (`RobotContainer.java`)

**Files:**
- Modify: `src/main/java/frc/robot/RobotContainer.java`

**Interfaces:**
- Consumes: `Superstructure.intakeCmd(): Command`, `Superstructure.stowCmd(): Command` — already exist (`Superstructure.java:140-142,150-152`).
- Produces: nothing consumed elsewhere.

- [x] **Step 1: Remove the now-unused `Agitate` import**

In `src/main/java/frc/robot/RobotContainer.java`, remove line 37:

```java
import frc.robot.subsystems.shooter.Shooter.Agitate;
```

(Confirmed unused elsewhere in this file after Step 2 below — `Agitate` only appeared at the line being replaced.)

- [x] **Step 2: Replace the NamedCommand registrations**

Replace (currently lines 77-88):

```java
                // Deliberately NOT superstructure.intakeCmd() -- INTAKING drives the roller directly with
                // no jam handling, and jam-reverse protection hasn't been migrated into Intake or the state
                // machine yet. Swapping this registration would silently drop stall protection from autos.
                // Don't move this to the Superstructure until jam handling has a new home.
                NamedCommands.registerCommand(
                                "Intake Start Sequence", Commands.parallel(
                                                intake.intakeJamReverse(),
                                                Commands.runOnce(() -> shooter.setAgitator(Agitate.IN))));
                // Paired with "Intake Start Sequence" above -- kept at the same mechanism-level as its
                // start command for the same reason (not yet safe to route through the state machine).
                NamedCommands.registerCommand(
                                "Intake Stop", intake.stopRoller());
```

with:

```java
                // Jam recovery now lives inside Intake.setRoller() itself (mechanism-level), so
                // Superstructure.INTAKING gets it automatically -- routing through the state machine no
                // longer drops stall protection from autos.
                NamedCommands.registerCommand(
                                "Intake Start Sequence", superstructure.intakeCmd());
                NamedCommands.registerCommand(
                                "Intake Stop", superstructure.stowCmd());
```

- [x] **Step 3: Static gate — compile clean**

Run:
```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
cd "C:/Users/xdm/Claude/experimental-FRC" && ./gradlew compileJava
```
Expected: `BUILD SUCCESSFUL`. (`intake.intakeJamReverse()` still exists in `Intake.java` at this point, just no longer referenced here — no error, only an unused-method situation resolved in Task 4.)

- [x] **Step 4: Behavior gate — full test suite**

Run:
```bash
./gradlew test
```
Expected: `BUILD SUCCESSFUL`, `RobotLifecycleTest`, `SuperstructureEjectingTest`, and `IntakeJamRecoveryTest` all PASSED. (This confirms the `RobotContainer` construction change didn't break anything the other two tests exercise.)

- [x] **Step 5: Runtime launch gate**

Run:
```bash
python SKILLS/run_headless_sim.py
```
Expected: PASS, exit 0. Required because this task changes `RobotContainer`'s NamedCommands construction, per `CLAUDE.md`'s Verification Loop gate 2.

- [x] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/RobotContainer.java
git commit -m "robotcontainer: route Intake Start Sequence/Stop through Superstructure"
```

---

### Task 4: Cleanup — delete `intakeJamReverse()` and `Intake.intake()`

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/intake/Intake.java`

**Interfaces:**
- Consumes: nothing (deletion only).
- Produces: nothing.

Both methods are dead code as of Task 3: `intakeJamReverse()`'s only caller was `RobotContainer.java` (removed in Task 3) and `Intake.intake()` itself (also deleted here); `Intake.intake()` has no callers anywhere in the codebase.

- [x] **Step 1: Delete `intakeJamReverse()`**

Remove from `src/main/java/frc/robot/subsystems/intake/Intake.java` (the method currently sits between `isJammed()` and `rollerSysIdQuasistatic()`, and by this point also between the new test-hook methods added in Task 1 and `rollerSysIdQuasistatic()`):

```java
  public Command intakeJamReverse() {
    return Commands.sequence(
        Commands.runOnce(() -> setRoller(Roller.INTAKE)),
        Commands.waitUntil(this::isJammed),
        Commands.runOnce(() -> setRoller(Roller.EJECT)),
        Commands.waitSeconds(0.3),
        Commands.runOnce(() -> setRoller(Roller.INTAKE))).repeatedly().finallyDo(() -> setRoller(Roller.STOP));
  }
```

- [x] **Step 2: Delete `Intake.intake()`**

Remove:

```java
  public Command intake() {
    return Commands.either(
        Commands.sequence(
            Commands.either(
                Commands.sequence(
                    Commands.runOnce(() -> {
                      goTo(PivotState.DOWN);
                      deploy = true;
                    }, this),
                    Commands.waitSeconds(0.5)),
                Commands.none(),
                () -> !deploy),
            intakeJamReverse()),
        intakeJamReverse(),
        () -> homed);
  }
```

- [x] **Step 3: Static gate — compile clean**

Run:
```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
cd "C:/Users/xdm/Claude/experimental-FRC" && ./gradlew compileJava
```
Expected: `BUILD SUCCESSFUL`.

- [x] **Step 4: Full verification loop**

Run:
```bash
./gradlew test
python SKILLS/run_headless_sim.py
```
Expected: `./gradlew test` → `BUILD SUCCESSFUL`, all three test classes PASSED. `run_headless_sim.py` → PASS, exit 0.

- [x] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems/intake/Intake.java
git commit -m "intake: delete intakeJamReverse() and dead intake(), superseded by setRoller()"
```

---

## Final Verification (run after Task 4)

Per `CLAUDE.md`'s Advanced Agent Verification Loop, before claiming this migration complete:

1. **Static gate:** `./gradlew compileJava` → `BUILD SUCCESSFUL`.
2. **Runtime launch gate:** `python SKILLS/run_headless_sim.py` → PASS, exit 0.
3. **Behavior gate:** `./gradlew test` → `BUILD SUCCESSFUL`, all three test classes PASSED (`RobotLifecycleTest`, `SuperstructureEjectingTest`, `IntakeJamRecoveryTest`).
4. Update `CLAUDE.md`'s Current Task State: remove the "Decide the NamedCommands migration" Next-up bullet (closed). Append the full change to `docs/claudex/history.md`, citing file paths, the two findings from the design's Context section (agitator direction bug, the pre-existing "two writers, same tick" auto-intake bug), and the verification evidence above. If this reveals anything about `docs/claudex/architecture.md`'s "known, accepted gap" note (Rule 3, scheduler requirements), update that file too — this migration is arguably the "first observed two-writers-same-tick conflict" that note said would be the trigger to revisit.
