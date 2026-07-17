# Phase 3 Test B (EJECTING) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prove, with a real JUnit test driving the actual `Robot.startCompetition()` loop, that `Superstructure.ejectCmd()` drives the EJECTING state (intake roller reversed, indexer reversed, agitator out) and that cancelling it stows the mechanisms (roller stopped) — the same rigor `RobotLifecycleTest` already applied to the IMUMode 1→4 transition (Phase 3 Test A).

**Architecture:** Add live-state getters to `Intake`/`Shooter` mirroring the `Vision.currentIMUMode`/`getIMUMode()` pattern already in the codebase, then add a second JUnit test class (`SuperstructureEjectingTest`) that boots the real robot loop via `DriverStationSim`/`SimHooks`, schedules `superstructure.ejectCmd()` through `CommandScheduler`, and asserts the getters. A second test class is required rather than a second `@Test` method, because `RobotLifecycleTest`'s existing Javadoc and `build.gradle`'s `forkEvery = 1` comment both establish that Gradle forks a fresh JVM per test **class** (not per method), and `Vision`/`Intake`/`Shooter`/`Superstructure` are process-wide Singletons with no reset hook — reusing a JVM across two `@Test` methods would let the second method observe a `Superstructure` still wired to the first method's `Robot`/drivetrain.

**Tech Stack:** WPILib 2026.2.1 (JDK 17 toolchain), AdvantageKit 26.0.2, JUnit 5 (`org.junit.jupiter`), Gradle 8.11.

## Global Constraints

- `JAVA_HOME` must point at the WPILib JDK 17 (`/c/Users/Public/wpilib/2026/jdk`) before any `./gradlew` invocation — PATH `java` is JDK 25 and Gradle 8.11 cannot configure under it.
- No vendor hardware imports (`com.ctre.*`, REVLib, etc.) — this plan only touches `Intake.java`, `Shooter.java`, `src/test/java/frc/robot/*.java`, and `build.gradle`, none of which are `*IOReal.java` files.
- One `@Test` method per test class in this package — `forkEvery = 1` forks per class, and the subsystem Singletons have no reset hook (see Architecture above).
- Every test class that boots `Robot` must drive the real `startCompetition()` loop (background daemon thread + `DriverStationSim`/`SimHooks`), never call `disabledInit()`/`autonomousInit()`/etc. directly — AdvantageKit's `Logger.periodicBeforeUser()`/`periodicAfterUser()` (which flush `recordOutput` and back the getters this plan adds) only run inside that loop.
- Final claim of completion requires: `./gradlew compileJava` exit 0, then `./gradlew test` exit 0 with both `RobotLifecycleTest` and `SuperstructureEjectingTest` reported PASSED, per this repo's `CLAUDE.md` Advanced Agent Verification Loop (gates 1 and 2.5).

---

### Task 1: Live-state getters on `Intake` and `Shooter`

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/intake/Intake.java`
- Modify: `src/main/java/frc/robot/subsystems/shooter/Shooter.java`

**Interfaces:**
- Consumes: nothing new — mirrors the existing `Vision.currentIMUMode` field / `getIMUMode()` getter pattern at `src/main/java/frc/robot/subsystems/vision/Vision.java:68,150-152`.
- Produces:
  - `Intake.getRollerState(): Intake.Roller` — Task 2 and later callers read this.
  - `Shooter.getIndexerState(): Shooter.indexing` — Task 2 and later callers read this.
  - `Shooter.getAgitateState(): Shooter.Agitate` — Task 2 and later callers read this.

There is no lightweight way to unit-test a getter on these classes in isolation: `Intake.getInstance()`/`Shooter.getInstance()` require `Constants.currentMode`, which resolves via `RobotBase.isReal()` and needs a running `HAL`, and neither subsystem exposes a reset hook. The codebase's own precedent (Phase 3 Test A, `Vision.getIMUMode()`) did not add a narrow unit test for the getter either — it was proven correct end-to-end by the full-lifecycle test plus a negative control. This task's own test is the **static gate** (compiles clean); Task 2 is where these getters get exercised and proven correct.

- [ ] **Step 1: Add the roller-state getter to `Intake`**

In `src/main/java/frc/robot/subsystems/intake/Intake.java`, add a field next to the existing `sysIdActive` field (around line 44):

```java
  private boolean sysIdActive = false;
  private Roller currentRoller = Roller.STOP;
```

Then update `setRoller` (currently lines 102-109) to record it:

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

  /** The roller state most recently sent via {@link #setRoller(Roller)}. */
  public Roller getRollerState() {
    return currentRoller;
  }
```

- [ ] **Step 2: Add the indexer/agitator-state getters to `Shooter`**

In `src/main/java/frc/robot/subsystems/shooter/Shooter.java`, add two fields next to `shooterTuningModeEnable` (around line 44):

```java
  public boolean shooterTuningModeEnable = false;
  private Agitate currentAgitate = Agitate.STOP;
  private indexing currentIndexing = indexing.STOP;
```

Update `setAgitator` (currently lines 83-86):

```java
  public void setAgitator(Agitate state) {
    io.setAgitatorVoltage(state.voltage().in(Volts));
    currentAgitate = state;
    Logger.recordOutput("Agitator State", state);
  }

  /** The agitator state most recently sent via {@link #setAgitator(Agitate)}. */
  public Agitate getAgitateState() {
    return currentAgitate;
  }
```

Update `indexControl` (currently lines 156-159):

```java
  public void indexControl(indexing state) {
    io.setIndexVoltage(state.voltage().in(Volts));
    currentIndexing = state;
    Logger.recordOutput("Indexer State", state);
  }

  /** The indexer state most recently sent via {@link #indexControl(indexing)}. */
  public indexing getIndexerState() {
    return currentIndexing;
  }
```

- [ ] **Step 3: Static gate — compile clean**

Run:
```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
cd "C:/Users/xdm/Claude/experimental-FRC" && ./gradlew compileJava
```
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 4: Commit**

```bash
git add src/main/java/frc/robot/subsystems/intake/Intake.java src/main/java/frc/robot/subsystems/shooter/Shooter.java
git commit -m "intake/shooter: add live-state getters for roller/indexer/agitator"
```

---

### Task 2: `SuperstructureEjectingTest` — the behavior proof

**Files:**
- Create: `src/test/java/frc/robot/SuperstructureEjectingTest.java`

**Interfaces:**
- Consumes:
  - `Intake.getRollerState(): Intake.Roller` (Task 1)
  - `Shooter.getIndexerState(): Shooter.indexing` (Task 1)
  - `Shooter.getAgitateState(): Shooter.Agitate` (Task 1)
  - `Superstructure.getInstance(CommandSwerveDrivetrain): Superstructure` and `Superstructure.ejectCmd(): Command` — already exist (`src/main/java/frc/robot/subsystems/superstructure/Superstructure.java:38,145-147`).
  - `Intake.getInstance(): Intake`, `Shooter.getInstance(): Shooter` — already exist.
- Produces: nothing consumed by later tasks — this is the terminal proof for Test B.

This test intentionally reuses `RobotLifecycleTest`'s harness shape (`src/test/java/frc/robot/RobotLifecycleTest.java`) — real `startCompetition()` loop on a daemon thread, `DriverStationSim`/`SimHooks` driving ticks — because that is the only way in this codebase to get `Superstructure`/`Intake`/`Shooter` constructed with a real drivetrain and to get `Superstructure.periodic()` actually invoked each tick (it runs off `CommandScheduler.getInstance().run()`, called from `Robot`'s loop). `Superstructure.getInstance(null)` is safe here for the same reason `Vision.getInstance(null)` is safe in `RobotLifecycleTest`: by the time `setup()` returns, `new Robot()` has already run `robotInit()` (blocked on via `SimHooks.waitForProgramStart()`), which already constructed the real singleton with a real drivetrain — the argument is only consulted on the very first call.

- [ ] **Step 1: Write the test**

```java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Second test class in this package deliberately -- build.gradle's forkEvery = 1 forks a fresh
 * JVM per test CLASS (not per method), which is what keeps this class's Superstructure/Intake/
 * Shooter singletons isolated from RobotLifecycleTest's. Kept to a single @Test method for the
 * same reason RobotLifecycleTest is: those singletons have no reset hook, so a second method in
 * this class would silently observe state left behind by the first.
 */
class SuperstructureEjectingTest {
  private Robot robot;
  private Thread robotThread;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    robot = new Robot();
    robotThread = new Thread(robot::startCompetition, "SuperstructureEjectingTest-competition");
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
  void ejectCmdDrivesEjectingThenCancelStopsRoller() {
    assertTrue(robotThread.isAlive(),
        "startCompetition() loop should still be running after waitForProgramStart()");

    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.1); // ~5 20ms cycles of teleopPeriodic

    Superstructure superstructure = Superstructure.getInstance(null);
    Intake intake = Intake.getInstance();
    Shooter shooter = Shooter.getInstance();

    assertEquals(Intake.Roller.STOP, intake.getRollerState(),
        "nothing scheduled yet: Superstructure's OFF state should hold the roller stopped");

    Command eject = superstructure.ejectCmd();
    CommandScheduler.getInstance().schedule(eject);
    SimHooks.stepTiming(0.1); // let EJECTING's periodic() commands land

    assertEquals(Intake.Roller.EJECT, intake.getRollerState(),
        "EJECTING should drive the intake roller in reverse");
    assertEquals(Shooter.indexing.EJECT, shooter.getIndexerState(),
        "EJECTING should drive the indexer in reverse");
    assertEquals(Shooter.Agitate.OUT, shooter.getAgitateState(),
        "EJECTING should drive the agitator outward");

    CommandScheduler.getInstance().cancel(eject);
    SimHooks.stepTiming(0.1); // let STOWED's periodic() commands land

    assertEquals(Intake.Roller.STOP, intake.getRollerState(),
        "cancelling ejectCmd() should request STOWED, which commands the roller to stop");
  }
}
```

- [ ] **Step 2: Run it to verify it currently fails to compile/run cleanly without Task 1**

This step only applies if Task 1 has not landed yet. If Task 1 is already committed, skip straight to Step 3 — there is nothing to observe failing, since the getters already exist and the production `EJECTING`/`STOWED` cases in `Superstructure.periodic()` already implement the behavior under test (this test proves existing behavior, the way `RobotLifecycleTest` proved the existing IMUMode fix — see the plan's Architecture section).

- [ ] **Step 3: Run the test and verify it passes**

Run:
```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
cd "C:/Users/xdm/Claude/experimental-FRC" && ./gradlew test --tests "frc.robot.SuperstructureEjectingTest"
```
Expected: `BUILD SUCCESSFUL`, 1 test PASSED.

- [ ] **Step 4: Negative control — prove the assertions actually discriminate**

Temporarily change one expected value, e.g. the `Intake.Roller.EJECT` assertion to `Intake.Roller.INTAKE`:
```java
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState(),
```
Run the same command from Step 3 again. Expected: FAILS with a real `AssertionFailedError: ... expected: <INTAKE> but was: <EJECT>`. Then revert the change back to `Intake.Roller.EJECT` and re-run to confirm it's green again. This mirrors the negative control Phase 3 Test A ran on `RobotLifecycleTest` (`docs/superpowers/plans/2026-07-17-phase3-imu-mode-junit-test.md`) — don't skip it; a test that always passes by construction is worthless.

- [ ] **Step 5: Commit**

```bash
git add src/test/java/frc/robot/SuperstructureEjectingTest.java
git commit -m "superstructure: add EJECTING behavior proof (phase 3 test B)"
```

---

### Task 3: `RobotLifecycleTest` teardown hardening + document the `startCompetition()` lesson

**Files:**
- Modify: `src/test/java/frc/robot/RobotLifecycleTest.java:43-49`
- Modify: `build.gradle:89-98`

**Interfaces:**
- Consumes: nothing.
- Produces: nothing consumed elsewhere — this is cleanup deferred from Phase 3 Test A's final review, explicitly scheduled for "the same session a second test class gets added to this file/package" (see `CLAUDE.md` Next-up). Task 2 just added that second class.

- [ ] **Step 1: Harden `RobotLifecycleTest`'s teardown**

In `src/test/java/frc/robot/RobotLifecycleTest.java`, add `assertFalse` to the existing static import (line 8):

```java
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
```

Replace the existing `teardown()` (lines 43-49):

```java
  @AfterEach
  void teardown() throws InterruptedException {
    robot.endCompetition();
    robotThread.join(1000);
    robot.close();
    SimHooks.resumeTiming();
  }
```

with:

```java
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
```

This matches the teardown `SuperstructureEjectingTest` already uses (Task 2) — both classes now fail loudly instead of silently proceeding if the robot thread doesn't exit in time, and `SimHooks.resumeTiming()` always runs even if an assertion or `close()` throws.

- [ ] **Step 2: Document the "must use `startCompetition()`" lesson next to the `forkEvery` comment**

In `build.gradle`, the `test {}` block (lines 89-98) currently reads:

```groovy
test {
    useJUnitPlatform()
    systemProperty 'junit.jupiter.extensions.autodetection.enabled', 'true'
    // Forks a fresh JVM per test CLASS (Gradle semantics, not per method): Vision/Intake/
    // Shooter/Superstructure are Singletons with no reset hook, so reusing a JVM across test
    // classes would cross-wire a later class's Robot with an earlier class's cached singleton
    // instances. Within a class, methods still share one JVM -- see RobotLifecycleTest, which
    // is deliberately kept to a single @Test method for exactly this reason.
    forkEvery = 1
}
```

Replace it with:

```groovy
test {
    useJUnitPlatform()
    systemProperty 'junit.jupiter.extensions.autodetection.enabled', 'true'
    // Forks a fresh JVM per test CLASS (Gradle semantics, not per method): Vision/Intake/
    // Shooter/Superstructure are Singletons with no reset hook, so reusing a JVM across test
    // classes would cross-wire a later class's Robot with an earlier class's cached singleton
    // instances. Within a class, methods still share one JVM -- see RobotLifecycleTest and
    // SuperstructureEjectingTest, each deliberately kept to a single @Test method for exactly
    // this reason.
    //
    // Any test that boots Robot must drive the real startCompetition() loop on a background
    // thread via DriverStationSim/SimHooks, never call disabledInit()/autonomousInit()/etc.
    // directly: AdvantageKit's Logger.periodicBeforeUser()/periodicAfterUser() -- which flush
    // recordOutput() calls to the data receivers and back every getter these tests assert on --
    // only run inside that loop. A direct-call test would pass while proving nothing. See
    // docs/superpowers/plans/2026-07-17-phase3-imu-mode-junit-test.md for how this was found.
    forkEvery = 1
}
```

- [ ] **Step 3: Verify both test classes still pass**

Run:
```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
cd "C:/Users/xdm/Claude/experimental-FRC" && ./gradlew test
```
Expected: `BUILD SUCCESSFUL`, both `RobotLifecycleTest` and `SuperstructureEjectingTest` reported PASSED (2 tests total).

- [ ] **Step 4: Commit**

```bash
git add src/test/java/frc/robot/RobotLifecycleTest.java build.gradle
git commit -m "test: harden RobotLifecycleTest teardown, document startCompetition() requirement"
```

---

## Final Verification (run after Task 3)

Per this repo's `CLAUDE.md` Advanced Agent Verification Loop, before claiming Phase 3 Test B complete:

1. **Static gate:** `./gradlew compileJava` → `BUILD SUCCESSFUL`.
2. **Behavior gate (2.5):** `./gradlew test` → `BUILD SUCCESSFUL`, both test classes PASSED.
3. Update `CLAUDE.md`'s Current Task State: move the Test B bullet from Next-up to Done, citing `Intake.java`/`Shooter.java` getter line numbers and the new test file path/commit SHA. Remove the "`RobotLifecycleTest` teardown hardening" Next-up bullet (closed by Task 3).
