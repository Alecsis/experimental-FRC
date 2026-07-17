# Phase 3 Test A — JUnit Sim Proof of the IMUMode 1→4 Autonomous Transition

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prove — with a real JUnit test plus real `.wpilog` bytes, not eyeballed code — that `Robot.autonomousInit()` actually switches the Limelight from `IMUMode(1)` (EXTERNAL_SEED, held during `disabledPeriodic`) to `IMUMode(4)` (fusion), closing gate 3 of the Advanced Agent Verification Loop for the 2026-07-17 MegaTag2 auto-snap fix.

**Architecture:** A single JUnit 5 test class (`RobotLifecycleTest`) runs the *actual* `Robot.startCompetition()` loop on a background thread — not direct calls to `disabledInit()`/`autonomousInit()` — because AdvantageKit's `Logger.periodicBeforeUser()`/`periodicAfterUser()` (which flush `recordOutput` calls to the SIM `WPILOGWriter` added in Phase 1) only run inside that loop (confirmed by reading `LoggedRobot.java` from the local AdvantageKit 26.0.2 sources jar — direct method calls would produce a test that passes without ever writing real log evidence). `DriverStationSim` + `SimHooks.pauseTiming()/stepTiming()` drive the loop deterministically through disabled → autonomous. Built in two passes **inside one `@Test` method** (see Global Constraints for why it can't be two methods): Task 2 proves the harness itself — background thread boots, `waitForProgramStart()` returns, mode reads 1 while disabled — before Task 3 extends the same method with the autonomous step and the 1→4 assertion.

**Tech Stack:** JUnit 5 (already wired in `build.gradle`), WPILib 2026.2.1 simulation APIs (`HAL`, `DriverStationSim`, `SimHooks` — signatures below extracted directly from the local `wpilibj-java-2026.2.1-sources.jar`/`hal-java-2026.2.1-sources.jar`, not guessed), AdvantageKit 26.0.2 (`LoggedRobot`, `Logger`).

## Global Constraints
- No vendor hardware APIs outside `*IOReal.java` (Architecture Rule 1) — this plan touches no vendor code.
- `./gradlew compileJava` must exit 0 after every task (Karpathy rule: never claim a change works before this passes). Use `JAVA_HOME=/c/Users/Public/wpilib/2026/jdk` (WPILib JDK 17).
- Verified method signatures (do not deviate without re-checking):
  - `edu.wpi.first.hal.HAL.initialize(int timeout, int mode)` — native, boolean return.
  - `edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled(boolean)`, `.setAutonomous(boolean)`, `.notifyNewData()`, `.resetData()`.
  - `edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming()`, `.resumeTiming()`, `.stepTiming(double deltaSeconds)`, `.waitForProgramStart()`.
  - `LoggedRobot.startCompetition()` calls `robotInit()`, then loops calling `Logger.periodicBeforeUser()` → `loopFunc()` → `Logger.periodicAfterUser(...)` every cycle; `endCompetition()` stops its internal `Notifier`. `close()` releases it.
- **`forkEvery` forks per test *class*, not per test method** (standard, stable Gradle `Test` task semantics). `Vision`/`Intake`/`Shooter`/`Superstructure` are Singletons (`private static instance`, no reset hook) — two `@Test` methods in the same class would share one forked JVM and the second would silently reuse the first's cached singletons, bound to a stale drivetrain. **Consequence: `RobotLifecycleTest` stays a single `@Test` method for the life of this plan.** If a future need arises for a second scenario, it goes in a second test *class*, not a second method here.
- This plan covers **Test A only** (IMU mode transition). Test B (EJECTING mechanism assertions) needs a preliminary fix first — `Intake.setRoller()` doesn't log its commanded state the way `Shooter.setAgitator()`/`indexControl()` already do (`"Agitator State"`/`"Indexer State"` outputs), so there's nothing in the log to assert EJECT vs STOP against yet. That's a separate, smaller plan — not in scope here.

---

### Task 1: Test scaffolding + harness — getter, `forkEvery`, and a `RobotLifecycleTest` that proves the competition loop boots

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/vision/Vision.java:141-149`
- Modify: `build.gradle:89-92`
- Create: `src/test/java/frc/robot/RobotLifecycleTest.java`

**Interfaces:**
- Produces: `public int Vision.getIMUMode()` — returns the `currentIMUMode` field Phase 2 already added.
- Produces: `RobotLifecycleTest` class with `@BeforeEach setup()` / `@AfterEach teardown()` harness (HAL init → `DriverStationSim.resetData()` → `SimHooks.pauseTiming()` → construct `Robot` → run `startCompetition()` on a daemon thread → `SimHooks.waitForProgramStart()`; teardown calls `endCompetition()`/`join`/`close()`/`resumeTiming()`) and one `@Test` method, `competitionLoopInitializesAndHoldsSeedModeWhileDisabled`, which Task 2 will extend in place (same method, do not add a second one — see Global Constraints).

- [ ] **Step 1: Add the getter**

In `Vision.java`, right after `setIMUMode`:

```java
  /** Forwards to the active VisionIO implementation. See {@link VisionIO#setIMUMode}. */
  public void setIMUMode(int mode) {
    io.setIMUMode(mode);
    currentIMUMode = mode;
  }

  /** The IMU mode most recently sent via {@link #setIMUMode(int)}. */
  public int getIMUMode() {
    return currentIMUMode;
  }
```

- [ ] **Step 2: Add `forkEvery` to the test task**

In `build.gradle`, the existing `test { ... }` block:

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

- [ ] **Step 3: Write the harness + first assertion**

```java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.vision.Vision;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Drives the real {@link Robot#startCompetition()} loop (not direct disabledInit/autonomousInit
 * calls) so AdvantageKit's periodicBeforeUser/periodicAfterUser flush cycle actually runs and the
 * SIM-mode WPILOGWriter (Robot.java) actually receives the Vision/IMUMode records this test
 * asserts on. Kept to a single @Test method deliberately -- see the plan's Global Constraints on
 * forkEvery granularity and the Vision/Intake/Shooter/Superstructure singletons.
 */
class RobotLifecycleTest {
  private Robot robot;
  private Thread robotThread;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    robot = new Robot();
    robotThread = new Thread(robot::startCompetition, "RobotLifecycleTest-competition");
    robotThread.setDaemon(true);
    robotThread.start();
    SimHooks.waitForProgramStart();
  }

  @AfterEach
  void teardown() throws InterruptedException {
    robot.endCompetition();
    robotThread.join(1000);
    robot.close();
    SimHooks.resumeTiming();
  }

  @Test
  @Timeout(30)
  void competitionLoopInitializesAndHoldsSeedModeWhileDisabled() {
    assertTrue(robotThread.isAlive(),
        "startCompetition() loop should still be running after waitForProgramStart()");

    // Disabled: matches a real boot sitting on the field before auto starts.
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.1); // ~5 20ms cycles of disabledPeriodic

    assertEquals(1, Vision.getInstance(null).getIMUMode(),
        "disabledInit/disabledPeriodic should hold IMUMode at 1 (EXTERNAL_SEED)");
  }
}
```

Note on `Vision.getInstance(null)`: by the time the test calls this, `new Robot()` (in `setup()`) has already constructed the Vision singleton via `Vision.getInstance(m_robotContainer.drivetrain)`. `Vision.getInstance` only uses its argument on the very first call (`if (instance == null)`), so `null` here is safe and returns the same live instance the running robot is using.

- [ ] **Step 4: Compile**

Run: `export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk" && ./gradlew compileJava --console=plain`
Expected: `BUILD SUCCESSFUL`

- [ ] **Step 5: Run the harness test**

Run: `export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk" && ./gradlew test --console=plain --tests "frc.robot.RobotLifecycleTest"`
Expected: `BUILD SUCCESSFUL`, one test executed, zero failures. If `HAL.initialize` throws `UnsatisfiedLinkError`, the GradleRIO-configured native library path isn't reaching the forked test JVM — stop and report this as a blocker rather than working around it (do not delete `forkEvery` to make it "pass"). If the test hangs past the 30s `@Timeout`, `waitForProgramStart()` never returned — report the full test output, don't retry blindly.

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/vision/Vision.java build.gradle src/test/java/frc/robot/RobotLifecycleTest.java
git commit -m "phase 3 harness: robot competition loop boots, holds imu seed mode while disabled"
```

---

### Task 2: Extend the same test to the autonomous 1→4 transition

**Files:**
- Modify: `src/test/java/frc/robot/RobotLifecycleTest.java` — extend `competitionLoopInitializesAndHoldsSeedModeWhileDisabled` in place (rename it; do not add a second `@Test` method — see Global Constraints).

**Interfaces:**
- Consumes: the harness and `Vision.getIMUMode()` from Task 1, unchanged.

- [ ] **Step 1: Extend the test method**

Rename `competitionLoopInitializesAndHoldsSeedModeWhileDisabled` to `autonomousInitSwitchesIMUModeFromSeedToFusion`, and after the existing disabled-mode assertion, append:

```java
    // Autonomous: the actual 2026-07-17 fix under test.
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.1); // ~5 20ms cycles of autonomousPeriodic

    assertEquals(4, Vision.getInstance(null).getIMUMode(),
        "autonomousInit should switch IMUMode to 4 (fusion) -- verifies the auto-snap fix");
```

- [ ] **Step 2: Run it**

Run: `export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk" && ./gradlew test --console=plain --tests "frc.robot.RobotLifecycleTest"`
Expected: `BUILD SUCCESSFUL`, one test executed, zero failures.

- [ ] **Step 3: Confirm it actually fails when it should**

Temporarily change the final `assertEquals` expected value from `4` to `5`, rerun the same command, confirm it fails with an `AssertionFailedError` naming the real captured value. Revert the temporary change and rerun to confirm green again.

- [ ] **Step 4: Commit**

```bash
git add src/test/java/frc/robot/RobotLifecycleTest.java
git commit -m "phase 3 test A: junit proof of imu mode 1->4 auto transition"
```

---

### Task 3: Evidence gate — parse the `.wpilog` the test just wrote

**Files:** none modified — this task runs existing tooling against the test's output.

- [ ] **Step 1: Find the log the test run produced**

Run: `ls -t logs/*.wpilog | head -1`

- [ ] **Step 2: Confirm the transition from log bytes, not memory**

Run: `python SKILLS/parse_akit_log.py logs/<newest>.wpilog --dump "/RealOutputs/Vision/IMUMode"`
Expected: at least two distinct values in the dump — `1` early, `4` later — matching Task 2's assertions.

- [ ] **Step 3: Run the pose-jump analysis PLAN.md's Test A calls for**

Run: `python SKILLS/parse_akit_log.py logs/<newest>.wpilog --pose-entry "/RealOutputs/Odometry/Robot" --jump-threshold 0.15`
Report whatever it prints verbatim. If no vision-driven pose corrections happen in this short a sim window (likely — no simulated AprilTag target is set up for this test), a no-jumps result is a valid, honestly-reported outcome, not a failure; it just means this test exercises the mode-switch side of the fix, not the pose-correction side.

---

### Task 4: Wire `./gradlew test` into the Verification Loop as gate 2.5

**Files:**
- Modify: `claude.md` — `## 🔁 Advanced Agent Verification Loop` section, "The lifecycle" list.

- [ ] **Step 1: Insert gate 2.5 between the existing gate 2 and gate 3 entries**

```markdown
2.5. **Behavior gate:** `./gradlew test`. Runs the JUnit 5 suite under `src/test/java` — currently
   `RobotLifecycleTest`, which drives the real `Robot.startCompetition()` loop via
   `DriverStationSim`/`SimHooks` through disabled → autonomous and asserts the `Vision/IMUMode`
   1→4 transition. Unlike gate 2, this exercises actual mode-transition behavior, not just
   launch/construction. A PASS here plus a parsed `.wpilog` (gate 3) is what "verified" means for
   this specific fix — see `docs/superpowers/plans/2026-07-17-phase3-imu-mode-junit-test.md`.
```

- [ ] **Step 2: Commit**

```bash
git add claude.md
git commit -m "verify loop: wire gradlew test as gate 2.5"
```

---

## Self-Review

- **Spec coverage:** PLAN.md's Phase 3 "Test A" bullet (assert `Vision/IMUMode` 1→4 transition, parse the log, run pose-jump analysis) — covered by Tasks 1–3, staged per the user's request to prove the harness boots before layering on the timing-driven mode assertions. "Wire `./gradlew test` into the Verification Loop as gate 2.5" — covered by Task 4. Test B (EJECTING) — explicitly out of scope, noted in Global Constraints with the concrete blocker.
- **Placeholder scan:** no TBD/TODO; every step has literal code or an exact command with an expected result.
- **Type consistency:** `Vision.getIMUMode()` (Task 1) is the only new symbol; Task 1's own test method and Task 2's extension of it are the only consumers — matches. `forkEvery` per-class granularity is stated once in Global Constraints and referenced (not restated inconsistently) in both the build.gradle comment and the test class Javadoc.
