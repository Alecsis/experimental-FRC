# Autonomous Completion-Trigger Framework Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the ad hoc fixed-duration timeout pattern used by every autonomous mechanism sequence with a generic, composable "run until a condition is met, or a timeout elapses" primitive, backed by sim-testable pose/path-based conditions today and pluggable to a real sensor later.

**Architecture:** Generalize `Superstructure.intakeSequence`/`shootingSequence` to accept an optional `BooleanSupplier` early-exit condition (default `() -> false`, preserving current behavior exactly), add a small condition-supplier library (`utility/AutoTriggers.java`) sourced from already-simulated drivetrain pose/path-following state, and add a structural JUnit test that prevents any future NamedCommand from reintroducing the `intakeCmd()`-hang bug class.

**Tech Stack:** Java 17, WPILib command-based v2 (`edu.wpi.first.wpilibj2.command`), AdvantageKit (`org.littletonrobotics.junction.Logger`), PathPlannerLib 2026.1.2, JUnit 5, Jackson `ObjectMapper` (already a test dependency, used by `AutoRegressionTestBase`).

## Global Constraints

- No vendor hardware APIs (CTRE Phoenix, REVLib) in any new file outside `*IOReal.java` or the sanctioned `CommandSwerveDrivetrain`/`MapleSimSwerveDrivetrain` exception (`CLAUDE.md` Rule 1).
- Zero behavior change to any existing auto unless a call site is explicitly opted in — every new overload's default must be provably identical to today's behavior.
- Every step that touches production Java ends with `./gradlew compileJava` passing (JDK 17 via `JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"`, not the PATH JDK 25).
- Follow the existing test harness conventions exactly: `forkEvery=1`/one-JUnit-class-per-behavior discipline (`docs/claudex/architecture.md`'s "Auto regression testing pattern" section) — do not add a second `@Test` method to `SuperstructureEjectingTest` or loop multiple autos in one class.
- No changes to `SuperstructureState` enum or `periodic()`'s switch-case arbitration logic.

---

## Task 1: `Superstructure` — generalized conditional sequences with end-reason telemetry

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/superstructure/Superstructure.java:154-170` (the existing `shootingSequence`/`intakeSequence` methods)
- Test: `src/test/java/frc/robot/SuperstructureConditionalSequenceTest.java` (new)

**Interfaces:**
- Produces: `Superstructure.EndReason` (public nested enum: `CONDITION_MET`, `TIMED_OUT`); `Superstructure.intakeSequence(double)`, `Superstructure.intakeSequence(BooleanSupplier, double)`, `Superstructure.shootingSequence(double)`, `Superstructure.shootingSequence(BooleanSupplier, double)` — all return `Command`.
- Consumes: nothing new (uses `Superstructure`'s own existing `intakeCmd()`, `requestShoot(double)`, `requestStow()`, `mShotInProgress`).

- [ ] **Step 1: Write the failing test**

Create `src/test/java/frc/robot/SuperstructureConditionalSequenceTest.java`:

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
import frc.robot.subsystems.superstructure.Superstructure;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Second test class in this package deliberately -- build.gradle's forkEvery = 1 forks a fresh
 * JVM per test CLASS, matching SuperstructureEjectingTest's own stated reasoning for why its
 * Superstructure/Intake/Shooter singletons must not be shared with another class's test bodies.
 */
class SuperstructureConditionalSequenceTest {
  private Robot robot;
  private Thread robotThread;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    robot = new Robot();
    robotThread = new Thread(robot::startCompetition, "SuperstructureConditionalSequenceTest-competition");
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
  void intakeSequenceFinishesEarlyWhenConditionMetBeforeTimeout() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.1);

    Superstructure superstructure = Superstructure.getInstance(null);
    boolean[] conditionMet = {false};

    Command sequence = superstructure.intakeSequence(() -> conditionMet[0], 5.0);
    CommandScheduler.getInstance().schedule(sequence);
    SimHooks.stepTiming(0.1); // let it start

    assertTrue(CommandScheduler.getInstance().isScheduled(sequence),
        "sequence should still be running before the condition flips");

    conditionMet[0] = true;
    SimHooks.stepTiming(0.1); // one tick for the scheduler to observe the flipped condition

    assertFalse(CommandScheduler.getInstance().isScheduled(sequence),
        "sequence should finish as soon as the condition is met, well before the 5.0s timeout");
  }

  @Test
  @Timeout(30)
  void intakeSequenceFallsBackToTimeoutWhenConditionNeverMet() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.1);

    Superstructure superstructure = Superstructure.getInstance(null);

    Command sequence = superstructure.intakeSequence(() -> false, 0.2);
    CommandScheduler.getInstance().schedule(sequence);
    SimHooks.stepTiming(0.1);

    assertTrue(CommandScheduler.getInstance().isScheduled(sequence),
        "sequence should still be running before the 0.2s timeout elapses");

    SimHooks.stepTiming(0.3); // past the 0.2s timeout

    assertFalse(CommandScheduler.getInstance().isScheduled(sequence),
        "sequence should finish via the timeout fallback when the condition never fires");
  }

  @Test
  @Timeout(30)
  void shootingSequenceFinishesEarlyAndStillRequestsStow() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.1);

    Superstructure superstructure = Superstructure.getInstance(null);
    boolean[] conditionMet = {false};

    Command sequence = superstructure.shootingSequence(() -> conditionMet[0], 5.0);
    CommandScheduler.getInstance().schedule(sequence);
    SimHooks.stepTiming(0.1);

    conditionMet[0] = true;
    SimHooks.stepTiming(0.1);

    assertFalse(CommandScheduler.getInstance().isScheduled(sequence),
        "shooting sequence should finish as soon as the condition is met");
    assertEquals(Superstructure.SuperstructureState.STOWED, superstructure.getSystemState(),
        "finishing early must still request STOWED -- this is the exact regression risk an "
            + "external .until() wrapper without finallyDo() cleanup would have introduced");
  }
}
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew test --tests "frc.robot.SuperstructureConditionalSequenceTest"
```
Expected: FAIL to compile — `intakeSequence(BooleanSupplier, double)` and `shootingSequence(BooleanSupplier, double)` don't exist yet.

- [ ] **Step 3: Implement the generalized sequences in `Superstructure.java`**

Add the import (top of file, alongside the existing imports):

```java
import java.util.function.BooleanSupplier;
```

Add the nested enum right after the existing `SuperstructureState` enum (`Superstructure.java:31-33`):

```java
  public enum SuperstructureState {
    OFF, INTAKING, EJECTING, STOWED, ALIGNING, SHOOTING
  }

  /** Why a bounded sequence Command finished -- CONDITION_MET requires an explicit early-exit
   * BooleanSupplier to have been supplied and to have fired; the single-argument overloads always
   * resolve to TIMED_OUT, matching their pre-existing behavior. */
  public enum EndReason {
    CONDITION_MET, TIMED_OUT
  }
```

Replace `Superstructure.java:154-170` (the current `shootingSequence(double)`/`intakeSequence(double)`):

```java
  /** Bridges the state machine into a bounded, self-finishing Command for autonomous/NamedCommands. */
  public Command shootingSequence(double timeoutSeconds) {
    return shootingSequence(() -> false, timeoutSeconds);
  }

  /**
   * Same as {@link #shootingSequence(double)}, but also finishes early the moment {@code
   * doneEarly} becomes true. {@code shootingSequence} is not built on {@link #shootCmd()}'s
   * startEnd shape -- it drives {@link #requestShoot(double)} directly and waits on {@code
   * mShotInProgress} -- so the early-exit condition is folded into the same {@code waitUntil}
   * rather than composed via an external {@code .until()}, and {@link #requestStow()} is called
   * explicitly in {@code finallyDo} so a condition-triggered exit cleans up the state machine
   * exactly like the internal {@code mFeedTimeoutSeconds} path already does.
   */
  public Command shootingSequence(BooleanSupplier doneEarly, double timeoutSeconds) {
    return Commands.sequence(
            Commands.runOnce(() -> requestShoot(timeoutSeconds)),
            Commands.waitUntil(() -> !mShotInProgress || doneEarly.getAsBoolean()))
        .finallyDo(interrupted -> {
          boolean earlyExit = doneEarly.getAsBoolean();
          requestStow();
          Logger.recordOutput("Superstructure/ShootSequenceEndReason",
              (earlyExit ? EndReason.CONDITION_MET : EndReason.TIMED_OUT).name());
        });
  }

  /**
   * Bridges the state machine into a bounded, self-finishing Command for autonomous/NamedCommands.
   * {@link #intakeCmd()} is a hold-forever startEnd built for a teleop button binding -- inside a
   * PathPlanner "parallel" block (which compiles to a plain ParallelCommandGroup, requiring every
   * branch to finish) it never releases the group, silently stalling the auto after its first path
   * segment. This wraps it with a timeout so it always finishes on its own.
   */
  public Command intakeSequence(double timeoutSeconds) {
    return intakeSequence(() -> false, timeoutSeconds);
  }

  /**
   * Same as {@link #intakeSequence(double)}, but also finishes early the moment {@code doneEarly}
   * becomes true. Safe to compose via {@code .until()} because {@link #intakeCmd()} is a plain
   * startEnd command -- its {@code end()} calls {@link #requestStow()} regardless of which race
   * participant (the condition or the timeout) ends it.
   */
  public Command intakeSequence(BooleanSupplier doneEarly, double timeoutSeconds) {
    return intakeCmd().until(doneEarly).withTimeout(timeoutSeconds)
        .finallyDo(interrupted -> Logger.recordOutput("Superstructure/IntakeSequenceEndReason",
            (doneEarly.getAsBoolean() ? EndReason.CONDITION_MET : EndReason.TIMED_OUT).name()));
  }
```

- [ ] **Step 4: Compile and run the tests**

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew compileJava
./gradlew test --tests "frc.robot.SuperstructureConditionalSequenceTest"
```
Expected: `BUILD SUCCESSFUL`, all 3 tests PASS.

- [ ] **Step 5: Run the full existing suite to confirm no regression**

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew test
```
Expected: every existing test class still passes, including `SuperstructureEjectingTest` and `LtNeutralAutoRegressionTest` (the latter exercises `RobotContainer`'s real `intakeSequence(5.0)`/`shootingSequence(5.0)` NamedCommand registrations end-to-end — since both single-argument overloads now just delegate to `() -> false`, its golden numbers must be unchanged).

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/superstructure/Superstructure.java src/test/java/frc/robot/SuperstructureConditionalSequenceTest.java
git commit -m "Add conditional early-exit overloads to Superstructure's bounded sequences"
```

---

## Task 2: `CommandSwerveDrivetrain` — expose path-following-active state

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java:97` (field block), `:380-388` (existing `setLogActivePathCallback`)
- Test: covered indirectly by Task 3's test and the existing regression suite (no dedicated new test file — this is a two-line observable-state exposure over an already-tested callback, matching the precedent set by `TrajectoryErrorTracker`'s own ingest hooks, which also have no dedicated unit test of their own).

**Interfaces:**
- Produces: `CommandSwerveDrivetrain.isFollowingAutoPath()` returning `boolean`.
- Consumes: nothing new (reuses the existing `PathPlannerLogging.setLogActivePathCallback` registration already present in `configureAutoBuilder()`).

- [ ] **Step 1: Add the field**

In `CommandSwerveDrivetrain.java`, immediately after the existing `m_hasAppliedSimPracticeSpawn` field (`:97`):

```java
    private boolean m_hasAppliedSimPracticeSpawn = false;
    // Driven by configureAutoBuilder()'s existing PathPlannerLogging.setLogActivePathCallback
    // registration below -- true whenever PathPlanner has an active path (non-empty list),
    // false once it reports the path complete (empty list). Read-only outside this class.
    private boolean m_isFollowingAutoPath = false;
```

- [ ] **Step 2: Update the existing callback to set the flag**

Replace `CommandSwerveDrivetrain.java:380-388`:

```java
        // Logs poses from auto
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    m_isFollowingAutoPath = !activePath.isEmpty();
                    Logger.recordOutput("Drive/IsFollowingAutoPath", m_isFollowingAutoPath);
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                    if (trajectoryErrorTracker != null) {
                        trajectoryErrorTracker.onActivePath(activePath);
                    }
                });
```

- [ ] **Step 3: Add the public getter**

Immediately after `setTrajectoryErrorTracker(...)` (`CommandSwerveDrivetrain.java:404-406`):

```java
    public void setTrajectoryErrorTracker(TrajectoryErrorTracker trajectoryErrorTracker) {
        this.trajectoryErrorTracker = trajectoryErrorTracker;
    }

    /** True whenever PathPlanner currently has an active autonomous path; false once it reports
     * the path list empty (path complete or no auto running). Sourced from the same
     * PathPlannerLogging push callback configureAutoBuilder() already registers for logging. */
    public boolean isFollowingAutoPath() {
        return m_isFollowingAutoPath;
    }
```

- [ ] **Step 4: Compile**

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew compileJava
```
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 5: Verify via the existing regression harness**

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew test --tests "frc.robot.auto.LtNeutralAutoRegressionTest"
python SKILLS/parse_akit_log.py $(ls -t logs/*.wpilog | head -1) --grep IsFollowingAutoPath
```
Expected: test passes with unchanged golden numbers; the grep shows `Drive/IsFollowingAutoPath` toggling `true`/`false` over the run (proves the new flag is live, not just compiling).

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
git commit -m "Expose Drive/IsFollowingAutoPath from the existing PathPlannerLogging callback"
```

---

## Task 3: `utility/AutoTriggers.java` — condition supplier library

**Files:**
- Create: `src/main/java/frc/robot/utility/AutoTriggers.java`
- Test: `src/test/java/frc/robot/utility/AutoTriggersTest.java` (new)

**Interfaces:**
- Consumes: `CommandSwerveDrivetrain.getState().Pose` (existing, `edu.wpi.first.math.geometry.Pose2d`); `CommandSwerveDrivetrain.isFollowingAutoPath()` (from Task 2).
- Produces: `AutoTriggers.withinToleranceOf(CommandSwerveDrivetrain, Pose2d, double)` and `AutoTriggers.pathFollowingComplete(CommandSwerveDrivetrain)`, both returning `BooleanSupplier`. Not consumed by any other task in this plan — call sites are deliberately deferred (see the design doc's "adoption is a separate, later, low-risk change").

- [ ] **Step 1: Write the failing test**

Create `src/test/java/frc/robot/utility/AutoTriggersTest.java`, mirroring `CommandSwerveDrivetrainSanitizeSpeedsTest`'s established setup/teardown pattern for a non-singleton, drivetrain-only test (`SimHooks.pauseTiming()`/`resumeTiming()` bookends, no full `Robot`/`stepTiming` needed since these are synchronous field reads):

```java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * Constructs CommandSwerveDrivetrain directly via TunerConstants.createDrivetrain(), matching
 * CommandSwerveDrivetrainSanitizeSpeedsTest's own precedent -- CommandSwerveDrivetrain is not a
 * singleton (docs/claudex/architecture.md's "Rule 2 note"), so a fresh instance per test class is
 * safe without the forkEvery=1/full-Robot machinery the Superstructure-touching tests need.
 */
class AutoTriggersTest {
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
  @Timeout(15)
  void withinToleranceOfIsFalseFarAwayAndTrueAtTarget() {
    Pose2d farAway = new Pose2d(20.0, 20.0, Rotation2d.kZero);
    BooleanSupplier nearOrigin = AutoTriggers.withinToleranceOf(drivetrain, Pose2d.kZero, 0.5);

    drivetrain.resetPose(farAway);
    assertFalse(nearOrigin.getAsBoolean(), "20m away must read false for a 0.5m tolerance");

    drivetrain.resetPose(Pose2d.kZero);
    assertTrue(nearOrigin.getAsBoolean(),
        "after resetPose(kZero), a 0.5m tolerance around the origin must read true");
  }

  @Test
  @Timeout(15)
  void pathFollowingCompleteReflectsIsFollowingAutoPath() {
    BooleanSupplier complete = AutoTriggers.pathFollowingComplete(drivetrain);

    assertTrue(complete.getAsBoolean(),
        "no path has ever run -- isFollowingAutoPath() defaults false, so 'complete' must be true");
  }
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew test --tests "frc.robot.utility.AutoTriggersTest"
```
Expected: FAIL to compile — `frc.robot.utility.AutoTriggers` doesn't exist yet.

- [ ] **Step 3: Implement `AutoTriggers.java`**

Create `src/main/java/frc/robot/utility/AutoTriggers.java`:

```java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;

/**
 * Condition-supplier library for Superstructure's bounded sequences (see
 * Superstructure#intakeSequence(BooleanSupplier, double) / #shootingSequence(BooleanSupplier,
 * double)). Every condition here is sourced from signals genuinely simulated today (drivetrain
 * pose, PathPlanner path-following state) -- deliberately excludes any note/fuel-presence
 * signal, which needs a hardware decision not yet made (docs/superpowers/specs/
 * 2026-07-22-autonomous-completion-trigger-framework-design.md's Non-goals).
 */
public final class AutoTriggers {
  private AutoTriggers() {}

  /** True once the drivetrain's fused pose is within {@code toleranceMeters} of {@code target}. */
  public static BooleanSupplier withinToleranceOf(
      CommandSwerveDrivetrain drivetrain, Pose2d target, double toleranceMeters) {
    return () -> drivetrain.getState().Pose.getTranslation().getDistance(target.getTranslation())
        <= toleranceMeters;
  }

  /** True once PathPlanner reports no active path (path complete or none running). */
  public static BooleanSupplier pathFollowingComplete(CommandSwerveDrivetrain drivetrain) {
    return () -> !drivetrain.isFollowingAutoPath();
  }
}
```

- [ ] **Step 4: Compile and run the test**

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew compileJava
./gradlew test --tests "frc.robot.utility.AutoTriggersTest"
```
Expected: `BUILD SUCCESSFUL`, both tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/utility/AutoTriggers.java src/test/java/frc/robot/utility/AutoTriggersTest.java
git commit -m "Add AutoTriggers condition-supplier library for bounded auto sequences"
```

---

## Task 4: `AutoCommandSafetyTest` — structural NamedCommand safety guard

**Files:**
- Modify: `src/main/java/frc/robot/RobotContainer.java:75-94` (add the allowlist constant)
- Create: `src/test/java/frc/robot/auto/AutoCommandSafetyTest.java`

**Interfaces:**
- Consumes: `RobotContainer.BOUNDED_NAMED_COMMANDS` (new `public static final Set<String>` constant); the `.auto` JSON files under `src/main/deploy/pathplanner/autos/`.
- Produces: nothing consumed by later tasks — this is a terminal structural guard.

- [ ] **Step 1: Add the allowlist constant to `RobotContainer.java`**

Add this import near the existing imports (`RobotContainer.java`, alongside `java.util.jar.Attributes.Name` at line 13 — note that pre-existing import is unused/vestigial and unrelated, leave it untouched, this is a different `Set`):

```java
import java.util.Set;
```

Add the constant immediately above the constructor (`RobotContainer.java`, right before `public RobotContainer() {` at line 71):

```java
        /**
         * NamedCommand names that are provably self-finishing (bounded timeout or a
         * finishOnAlign-style condition) and therefore safe to use inside a PathPlanner "parallel"
         * block (which compiles to ParallelCommandGroup, requiring every branch to finish).
         * AutoCommandSafetyTest asserts every parallel-block NamedCommand across all .auto files is
         * in this set -- add a new name here the moment you register a NamedCommand that will be
         * used inside a "parallel" block, or the test will fail and tell you why.
         */
        public static final Set<String> BOUNDED_NAMED_COMMANDS = Set.of(
                        "Intake Start Sequence", "Orbit", "Shooting Sequence", "Quick Shooting");

        public RobotContainer() {
```

- [ ] **Step 2: Compile to confirm the constant alone is valid**

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew compileJava
```
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 3: Write the failing structural test**

Create `src/test/java/frc/robot/auto/AutoCommandSafetyTest.java`:

```java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.RobotContainer;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

/**
 * Static guard against the exact bug class that hung every in-scope auto until fixed (see
 * Superstructure's intakeSequence javadoc / docs/claudex/history.md's thirteenth-session entry):
 * a hold-forever Command registered as a NamedCommand and used inside a PathPlanner "parallel"
 * block (ParallelCommandGroup, requires every branch to finish) silently stalls the auto forever.
 * Pure JSON parsing -- no CommandScheduler, no Robot, no sim, no forkEvery/singleton concerns.
 */
class AutoCommandSafetyTest {
  private static final Path AUTOS_DIR = Paths.get("src", "main", "deploy", "pathplanner", "autos");

  @Test
  void everyParallelBlockNamedCommandIsBounded() throws IOException {
    assertTrue(Files.isDirectory(AUTOS_DIR), "expected " + AUTOS_DIR + " to exist");
    ObjectMapper mapper = new ObjectMapper();
    List<String> violations = new ArrayList<>();

    try (Stream<Path> files = Files.list(AUTOS_DIR)) {
      for (Path autoFile : files.filter(p -> p.toString().endsWith(".auto")).toList()) {
        JsonNode root = mapper.readTree(autoFile.toFile());
        collectParallelBlockViolations(autoFile.getFileName().toString(), root.get("command"),
            violations);
      }
    }

    if (!violations.isEmpty()) {
      fail("Found NamedCommand(s) inside a \"parallel\" block that are not in "
          + "RobotContainer.BOUNDED_NAMED_COMMANDS -- an unbounded (hold-forever) Command here "
          + "will silently stall its auto forever, exactly like the original intakeCmd() bug:\n"
          + String.join("\n", violations));
    }
  }

  private static void collectParallelBlockViolations(String fileName, JsonNode command,
      List<String> violations) {
    if (command == null || !command.has("type")) {
      return;
    }
    String type = command.get("type").asText();
    JsonNode data = command.get("data");
    if ("parallel".equals(type) && data != null && data.has("commands")) {
      for (JsonNode child : data.get("commands")) {
        if ("named".equals(child.get("type").asText())) {
          String name = child.get("data").get("name").asText();
          if (!RobotContainer.BOUNDED_NAMED_COMMANDS.contains(name)) {
            violations.add(fileName + ": \"" + name + "\" used inside a parallel block but not "
                + "in BOUNDED_NAMED_COMMANDS");
          }
        }
      }
    }
    if (data != null && data.has("commands")) {
      for (JsonNode child : data.get("commands")) {
        collectParallelBlockViolations(fileName, child, violations);
      }
    }
  }
}
```

- [ ] **Step 4: Run the test to verify it currently passes (it's a regression guard, not new behavior)**

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew test --tests "frc.robot.auto.AutoCommandSafetyTest"
```
Expected: `BUILD SUCCESSFUL`, PASS — the four names verified by direct inspection in Task 4's design (`Intake Start Sequence`, `Orbit`, `Shooting Sequence`, `Quick Shooting`) are already exactly `BOUNDED_NAMED_COMMANDS`'s contents.

- [ ] **Step 5: Prove the test is non-vacuous** — temporarily remove one entry and confirm it fails

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
git stash push -- src/main/java/frc/robot/RobotContainer.java
```
Manually edit `BOUNDED_NAMED_COMMANDS` to drop `"Orbit"`, then:
```bash
./gradlew test --tests "frc.robot.auto.AutoCommandSafetyTest"
```
Expected: FAILS, listing every `.auto` file that uses `"Orbit"` inside a `parallel` block. Then restore:
```bash
git stash pop
./gradlew test --tests "frc.robot.auto.AutoCommandSafetyTest"
```
Expected: PASSES again, confirming the test actually detects the violation it's designed to catch.

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/RobotContainer.java src/test/java/frc/robot/auto/AutoCommandSafetyTest.java
git commit -m "Add structural test preventing unbounded NamedCommands in parallel auto blocks"
```

---

## Task 5: Regression-suite telemetry assertion — prove the end-reason logging fires end-to-end

**Files:**
- Create: `src/test/java/frc/robot/auto/WpilogEndReasonReader.java`
- Modify: `src/test/java/frc/robot/auto/AutoRegressionTestBase.java` (add one assertion after the existing golden-comparison block, `:227`)

**Interfaces:**
- Consumes: the `/RealOutputs/Superstructure/IntakeSequenceEndReason` wpilog entry (from Task 1); `WpilogTrajectoryErrorReader`'s established `DataLogReader`/`DataLogRecord` pattern (read-only reference, not modified).
- Produces: `WpilogEndReasonReader.wasLogged(Path wpilogFile, String entryName)` returning `boolean`.

- [ ] **Step 1: Write the failing assertion in `AutoRegressionTestBase.java`**

Add immediately after the existing longitudinal-error assertion, before the closing brace of `regressionCheck()` (`AutoRegressionTestBase.java:221-227`):

```java
        assertTrue(
                actual.maxLongitudinalErrorMeters
                        <= golden.maxLongitudinalErrorMeters + AutoRegressionTolerances.kLongitudinalErrorHeadroomMeters,
                () -> autoName() + ": maxLongitudinalErrorMeters " + actual.maxLongitudinalErrorMeters
                        + " exceeds golden " + golden.maxLongitudinalErrorMeters + " + "
                        + AutoRegressionTolerances.kLongitudinalErrorHeadroomMeters + "m headroom");

        // Every in-scope auto runs "Intake Start Sequence" (Superstructure.intakeSequence(5.0))
        // inside a "parallel" block -- this asserts the new end-reason telemetry from Task 1
        // actually reaches the wpilog in a real run, not just that it compiles.
        assertTrue(
                WpilogEndReasonReader.wasLogged(wpilog, "/RealOutputs/Superstructure/IntakeSequenceEndReason"),
                () -> autoName() + ": expected Superstructure/IntakeSequenceEndReason to be logged "
                        + "at least once -- is intakeSequence() still wired into this auto?");
    }
```

- [ ] **Step 2: Run to verify it fails**

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew test --tests "frc.robot.auto.LtNeutralAutoRegressionTest"
```
Expected: FAIL to compile — `WpilogEndReasonReader` doesn't exist yet.

- [ ] **Step 3: Implement `WpilogEndReasonReader.java`**

Create `src/test/java/frc/robot/auto/WpilogEndReasonReader.java`, mirroring `WpilogTrajectoryErrorReader`'s established `DataLogReader`/`DataLogRecord` pattern:

```java
package frc.robot.auto;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

/**
 * Scans a .wpilog for whether a given string entry was ever logged, at least once. Deliberately
 * minimal compared to WpilogTrajectoryErrorReader -- this only needs presence, not a computed
 * max, to prove Superstructure's end-reason telemetry (Task 1 of the completion-trigger framework
 * plan) actually reaches the log in a real run.
 */
final class WpilogEndReasonReader {
    private static final String kStringType = "string";

    private WpilogEndReasonReader() {}

    static boolean wasLogged(Path wpilogFile, String entryName) throws IOException {
        DataLogReader reader = new DataLogReader(wpilogFile.toString());
        if (!reader.isValid()) {
            throw new IOException("Not a valid WPILOG file: " + wpilogFile);
        }

        Map<Integer, String> activeNames = new HashMap<>();
        Map<Integer, String> activeTypes = new HashMap<>();

        for (DataLogRecord record : reader) {
            if (record.isStart()) {
                DataLogRecord.StartRecordData start = record.getStartData();
                activeNames.put(start.entry, start.name);
                activeTypes.put(start.entry, start.type);
                continue;
            }
            if (record.isFinish() || record.isSetMetadata()) {
                continue;
            }

            String name = activeNames.get(record.getEntry());
            String type = activeTypes.get(record.getEntry());
            if (entryName.equals(name) && kStringType.equals(type)) {
                return true;
            }
        }

        return false;
    }
}
```

- [ ] **Step 4: Compile and run**

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew compileJava
./gradlew test --tests "frc.robot.auto.LtNeutralAutoRegressionTest"
```
Expected: `BUILD SUCCESSFUL`, PASS.

- [ ] **Step 5: Run the full test suite one final time**

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew test
```
Expected: every test class passes — this is the plan's final full-suite gate.

- [ ] **Step 6: Commit**

```bash
git add src/test/java/frc/robot/auto/WpilogEndReasonReader.java src/test/java/frc/robot/auto/AutoRegressionTestBase.java
git commit -m "Assert Superstructure end-reason telemetry reaches the wpilog in the regression suite"
```

---

## Final Verification

- [ ] `./gradlew compileJava` — `BUILD SUCCESSFUL`
- [ ] `python SKILLS/run_headless_sim.py --run-seconds 12` — PASS
- [ ] `./gradlew test` — all classes PASS, including the pre-existing `LtNeutralAutoRegressionTest` golden (unchanged numbers) and the four new/modified test classes from this plan
- [ ] `python SKILLS/parse_akit_log.py <newest .wpilog> --grep EndReason` — confirms both `Superstructure/IntakeSequenceEndReason` and `Drive/IsFollowingAutoPath` are present in a real run's log
