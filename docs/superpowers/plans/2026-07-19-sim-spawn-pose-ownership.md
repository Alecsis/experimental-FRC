# Sim Spawn Pose Ownership — Regression Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Lock in the already-applied `maybeApplySimPracticeSpawn()` fix with a JUnit regression test that fails against the old guard, plus an interactive sim validation pass by the mentor.

**Architecture:** The production fix is **already written and compiling** in `CommandSwerveDrivetrain.java` (see "Already Done" below). This plan adds the missing automated regression test — a `DriverStationSim`-driven test that attaches the DS and enables into autonomous in the same tick (the exact trigger), then asserts `Drivetrain/SimSpawnPose` never fires while enabled. Then a manual interactive validation gate the mentor runs.

**Tech Stack:** Java 17 (WPILib JDK at `C:/Users/Public/wpilib/2026/jdk`), Gradle 8.11, JUnit 5, WPILib `DriverStationSim`/`SimHooks`, AdvantageKit, CTRE Phoenix 6, MapleSim.

## Global Constraints

- `export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"` before ANY gradle command. PATH `java` is JDK 25 and Gradle 8.11 cannot run it — every task dies during configuration with `Could not create task ':test'`.
- Commit style: short caveman-style one-line messages, **no** `Co-Authored-By` trailer.
- Surgical changes only. Do NOT touch `TunerConstants.java` Slot0 gains, `PPHolonomicDriveController` PID, or anything drivetrain-tuning related. That work is explicitly deferred until this plan passes.
- Do NOT edit anything under `temp_reference/`.
- Never claim a gate passed without running it. A gate that can't run is reported UNVERIFIED.

---

## Already Done (do not redo — verify only)

Applied to `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java` this session:

1. New field near line 88, beside `m_hasAppliedOperatorPerspective`:
   ```java
   private boolean m_hasAppliedSimPracticeSpawn = false;
   ```
2. The sim-spawn block was **removed** from inside the `if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled())` guard in `periodic()`. That guard now only applies operator perspective. A single unconditional call `maybeApplySimPracticeSpawn();` was added immediately after it.
3. New private method added directly above `startSimThread()`:
   ```java
   private void maybeApplySimPracticeSpawn() {
       if (mapleSim == null || m_hasAppliedSimPracticeSpawn || !DriverStation.isDisabled()) {
           return;
       }
       DriverStation.getAlliance().ifPresent(allianceColor -> {
           Pose2d spawnPose = FieldConstants.simPracticeSpawn(allianceColor);
           resetPose(spawnPose);
           m_hasAppliedSimPracticeSpawn = true;
           Logger.recordOutput("Drivetrain/SimSpawnPose", spawnPose);
       });
   }
   ```

Verification already run and passed: `./gradlew compileJava` BUILD SUCCESSFUL; `python SKILLS/run_headless_sim.py --run-seconds 12` PASS; `./gradlew test` 4/4 passed, 0 failures.

**Known gap this plan closes:** none of those three gates attaches a Driver Station, so none of them actually exercises the bug path. They prove no regression, not that the fix works.

### Root cause, for the record

From `logs/akit_26-07-19_00-52-32.wpilog` (the run in the mentor's screenshot):

```
/DriverStation/DSAttached        t=0.399 False -> t=3.882 True
/DriverStation/Enabled           t=0.399 False -> t=3.882 True
/DriverStation/Autonomous        t=0.399 False -> t=3.882 True
/RealOutputs/Drivetrain/SimSpawnPose   1 record, t=3.882  <- fired while ENABLED in AUTO

Odometry/Robot:
  t=3.8614s  ( 0.2077, -0.4658, -14.05 deg)   idle, pre-enable
  t=3.8818s  (12.8865,  0.5700, -90.00 deg)   AutoBuilder seeds path start
  t=3.9015s  ( 8.2590,  4.0215,  -0.60 deg)   simPracticeSpawn stomps it
```

Max frame-to-frame pose delta 12.72 m at t=3.88s. The DS attached at the same instant it enabled into auto, so `getAlliance()` was empty for the whole disabled period, so `m_hasAppliedOperatorPerspective` stayed `false`, so the left half of the `||` was still true on the first enabled tick.

---

## File Structure

- **Create:** `src/test/java/frc/robot/SimSpawnPoseOwnershipTest.java` — the regression test. One responsibility: prove the sim practice spawn never resets pose while enabled.
- **Modify:** `CLAUDE.md` — Current Task State, replacing the "Mid-field spawn-reset guard bug" Next-up item.
- **Modify:** `docs/claudex/history.md` — append the fifteenth-session entry.
- **Reference only (read, do not edit):** `src/test/java/frc/robot/RobotLifecycleTest.java` — the existing `DriverStationSim` harness pattern to copy.

---

### Task 1: Regression test for spawn-while-enabled

**Files:**
- Create: `src/test/java/frc/robot/SimSpawnPoseOwnershipTest.java`
- Read first (do not modify): `src/test/java/frc/robot/RobotLifecycleTest.java`
- Under test: `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`

**Interfaces:**
- Consumes: `CommandSwerveDrivetrain.getInstance()` (singleton per Architecture Rule 2), `CommandSwerveDrivetrain.periodic()`, `CommandSwerveDrivetrain.getState().Pose`, `CommandSwerveDrivetrain.resetPose(Pose2d)`, `FieldConstants.simPracticeSpawn(Alliance)`.
- Produces: nothing consumed by later tasks. This is a leaf test.

- [ ] **Step 1: Read the existing harness pattern**

Read `src/test/java/frc/robot/RobotLifecycleTest.java` in full. Copy its `@BeforeEach`/`@AfterEach` structure verbatim — specifically how it calls `HAL.initialize(500, 0)`, how it drives `DriverStationSim`, and how it tears down. Do NOT invent a new harness shape; match what is already there.

Note the accessor for the drivetrain singleton as it actually appears in that file. If `CommandSwerveDrivetrain.getInstance()` is not the real accessor name, use whatever `RobotLifecycleTest` uses and adjust every snippet below accordingly.

- [ ] **Step 2: Write the failing test**

Create `src/test/java/frc/robot/SimSpawnPoseOwnershipTest.java`:

```java
package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Regression guard for the mid-auto teleport fixed 2026-07-19.
 *
 * <p>The DS can attach at the same instant it enables into autonomous. When that happened, the
 * alliance resolved for the first time on an ENABLED tick, and the sim practice spawn — then
 * sharing the operator-perspective guard — reset the pose one loop after AutoBuilder had already
 * seeded the path start, teleporting the robot to midfield. Evidence:
 * logs/akit_26-07-19_00-52-32.wpilog, pose steps (12.887, 0.570) -> (8.259, 4.022) across
 * t=3.88s -> 3.90s with Enabled and Autonomous both true.
 */
public class SimSpawnPoseOwnershipTest {

  /** Stand-in for a pose AutoBuilder would have seeded; far from simPracticeSpawn's midfield. */
  private static final Pose2d PATH_START =
      new Pose2d(12.8865, 0.5700, Rotation2d.fromDegrees(-90.0));

  /** simPracticeSpawn sits at field centre; anything within this of it means the spawn fired. */
  private static final double SPAWN_MATCH_TOLERANCE_METERS = 0.25;

  private CommandSwerveDrivetrain drivetrain;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0);
    // Cold DS: not attached, no alliance. This is the state that reproduced the bug.
    DriverStationSim.setDsAttached(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    drivetrain = CommandSwerveDrivetrain.getInstance();
  }

  @AfterEach
  public void teardown() {
    DriverStationSim.setDsAttached(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  public void spawnDoesNotStompPathStartPoseWhenDsAttachesAtAutoEnable() {
    // Disabled with no DS attached: alliance is unknown, so the spawn must not have latched.
    for (int i = 0; i < 5; i++) {
      drivetrain.periodic();
    }

    // Stand in for autonomousInit() -> AutoBuilder.resetOdom(pathStart).
    drivetrain.resetPose(PATH_START);

    // The trigger: DS attaches, alliance resolves, and we enable into auto all at once.
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(
        edu.wpi.first.hal.AllianceStationID.Blue1);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();

    for (int i = 0; i < 10; i++) {
      drivetrain.periodic();
    }

    Pose2d spawn = FieldConstants.simPracticeSpawn(Alliance.Blue);
    Pose2d actual = drivetrain.getState().Pose;
    double distanceToSpawn = actual.getTranslation().getDistance(spawn.getTranslation());

    assertTrue(
        distanceToSpawn > SPAWN_MATCH_TOLERANCE_METERS,
        () ->
            "Pose was teleported to simPracticeSpawn while enabled in autonomous. Expected to stay"
                + " near "
                + PATH_START
                + " but got "
                + actual
                + " (distance to spawn "
                + distanceToSpawn
                + " m). The sim practice spawn must never reset pose while enabled.");
  }
}
```

- [ ] **Step 3: Run the test — expect it to PASS against the fixed code**

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew test --tests "frc.robot.SimSpawnPoseOwnershipTest" 2>&1 | tail -20
```

Expected: PASS.

**If it ERRORS rather than passing or failing** (compile error, `getInstance()` not found, `AllianceStationID` import wrong, NPE because `mapleSim` is null in the JUnit harness): that is a harness problem, not a fix problem. Fix the harness. In particular, if `mapleSim` is null under JUnit then `maybeApplySimPracticeSpawn()` returns early and the test is vacuous — see Step 4, which exists specifically to catch that.

- [ ] **Step 4: Prove the test is not vacuous — temporarily revert the fix**

A passing test means nothing until you've seen it fail for the right reason. Temporarily restore the old buggy shape in `CommandSwerveDrivetrain.java`: move the spawn block back inside the `if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled())` guard, keyed off the `allianceColor != m_lastAppliedAlliance` change, exactly as it was.

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew test --tests "frc.robot.SimSpawnPoseOwnershipTest" 2>&1 | tail -25
```

Expected: **FAIL**, with the assertion message showing a pose near (8.26, 4.03).

If it still PASSES with the fix reverted, the test does not reproduce the bug — most likely `mapleSim` is null under JUnit so the spawn path never runs at all. In that case **stop and report the test as UNVERIFIED**; do not keep a green test that proves nothing. Escalate to the mentor: the honest fallback is that this bug is only reachable in interactive sim, and Task 2's manual procedure is the real gate.

- [ ] **Step 5: Restore the fix**

Restore `CommandSwerveDrivetrain.java` to the fixed shape documented in "Already Done" above.

```bash
git diff src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
```

Expected: the diff shows the fix (new field, extracted `maybeApplySimPracticeSpawn()`, spawn block removed from the perspective guard) and nothing else.

- [ ] **Step 6: Run the full gate set**

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew compileJava 2>&1 | tail -5
./gradlew test 2>&1 | tail -10
python SKILLS/run_headless_sim.py --run-seconds 12 2>&1 | tail -8
```

Expected: BUILD SUCCESSFUL; 5 tests, 0 failures (the 4 existing plus the new one); headless sim PASS.

Confirm the count with:

```bash
python -c "
import glob,xml.etree.ElementTree as ET
t=f=0
for p in glob.glob('build/test-results/test/*.xml'):
    r=ET.parse(p).getroot(); t+=int(r.get('tests')); f+=int(r.get('failures'))+int(r.get('errors'))
    print(f\"{r.get('name'):55s} tests={r.get('tests')} fail={r.get('failures')} err={r.get('errors')}\")
print(f'TOTAL tests={t} failures/errors={f}')"
```

- [ ] **Step 7: Commit**

```bash
git add src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java src/test/java/frc/robot/SimSpawnPoseOwnershipTest.java
git commit -m "sim spawn no longer stomps auto start pose"
```

---

### Task 2: Interactive sim validation (mentor-run)

**Files:** none modified. This task produces a verdict, not code.

**Interfaces:**
- Consumes: the fix from Task 1.
- Produces: a GO/NO-GO for starting drivetrain characterization work.

This gate **cannot be run by an agent** — it needs a human driving the sim GUI and AdvantageScope. Do not mark it complete on the agent's own authority. Present the procedure to the mentor and wait.

- [ ] **Step 1: Confirm the practice spawn still works**

Run `./gradlew simulateJava`. Leave the DS **disabled**. Confirm the robot spawns at midfield, approximately (8.26, 4.03).

Expected: spawn works. If the robot sits at (0,0), the fix over-corrected — the one-shot latched before the alliance resolved.

- [ ] **Step 2: Normal auto**

Select an auto, enable Autonomous. Verify the pose snaps to the PathPlanner start pose and **never** returns to (8.26, 4.03).

- [ ] **Step 3: The actual regression case**

Restart the sim. Enable straight into Autonomous **without** letting the DS settle while disabled. This is the exact sequence that produced the original failure.

Expected: robot starts at the path start pose. No midfield teleport.

- [ ] **Step 4: Log-level confirmation**

In AdvantageScope, open `Drivetrain/SimSpawnPose`. It must have **zero** records timestamped while `DriverStation/Enabled` is true.

Then run the pose-jump analysis on the new log:

```bash
python SKILLS/parse_akit_log.py logs/<newest>.wpilog --pose-entry "/RealOutputs/Odometry/Robot"
```

Expected: no multi-metre jump at the auto-enable instant. Small jumps elsewhere from vision fusion are expected and acceptable — the specific thing that must be gone is a ~5-13 m step coincident with the enable.

- [ ] **Step 5: Multiple autos**

Run 2-3 different autos. Confirm `Trajectory/ErrorLongitudinalMeters` no longer opens at multiple metres.

- [ ] **Step 6: Record the verdict**

Only once Step 4 is clean are `Trajectory/ErrorLateralMeters` / `ErrorLongitudinalMeters` meaningful. Report GO or NO-GO explicitly.

---

### Task 3: Documentation sync

**Files:**
- Modify: `CLAUDE.md` (Current Task State)
- Modify: `docs/claudex/history.md` (append)

Do this only after Task 2 returns GO.

- [ ] **Step 1: Update `CLAUDE.md`**

Replace the "Mid-field spawn-reset guard bug — confirmed real, not yet fixed" bullet under **Next up** with a fixed-and-verified entry naming `maybeApplySimPracticeSpawn()`, `SimSpawnPoseOwnershipTest`, and the interactive validation result.

Then unblock the two items that were explicitly gated on this fix:
- The **PathPlanner auto regression suite** item — remove the "HOLD until the spawn-reset bug above is fixed" blocker, keeping the rest of the scope note intact.
- The **PID retune milestone** item — the recommended fix order was "(1) fix the spawn-reset bug first; (2) SysId; (3) drive Slot0 kP; (4) chassis PID". Mark step (1) done and note that SysId is now the next action.

Refresh the verified-tree line: fifteenth session, note that `CommandSwerveDrivetrain.java` and the new test are now committed while the other carried-over files remain uncommitted.

- [ ] **Step 2: Append to `docs/claudex/history.md`**

Fifteenth-session entry. Include: the root-cause evidence block from this plan verbatim (log path, DS transitions, the three-line pose trace, the 12.72 m jump), the Option C + A fix rationale, why Option B alone was rejected (alliance is genuinely unknown at `simulationInit()`, so it must poll), and the gate results.

- [ ] **Step 3: Commit**

```bash
git add CLAUDE.md docs/claudex/history.md docs/superpowers/plans/2026-07-19-sim-spawn-pose-ownership.md
git commit -m "docs for spawn pose fix"
```

---

## Known Behaviour Changes (flag to mentor, do not silently accept)

1. **Cold-DS teleop starts at (0,0).** If the alliance first resolves while *enabled*, the practice spawn never applies — the robot stays at its construction pose. Correct for auto; mildly annoying if you enable straight into teleop from a cold DS. Disable-then-enable fixes it. This was judged preferable to any chance of a mid-auto teleport. If the mentor disagrees, the alternative is to also allow the spawn on the first disabled tick after teleop ends.
2. **One-shot per robot-code lifetime.** Re-spawning now needs a sim restart, not just a disable. Previously an alliance *change* re-triggered it.
3. **No hardware risk.** The whole path is `mapleSim != null` gated, and `mapleSim` is null on real hardware.

## Out of Scope

Explicitly deferred until Task 2 returns GO — do not start these:
- SysId / Phoenix Tuner characterization of the drive motors.
- Retuning `TunerConstants.java` Slot0 (`kP=0.1, kS=0.1, kV=0.124, kA=0` — still unedited CTRE template values).
- Retuning `PPHolonomicDriveController` (`kP=5` translation, `kP=3` rotation).
- Expanding the auto regression suite to the remaining 10 autos.

Incorrect starting poses invalidate controller characterization. That is the whole reason this plan exists.
