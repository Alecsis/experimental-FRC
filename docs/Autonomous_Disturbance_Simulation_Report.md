# Autonomous Disturbance Simulation Report

**Date:** 2026-07-28
**Follows:** `docs/Autonomous_Recovery_Audit.md` (F4, F5, F7, F8/F9 referenced throughout)
**Scope:** Evidence collection only. No production code was modified. No recovery behavior was
implemented. Drivetrain velocity architecture, SysId, Slot0 gains, `DriveRequestType`, and
Controls Academy work were untouched, as were the autonomous-completion-trigger-framework
worktree/branch.

## Objective

Determine, with real simulation evidence rather than code-reading alone, how this codebase's
autonomous stack actually behaves when the robot is physically displaced mid-path (simulating
being pushed by another robot) — and use that evidence to validate or challenge three specific
Recovery Audit findings: **F4** (tracking-error telemetry has no live consumer), **F5** (vision's
innovation gate has no backoff/recovery path), and **F8/F9** (PathPlanner ships unused
pathfinding/replanning primitives).

## Test setup

### Why a single path, not a full `.auto`

The obvious choice was to reuse the existing `AutoRegressionTestBase`/`LtNeutralAutoRegressionTest`
harness and inject a disturbance into a real registered auto (e.g. "LT Neutral"). This was
rejected: `LT_Neutral.json`'s own checked-in golden already records `completed: false` and
`maxLateralErrorMeters: 7.01` **with zero disturbance** — a real, already-documented,
already-out-of-scope chassis-PID divergence bug (unclamped `kP=5` feedback, see this file's
sibling `CLAUDE.md` "PID retune milestone" history). Injecting a disturbance on top of a baseline
that already diverges by 7 meters would make it impossible to attribute any observed error growth
to the disturbance versus the pre-existing bug.

Instead, a new throwaway JUnit package `src/test/java/frc/robot/recovery/` drives a **single**
PathPlanner path in isolation via `AutoBuilder.followPath(PathPlannerPath.fromPathFile(...))`,
using `"Left Trench Neutral"` (one of the three path segments inside the real "LT Neutral" auto).
This isolates path-following dynamics from `NamedCommand`/intake/shoot complexity, and — critically
— lets a **zero-displacement control run** on the exact same path establish how much error this
codebase already produces with no disturbance at all, so the three real disturbance runs can be
read as deltas against that control, not against an idealized zero baseline.

### Harness (`PathDisturbanceSimTestBase` + 4 concrete subclasses)

Structurally a trimmed copy of `AutoRegressionTestBase`'s boot/teardown dance (`HAL.initialize()`,
`SimHooks.pauseTiming()`/`stepTiming()`/`resumeTiming()`, `DriverStationSim`, real-time polling
loop, newest-`.wpilog` discovery) — same known-correct pattern, no new synchronization was
invented. Each subclass overrides one method, `displacementMeters()`:

| Class | Displacement |
|---|---|
| `NoDisturbanceControlTest` | 0.0 m (control) |
| `SmallDisplacementDisturbanceTest` | 0.4 m |
| `MediumDisplacementDisturbanceTest` | 1.0 m |
| `SevereDisplacementDisturbanceTest` | 2.0 m |

At a fixed elapsed time (t=1.5s into the path, roughly a quarter of the path's ~6.35s nominal
duration, confirmed via the control run before tuning the other three), each test calls:

```java
Pose2d groundTruth = RobotContainer.drivetrain.getSimulatedGroundTruthPose();
Translation2d lateralShove =
        new Translation2d(0, displacementMeters()).rotateBy(groundTruth.getRotation());
Pose2d displaced = new Pose2d(groundTruth.getTranslation().plus(lateralShove), groundTruth.getRotation());
RobotContainer.drivetrain.getMapleSimDrive().setSimulationWorldPose(displaced);
```

This is the key design decision: it calls maple-sim's `AbstractDriveTrainSimulation
.setSimulationWorldPose()` **directly**, bypassing `CommandSwerveDrivetrain.resetPose()` entirely.
`resetPose()` (the method this project's existing sim-practice-spawn code already uses) calls
**both** `super.resetPose()` (which snaps the CTRE pose *estimator*) **and**
`setSimulationWorldPose()` (which snaps the physics body) — that would look like an instant,
perfectly-corrected teleport, not a collision. Calling `setSimulationWorldPose()` alone moves only
the physics body's true position (confirmed from the decompiled maple-sim 0.4.0-beta source,
`AbstractDriveTrainSimulation.java`: *"This method instantly teleports the robot... The robot does
not drive to the new pose; it is moved directly"*), leaving the wheel encoders, gyro, and CTRE pose
estimator completely unaware anything happened — exactly like a real collision, where another
robot's momentum changes the true pose but the drivetrain's own sensors don't "see" the jump. The
shove is applied robot-relative-lateral (rotated by the robot's current heading) to model a
perpendicular "T-bone" hit rather than a push along the direction of travel. Heading itself is left
unperturbed — a deliberate simplification to isolate translational effects as one variable; see
"What this experiment did not do."

No production file was touched to build this. `getMapleSimDrive()` and
`getSimulatedGroundTruthPose()` were already public accessors on `CommandSwerveDrivetrain`
(added in an earlier session for a different purpose) and were reused as-is.

### Telemetry captured

All from the resulting `.wpilog`, via `SKILLS/parse_akit_log.py`'s real WPILOG parser (reused, not
reimplemented) plus a one-off scratchpad script pairing several channels by timestamp:

- `/RealOutputs/Odometry/Robot` — the pose **estimate** (Telemetry.java's publish)
- `/RealOutputs/Trajectory/ErrorLateralMeters` / `ErrorLongitudinalMeters` — `TrajectoryErrorTracker`'s
  live setpoint-vs-estimate error (F4's subject)
- `/RealOutputs/Vision/RejectedJumpMeters` / `AcceptedStdDevMeters` — the innovation gate's own
  per-cycle telemetry (F5's subject)
- `/RealOutputs/Drive/AppliedVoltsPerModule` / `StatorCurrentAmpsPerModule` — per-module drive
  telemetry
- Ground-truth pose was **not** logged via AdvantageKit (adding that would have meant touching
  production `CommandSwerveDrivetrain.periodic()`, which the task rules exclude) — instead the
  test itself prints the ground-truth pose immediately before/after injection to stdout, and the
  exact injected delta is known deterministically from the test's own code, not inferred.
- **Not captured:** `Auto/Running`/`Auto/EndedInterrupted` (Robot.java's autonomous lifecycle
  telemetry). This harness bypasses `Robot.autonomousInit()`/`RobotContainer.getAutonomousCommand()`
  entirely (same reason `AutoRegressionTestBase` does — scheduling `AutoBuilder.buildAuto(...)`
  directly), so that wrapper is never exercised here. The command-lifecycle-during-auto question is
  answered from the existing, already-verified Lifecycle Audit instead of fabricating log evidence
  for a code path this harness doesn't run.

## Results

All four runs used the identical starting pose, identical path, identical injection instant
(t=1.5s), and differed only in `displacementMeters()`.

| Run | Completed | Runtime | Max lateral err (before / after inject) | Max longitudinal err (before / after) | Vision rejections after inject | Vision acceptances after inject |
|---|---|---|---|---|---|---|
| Control (0.0m) | true | 6.35s | 0.67m / **3.10m** | 0.65m / 2.62m | 0 | 0 |
| Small (0.4m) | true | 6.36s | 0.68m / **2.73m** | 0.69m / 2.19m | 0 | 0 |
| Medium (1.0m) | true | 6.35s | 0.69m / **3.47m** | 0.68m / 1.64m | 0 | 0 |
| Severe (2.0m) | true | 6.35s | 0.65m / **0.36m** | 0.66m / 0.71m | **242** (max 3.25m, first at t=1.54s) | 0 |

Full lateral-error time series (sampled every 0.3s from injection onward) for all four runs are in
the raw analysis output (`analyze_disturbance.py`, reproducible from the four `.wpilog` files
listed in "Reproducing this experiment" below).

### Finding 1 (confirms a pre-existing, out-of-scope bug — not new): the undisturbed baseline
### already diverges by ~3m over a single 6.35s path segment

The **control run**, with zero injected disturbance, grows from ~0.03m of tracking error right
after path-start to **3.10m** of lateral error by the end of the same single path segment, with a
visibly oscillatory (not monotonic) error trace — error swings from -1.5m to +1.7m to -3.0m rather
than settling. This is consistent with, and roughly proportional to, the already-documented
whole-auto golden of ~7m over three segments (`LT_Neutral.json`). **This is not a disturbance
effect** — it reproduces on a completely undisturbed run — and confirms this project's own
already-flagged, unclamped `PPHolonomicDriveController` chassis-feedback issue is real, measurable
on a single path segment in isolation, and large enough to swamp a naive "did the disturbance cause
tracking error" comparison if a bare LT Neutral auto had been used instead of this isolated-path
control design.

### Finding 2 (direct evidence for F7, already correct in the Recovery Audit): command
### completion is 100% decoupled from tracking success

All four runs — 0m, 0.4m, 1.0m, and 2.0m of injected displacement — report `completed=true` at
**the same elapsed time (6.35-6.36s)**, regardless of final tracking error (0.36m to 3.47m across
the four runs). This is direct empirical confirmation that `FollowPathCommand.isFinished()` is
purely time-bounded and has no dependency on whether the robot is anywhere near where it should be.
**A 2-meter mid-path collision produces an identical "success" signal to a perfect run.** Anything
downstream that treats "auto command finished" as "auto succeeded" (which, per the Lifecycle Audit,
is exactly what `Robot.wrapAutonomousForTelemetry()`'s `Auto/Running`/`Auto/EndedInterrupted`
telemetry does — it distinguishes interrupted-vs-not, not accurate-vs-not) cannot detect this class
of failure at all.

### Finding 3 (direct evidence for F5, sharpened with real numbers): the vision innovation gate
### has no backoff, and a ≥2m disturbance triggers a sustained, near-total rejection streak

The severe (2.0m) run triggered **242 consecutive `Vision/RejectedJumpMeters` rejections** starting
at t=1.54s (40ms after injection) and continuing for effectively the rest of the ~4.8s remaining
run — out of roughly 318 total scheduler ticks in that window, meaning vision was rejected on
almost literally every tick after the hit. The small (0.4m) and medium (1.0m) runs triggered **zero**
rejections — the 1.0m case landed right at the gate's `Constants.kMaxVisionJumpMeters` threshold and,
as measured against the estimate's own drift at that instant, didn't clear it. This directly
confirms F5's mechanism: once a real disturbance's *effective* jump (vision-measured-pose vs.
current estimate, not the raw injected displacement) exceeds 1.0m, `Vision.fuseMeasurements()`
rejects every correction attempt with **no backoff, no widened tolerance, no "N consecutive
rejections" escalation** — the gate has no way to distinguish "one bad frame" from "a real hit that
needs a bigger correction," and this codebase does not do anything with a rejection streak once it
starts (nothing consumes `Vision/RejectedJumpMeters` at runtime; it is telemetry only, matching what
the Recovery Audit found for `TrajectoryErrorTracker`).

### Finding 4 (a genuine surprise, reported honestly rather than oversold — needs a controlled
### re-test, not a conclusion)

The severe (2.0m) run's tracking error, instead of growing like the other three, **collapsed to
near-zero (≤0.02m) within ~0.5s of injection and stayed there for the rest of the run.** Read in
isolation this would look like "the controller recovered from a severe hit better than smaller
ones" — but that conclusion does not survive scrutiny:

- The pose trace across the injection instant (`analyze_disturbance.py`'s dump) shows the
  **estimate** changing smoothly and continuously through the injection window, with no
  discontinuity — exactly as expected, since only wheel/gyro integration feeds the estimate and the
  physics teleport doesn't touch either. The controller's inputs did not change at the moment of
  injection; only the *ground truth* did.
- Drive voltage/current in the same window stay in a normal actively-driving range (11-120A,
  fluctuating, never pinned at a stall-current plateau), ruling out "the robot got physically stuck
  against a field-boundary collider" as the explanation.
- The most likely mechanism, stated as a hypothesis rather than a proven cause: maple-sim's
  `setSimulationWorldPose()` also unconditionally zeroes the physics body's linear velocity
  (`super.linearVelocity.set(0, 0)`, confirmed from the same decompiled source cited above) — a real
  side effect of *any* teleport-based disturbance injection, not something specific to a 2m
  magnitude. Combined with the control run's own demonstrated chaotic, oscillatory (not smoothly
  convergent or divergent) error behavior, a small perturbation to the system's exact state at
  t=1.5s could plausibly flip which side of that chaotic behavior a given run lands on — i.e. the
  severe run's low final error looks more like this run happened to land on the "good" side of an
  already-unstable controller than like genuine disturbance recovery.
- **This cannot be resolved without fixing the pre-existing chassis-PID divergence bug first**,
  which is explicitly out of scope for this task. Recommendation: re-run this exact experiment
  (harness is fully reusable) once that bug has a real fix, so disturbance-recovery behavior can be
  read against a stable, non-diverging baseline instead of a chaotic one.

## Answers to the assigned questions

**1. When following a trajectory and suddenly displaced:**
- *Does PathPlanner continue following the original trajectory?* Yes — confirmed directly:
  `FollowPathCommand`'s setpoint stream is unaffected by the disturbance at the moment it happens
  (matches the already-established fact that it samples a pre-planned, time-indexed trajectory, not
  a live replan against current pose mid-execution).
- *Does it naturally recover?* Not established either way by this experiment — see Finding 4. Small
  and medium displacements show no clear recovery signal distinguishable from the control run's own
  baseline noise; the severe run's apparent recovery is confounded by the pre-existing bug and a
  velocity-reset side effect, not proven disturbance tolerance.
- *Does tracking error grow indefinitely?* In the **undisturbed control run**, yes, within a single
  6.35s path segment (Finding 1) — a pre-existing, already-documented, out-of-scope issue. Whether a
  disturbance makes this materially worse could not be cleanly isolated given that confound.
- *Does the command finish successfully despite failure?* Yes, definitively (Finding 2) — identical
  `completed=true` at identical elapsed time across all four magnitudes.

**2. Disturbance magnitudes measured:** 0.4m, 1.0m, 2.0m (plus a 0.0m control), all run and logged;
see the Results table.

**3. Telemetry recorded:** pose (estimate), `TrajectoryErrorTracker` outputs, vision accepted/rejected
counts, drive voltage/current — all captured per run. Planned-trajectory pose and `Auto/*` lifecycle
telemetry were **not** capturable with this harness design (see "Telemetry captured" above); this is
disclosed, not glossed over.

**4. Is current behavior acceptable? Is failure detected? Is recovery possible today? What would an
`AutonomousHealthMonitor` need?**
- **Not acceptable.** Three concrete, evidence-backed problems: silent unbounded-looking tracking
  error even without a disturbance (Finding 1); a disturbance-triggered vision lockout with no
  recovery path (Finding 3); and a completion signal that cannot distinguish a perfect run from a
  2-meter collision (Finding 2).
- **Failure is not detected live**, by anything. Every one of these findings required this
  experiment's own post-hoc log analysis to surface — nothing in the running robot code reacts to
  any of it in real time. This matches, and now has concrete numbers behind, the Recovery Audit's F4
  and F5.
- **Partial recovery capability exists in PathPlanner today, unused** (F8/F9, previously confirmed
  via decompiled source, not re-verified this session): `AutoBuilder.pathfindToPose`/
  `pathfindThenFollowPath` with a real, populated `navgrid.json` already deployed. Nothing in this
  experiment required touching that, but nothing in this codebase invokes it either.
- **An `AutonomousHealthMonitor` would minimally need:** (a) a live consumer of
  `TrajectoryErrorTracker.getLateralErrorMeters()`/`getLongitudinalErrorMeters()` against a
  threshold (F4's fix, still not implemented); (b) a live consecutive-rejection counter fed by
  `Vision.fuseMeasurements()`'s existing (but unconsumed) rejection signal, since this experiment
  showed a real disturbance produces a long, unambiguous rejection *streak*, not just one bad frame
  (F5's fix); (c) a trigger, once either crosses a threshold, to invoke `AutoBuilder.pathfindToPose`
  back onto the original path's remaining trajectory (F8/F9's unused capability) instead of letting
  `FollowPathCommand` run out its clock against a stale plan.

## What this experiment did not do

- Did not fix, retune, or otherwise touch the pre-existing chassis-PID divergence bug that Finding 1
  and Finding 4 both depend on — explicitly out of scope, and doing so would have changed the very
  baseline this experiment was measuring against.
- Did not perturb heading, only translation — isolates one variable, but a real collision often
  imparts angular disturbance too; not modeled here.
- Did not run multiple trials per magnitude — maple-sim/dyn4j is deterministic given a fixed
  timestep and no injected noise, so repeat runs of the identical scenario should reproduce
  identically, but this was not independently re-verified by re-running any of the four tests twice.
- Did not exercise `Robot.autonomousInit()`'s `Auto/Running`/`Auto/EndedInterrupted` telemetry (see
  "Telemetry captured").
- Did not implement any recovery behavior, `AutonomousHealthMonitor`, or wiring of
  `TrajectoryErrorTracker`'s getters — per the task's explicit instruction, and because no finding
  here rose to a bug requiring immediate correction (the closest candidate, Finding 1, is already a
  known, separately-tracked issue, not a new one).

## Cleanup instructions

This entire investigation is isolated to files that can be deleted without touching any production
code:

- `src/test/java/frc/robot/recovery/` (5 files: `PathDisturbanceSimTestBase.java` +
  `NoDisturbanceControlTest.java` + `SmallDisplacementDisturbanceTest.java` +
  `MediumDisplacementDisturbanceTest.java` + `SevereDisplacementDisturbanceTest.java`) — delete the
  whole package once this report is no longer needed as a live reference, or keep it if a future
  session wants to re-run the experiment after the chassis-PID bug is fixed (see Finding 4).
- The four `.wpilog` files this experiment produced (`logs/akit_26-07-28_16-03-47.wpilog` through
  `akit_26-07-28_16-04-41.wpilog`) are ordinary gitignored sim-run logs, safe to delete like any
  other local sim log.
- The scratchpad analysis script used to produce the Results table lives outside the repo (session
  scratchpad directory), not checked in.

## Reproducing this experiment

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew test --tests "frc.robot.recovery.NoDisturbanceControlTest"
./gradlew test --tests "frc.robot.recovery.SmallDisplacementDisturbanceTest"
./gradlew test --tests "frc.robot.recovery.MediumDisplacementDisturbanceTest"
./gradlew test --tests "frc.robot.recovery.SevereDisplacementDisturbanceTest"
```

Each writes a fresh `.wpilog` to `logs/`. `python SKILLS/parse_akit_log.py <log> --grep Trajectory`
(or `--grep Vision`, `--dump <entry>`) inspects any single channel directly; the paired,
timestamp-aligned cross-channel comparison used for this report's tables re-imports
`SKILLS/parse_akit_log.py`'s `parse_log`/`decode_scalar`/`decode_pose` functions rather than
re-parsing the binary format by hand.
