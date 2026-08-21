# Trajectory Error Instrumentation — Technical Specification

**Milestone 1 of the autos reliability roadmap.** Status: SPEC ONLY, no code written.
Author: drafted 2026-07-18 against commit `3cd75a3`, branch `refactor/hybrid`.

## 0. Why this exists

`CommandSwerveDrivetrain.java:311-315` carries untuned Tuner-X template gains
(`PIDConstants(5,0,0)` / `(3,0,0)`, zero D). The same values were present at both real
competition checkpoints in `C:\Users\xdm\frc-2026\2026` (`Week-8`, `NCCMP`). We cannot tune
them, and cannot prove a tune worked, because **nothing in this codebase measures how far the
robot is from where the path said it should be.** Every later milestone (collision detection,
replay regression) depends on this number existing first.

Non-goal: this milestone does not change robot behavior. It is read-only observability.

---

## 1. Where trajectory error should be computed

### 1.1 The seam already exists — do not add a new one

`configureAutoBuilder()` (`CommandSwerveDrivetrain.java:326-336`) already registers both
`PathPlannerLogging` static callbacks. PathPlanner **pushes** the active path and the
per-loop target pose to us. We never hold a `FollowPathCommand`, never subclass
`PPHolonomicDriveController`, never import a PathPlanner command type outside the file that
already imports them. This is the integration point. Section 5 expands on why.

### 1.2 Single computation site

Error is computed in **exactly one place**: a new `utility/TrajectoryErrorTracker.java`.

Rejected alternatives:

| Option | Why rejected |
|---|---|
| Inside `Telemetry.telemeterize` | Runs on CTRE's 250 Hz odometry thread. `Logger.recordOutput` off the main loop is not the AdvantageKit contract, and 250 Hz of trajectory error is 5x the data for zero extra information — the setpoint only updates at 50 Hz. |
| Inside `CommandSwerveDrivetrain.periodic()` | Grows the sanctioned CTRE-generated exception file. Architecture Rule 1 tolerates that file; it does not invite additions. |
| A new `Subsystem` | Would take a scheduler requirement it does not need, and could deadlock against drive requirements. Observability must never be able to interrupt a path. |
| Inside `Superstructure.periodic()` | Rule 3 scopes `Superstructure` to multi-subsystem *arbitration*. Trajectory error is not a state decision. |
| Extend a `*IOInputs` `@AutoLog` record | Drive has no IO layer (sanctioned exception). Nothing to extend. |

### 1.3 Class shape

`TrajectoryErrorTracker` is a **plain singleton, not a `SubsystemBase`** — consistent with
`HubActiveState` (`utility/HubActiveState.java`), which is already driven the same way from
`Robot.robotPeriodic()`.

- `getInstance()` per Architecture Rule 2's spirit.
- Two package-visible ingest methods, called only from the two `PathPlannerLogging` lambdas:
  `onActivePath(List<Pose2d>)`, `onTargetPose(Pose2d)`. These only **store**; they compute
  nothing and log nothing.
- One `periodic()`, called from `RobotContainer.periodic()` (already invoked at
  `Robot.java:92`, 50 Hz, before `CommandScheduler.run()` — see §7.9 for the ordering
  consequence). All computation and all `recordOutput` calls live here.
- Reads measured pose via a `Supplier<Pose2d>` handed in at construction
  (`() -> drivetrain.getState().Pose`) — the tracker does not import `CommandSwerveDrivetrain`,
  so it stays unit-testable without a drivetrain.

Ingest-stores-only / periodic-computes is deliberate: the callbacks fire from inside
PathPlanner's command execution, i.e. from within `CommandScheduler.run()`. Keeping them
side-effect-free means log output can never interleave mid-scheduler and every topic in a
given AdvantageKit cycle carries a consistent timestamp.

---

## 2. What is computed

### 2.1 Field-relative error

```
errorField = setpointPose.getTranslation() - measuredPose.getTranslation()
errorRotation = (setpointPose.getRotation() - measuredPose.getRotation())  // wrapped to (-pi, pi]
```

Rotation error MUST be angle-wrapped. `Rotation2d.minus` already returns a wrapped result;
never subtract raw radians.

### 2.2 Path-relative decomposition (the diagnostic that matters)

Field-relative X/Y error is nearly useless for tuning — its meaning rotates with the path.
Decompose into the path frame:

- **Longitudinal error** (along direction of travel): positive = robot is *behind* the
  setpoint. Diagnoses feedforward deficit, brownout, or a mechanically slow drivetrain.
- **Lateral error** (perpendicular): diagnoses the translation PID losing the line, wheel
  slip, or a bad odometry heading.

Two different failures, two different fixes, indistinguishable in a scalar norm.

**Deriving the path direction.** `setLogTargetPoseCallback` provides only a `Pose2d`. Its
rotation is the *robot heading*, which on a swerve is decoupled from direction of travel — it
is NOT the path tangent. Derive the tangent by finite-differencing consecutive setpoints:

```
delta = setpointPose_k.getTranslation() - setpointPose_{k-1}.getTranslation()
if (delta.getNorm() < kTangentEpsilonMeters)  // 1e-4 m ~= 5 mm/s at 50 Hz
    -> hold the last valid tangent; if none exists, publish decomposition as NaN
else
    tangent = delta.getAngle()
```

The epsilon guard is required: at path start, at path end, and during any rotate-in-place
segment, `delta` is numerically zero and its angle is garbage. Holding the last valid tangent
keeps the decomposition meaningful through a stationary rotation; NaN (not 0.0) is published
when no tangent has ever been valid, so AdvantageScope shows a gap rather than a plausible lie.

### 2.3 Per-path aggregates

Reset on every new path (§7.5), accumulated each loop while following:

- peak translation error, peak rotation error — the worst moment
- RMS translation error — sustained quality, insensitive to a single vision snap
- elapsed following duration
- loop count (denominator; also detects a path that never ran)

### 2.4 Latched endpoint summary

The single number that answers "did the auto work": translation and rotation error at the
**last loop the path was active**, latched and held after the path ends so it is readable in
disabled and greppable in a log. Paired with an exit reason (§7.4).

---

## 3. Logged topics

Root namespace: **`Trajectory/`**. The existing `Odometry/Trajectory` and
`Odometry/TrajectorySetpoint` topics stay exactly where they are — AdvantageScope layouts and
the `Odometry/Robot` 2D-field pairing already reference them, and renaming them is churn with
a cost and no benefit.

All units SI, all angles **radians**, per AdvantageKit convention. No degrees, no feet, no
mixed units anywhere in this namespace. (`Vision.java:265`'s `Hub Distance` string in feet is
a human-readable dashboard nicety and is not a precedent for this table.)

| Topic | Type | Unit | Notes |
|---|---|---|---|
| `Trajectory/Following` | boolean | — | true iff a fresh setpoint arrived this loop (§7.3) |
| `Trajectory/SetpointPose` | `Pose2d` | m, rad | mirrors what the controller was aiming at |
| `Trajectory/MeasuredPose` | `Pose2d` | m, rad | snapshot at the same instant — pairing them here removes any doubt about which odometry sample the error was taken against |
| `Trajectory/ErrorTranslationMeters` | double | m | `errorField` norm |
| `Trajectory/ErrorRotationRadians` | double | rad | wrapped |
| `Trajectory/ErrorLongitudinalMeters` | double | m | +ve = robot behind setpoint |
| `Trajectory/ErrorLateralMeters` | double | m | +ve = robot left of path |
| `Trajectory/ErrorFieldX` , `ErrorFieldY` | double | m | raw components; kept for cross-checking the decomposition |
| `Trajectory/PeakErrorTranslationMeters` | double | m | per-path running max |
| `Trajectory/PeakErrorRotationRadians` | double | rad | per-path running max |
| `Trajectory/RmsErrorTranslationMeters` | double | m | per-path |
| `Trajectory/ElapsedSeconds` | double | s | per-path |
| `Trajectory/PathIndex` | int | — | increments per activePath callback; identity without needing path names (§9.1) |
| `Trajectory/ReplanCount` | int | — | activePath callbacks within one following episode minus one (§7.6) |
| `Trajectory/EndpointErrorTranslationMeters` | double | m | latched, survives into disabled |
| `Trajectory/EndpointErrorRotationRadians` | double | rad | latched |
| `Trajectory/ExitReason` | String | — | `NONE` / `COMPLETED` / `INTERRUPTED` / `DISABLED` (§7.4) |
| `Trajectory/PoseJumpMeters` | double | m | measured-pose delta this loop; flags vision snaps (§7.7) |
| `Trajectory/PoseJumpSuspected` | boolean | — | delta exceeds physically possible travel |

**Update frequency: 50 Hz**, one write per topic per main-loop iteration, unconditional.

Unconditional matters. Logging only while following produces sparse topics; AdvantageScope
holds the last value forever and a stale 2 m error from auto renders as a live reading during
teleop. When not following, error/decomposition topics publish `NaN` (a visible gap) and
`Following` publishes false. Latched endpoint topics deliberately keep their values.

Cost estimate: ~20 topics x 8 bytes x 50 Hz ~= 8 KB/s, ~1.2 MB over a full match. Negligible
against existing swerve module logging.

---

## 4. Naming conventions (binding for future milestones)

1. Root segment names the *concern*, not the class (`Trajectory/`, not `TrajectoryErrorTracker/`).
2. `PascalCase` segments, `/` separated — matches `Drive/CancoderBootReadRotations/FL`.
3. Unit suffix on every scalar (`Meters`, `Radians`, `Seconds`). Non-negotiable: `Vision/RejectedJumpMeters` already sets this precedent and it is the reason that topic is unambiguous.
4. Struct types (`Pose2d`) get no unit suffix — the struct schema carries units.
5. Booleans read as assertions (`Following`, `PoseJumpSuspected`), never `IsX` / `HasX`.
6. Enums log as `String` via `.toString()`, matching `Intake/TargetState` at `Intake.java:152`.

Existing topics violating (3) (`Hub Distance`, `Shooter Target RPM`, `Agitator State` — spaces,
no units) are pre-existing and out of scope. Do not fix them in this milestone; note them for a
later naming cleanup.

---

## 5. PathPlanner integration without coupling

**Direction of dependency: PathPlanner -> us. Never us -> PathPlanner.**

`PathPlannerLogging`'s static callbacks are a pure observer hook. Concretely, this design:

- adds **zero** new PathPlanner imports anywhere in the tree;
- adds **two delegating lines** inside the two lambdas already present at
  `CommandSwerveDrivetrain.java:328-336`, both of which already exist and already log;
- never wraps, subclasses, or decorates `PPHolonomicDriveController`, `FollowPathCommand`,
  `AutoBuilder`, or any `PathPlannerAuto`;
- leaves `TrajectoryErrorTracker` itself importing only WPILib geometry + AdvantageKit,
  so it is constructible in a JUnit test with two `Pose2d` suppliers and no vendor stack.

Consequence: if PathPlanner is swapped for Choreo or a hand-rolled follower, only the two
lambdas move. The tracker, the topic schema, and every AdvantageScope layout built on it
survive unchanged. That property is the whole point of doing it this way rather than reading
error out of the controller.

**Explicitly rejected:** calling `PPHolonomicDriveController`'s internal error state, or
subclassing it to intercept `calculateRobotRelativeSpeeds`. Both bind us to PathPlanner
internals that have broken across seasons, and neither yields information the pose pair
does not already contain.

---

## 6. AdvantageScope visualizations

**Layout 1 — "Auto Overview" (the first thing you open after a bad auto).**
2D Field: `Odometry/Robot` (robot), `Trajectory/SetpointPose` (ghost), `Odometry/Trajectory`
(trajectory line). Setpoint vs measured as separate objects makes lag visible as a gap that
travels with the robot. Scrub to the moment the gap opens.

**Layout 2 — "Trajectory Error" (the tuning view).**
Single line graph, shared left axis in meters: `ErrorLongitudinalMeters`,
`ErrorLateralMeters`, plus `ErrorTranslationMeters` for reference. Right axis in radians:
`ErrorRotationRadians`. Discrete overlay: `Following`, `PoseJumpSuspected`.

Reading it: lateral riding high through curves -> translation P too low. Longitudinal
consistently positive and growing with speed -> feedforward/`kS` deficit, not a P problem.
Both spiking at one instant with `PoseJumpSuspected` -> that is vision, not the controller;
ignore that sample.

**Layout 3 — "Auto Scorecard" (post-match, seconds to read).**
Table: `PathIndex`, `EndpointErrorTranslationMeters`, `EndpointErrorRotationRadians`,
`PeakErrorTranslationMeters`, `RmsErrorTranslationMeters`, `ReplanCount`, `ExitReason`.
Sortable; a red row is a path to investigate.

**Layout 4 — cross-check with vision.**
`ErrorTranslationMeters` over `Vision/RejectedJumpMeters` and `Vision/AcceptedStdDevMeters`
(`Vision.java:239-240`). Answers "was the pose lying to the controller?" — the single most
common way a correct controller looks broken.

---

## 7. Edge cases

**7.1 Disabled.** Callbacks stop; no new setpoints. `Following` -> false, live error -> NaN,
latched endpoint values retained. On `disabledInit` the tracker is NOT reset — the whole point
is reading the endpoint error after the match ends.

**7.2 Teleop.** `driveToPOI` (`CommandSwerveDrivetrain.java:373+`) uses raw `SwerveRequest`,
not PathPlanner, so no callbacks fire and the namespace stays inactive. Correct for M1.
Instrumenting `driveToPOI` against the same schema is a natural later extension and the
tracker's supplier-based design allows it without change.

**7.3 Fresh-setpoint detection.** There is no "path started"/"path ended" callback. Following
is inferred: `onTargetPose` sets a dirty flag; `periodic()` reads and clears it. A setpoint
arriving this loop => following. This is exact rather than heuristic **only because of the
ordering in §7.9** — verify that ordering before trusting it.

**7.4 Interrupted paths.** Also unsignalled. On the following->not-following transition, latch
the last error and classify: if the last measured error was within the endpoint tolerance
(`kCompletionToleranceMeters`, default 0.10 m / 0.10 rad — a *reporting* threshold only, it
gates nothing) -> `COMPLETED`; if DS is disabled -> `DISABLED`; otherwise -> `INTERRUPTED`.
This is a heuristic and must be documented as such in the class Javadoc. It is diagnostic
sugar; the raw endpoint error is the ground truth and is logged independently.

**7.5 New path / multi-path autos.** `onActivePath` marks a path boundary: increment
`PathIndex`, reset peak/RMS/elapsed. Without this, a 4-path auto reports one meaningless
peak across all four.

**7.6 Replanning.** PathPlanner replanning fires `onActivePath` *mid-episode*. Distinguish
from a genuine new path by whether the previous loop was already following: if it was,
increment `ReplanCount` and preserve the aggregates (it is the same episode); if it was not,
it is a new path. Treating a replan as a new path silently resets the peak error at exactly
the moment things are going wrong — the single worst failure mode of a naive implementation.

**7.7 Vision corrections.** A MegaTag2 acceptance steps the pose instantly; error jumps
without the controller doing anything wrong. Compute `PoseJumpMeters` = measured-pose delta
per loop and flag `PoseJumpSuspected` when it exceeds `maxSpeed * 0.020 s * 1.5`. This does
not filter anything — filtering would hide real data — it annotates, so a human reading
Layout 2 knows which spikes to discount.

**7.8 REPLAY determinism.** Everything published is a pure function of logged inputs, so
REPLAY reproduces it bit-for-bit — but only if timing comes from `Logger.getTimestamp()`,
never `System.currentTimeMillis()` or a wall clock. Consequence worth stating: this
instrumentation can be added **retroactively to existing `.wpilog` files**, because the poses
it needs are already logged. That is a real capability, not a footnote.

**7.9 Ordering hazard — verify first.** `RobotContainer.periodic()` runs *before*
`CommandScheduler.run()` (`Robot.java:92-99`). The tracker therefore sees the setpoint
produced by the **previous** scheduler iteration, paired against a pose read this loop — a
consistent one-loop (20 ms) skew. At 4 m/s that is 80 mm of phantom longitudinal error,
which is the same order as the real errors being measured. **This must be resolved before
implementation**, by either (a) moving the tracker's `periodic()` call to after
`CommandScheduler.run()` in `robotPeriodic()`, or (b) snapshotting the measured pose inside
`onTargetPose` so both halves come from the same instant. (b) is preferred: it is local to
the tracker and does not depend on call-order in `Robot.java` staying put. This is the
highest-risk detail in the spec — a wrong choice here produces a plausible, consistently
wrong number, which is worse than no number.

**7.10 Empty / single-point active path.** `onActivePath` may deliver an empty list. Guard;
do not index [0] unchecked.

**7.11 Alliance flipping.** `AutoBuilder` flips setpoints for red; `getState().Pose` is
already field-absolute in the same frame. Both sides are consistent — **do not apply any
flip in the tracker.** A second flip here is a silent, red-only, sign-flipped error.

**7.12 Simulation fidelity.** maple-sim models neither wheel slip nor real battery sag, so
sim trajectory error will be optimistically low. Sim proves the *instrumentation* works; it
does not validate gains. Any gain tuned purely from sim numbers is unverified — say so.

---

## 8. Success criteria

Implementation is complete when all of the following hold. Anything unmet is reported
UNVERIFIED per `docs/claudex/verification-loop.md`.

**Gate 1 — `./gradlew compileJava`** zero errors.

**Gate 2 — `python SKILLS/run_headless_sim.py`** passes, and the produced `.wpilog` contains
every topic in §3.

**Gate 2.5 — `./gradlew test`** passes, including new unit tests over
`TrajectoryErrorTracker` in isolation (no drivetrain, no vendor stack):
1. zero error when measured == setpoint;
2. pure-lateral offset -> lateral == offset, longitudinal == 0 (within 1e-6);
3. pure-longitudinal lag -> the converse;
4. rotation error wraps correctly across the +/-pi boundary (e.g. setpoint 179 deg,
   measured -179 deg -> 2 deg, not 358 deg);
5. `onActivePath` mid-episode increments `ReplanCount` and preserves peak (§7.6);
6. `onActivePath` after an idle gap resets aggregates and increments `PathIndex` (§7.5);
7. stationary rotate-in-place holds the last tangent and does not emit a garbage
   decomposition (§2.2);
8. no-tangent-ever emits NaN, not 0.0.

**Gate 3 — `python SKILLS/parse_akit_log.py`** over a sim auto run confirms: `Following`
true for the duration of the path and false outside it; `EndpointErrorTranslationMeters`
latched non-NaN at path end; `PathIndex` equal to the number of paths in the chosen auto.

**Behavioral criteria:**
- B1. Running an existing auto in sim produces a non-trivial, time-varying error trace
  (a flat-zero trace means the setpoint is being compared against itself — a real and easy
  bug to ship).
- B2. Deliberately detuning translation P from 5 to 0.5 produces a visibly larger lateral
  error trace. **This is the acceptance test for the whole milestone** — the instrument must
  demonstrably respond to the thing it exists to measure, or it is decoration.
- B3. Zero measured change in robot behavior: same auto, same path, same endpoint pose within
  sim run-to-run noise.
- B4. Loop-overrun count unchanged from the pre-change baseline.

**Non-criteria (explicitly not claimed by this milestone):** gains are not tuned; real-hardware
behavior is UNVERIFIED until bring-up, per the standing `RobotMotor` precedent.

---

## 9. Deferred — deliberately not in M1

**9.1 Path names.** `PathPlannerLogging` exposes poses, not names. Getting a human-readable
name requires wrapping `AutoBuilder`/`PathPlannerAuto` or registering per-path
`NamedCommands` — i.e. the exact coupling §5 exists to avoid, for a cosmetic gain.
`PathIndex` plus the logged trajectory poses identify a path unambiguously. Revisit only if
reading logs proves genuinely confusing in practice.

**9.2 Alerting / dashboard threshold LEDs.** No consumer yet. `ErrorExceedsThreshold` was
specified and cut (§10).

**9.3 Velocity-error tracking.** Real, and the natural M2 companion to feedforward tuning,
but pose error is the prerequisite and adds no dependency on this.

**9.4 Collision detection, replay regression harness.** Milestones 3+; both consume this
schema.

---

## 10. Self-critique — complexity cut before implementation

Reviewed against "simplest thing that satisfies the requirement" (Karpathy guidelines).

**Cut outright:**
- `ErrorExceedsThreshold` boolean — a threshold with no consumer and an arbitrary constant
  that would get cargo-culted. AdvantageScope draws a threshold line client-side.
- Path-name plumbing (§9.1) — cost is architectural, benefit is cosmetic.
- Per-path RMS was nearly cut too. Kept, because peak error is dominated by single vision
  snaps (§7.7) and RMS is the only aggregate that survives them. One accumulator, two lines.

**Kept but genuinely arguable:**
- **`ErrorFieldX`/`ErrorFieldY` alongside the decomposition.** Redundant — recoverable from
  the pose pair. Kept only because the first thing anyone does with a new decomposition is
  distrust it, and having the raw components adjacent makes that check instant. Defensible to
  cut; two topics.
- **`MeasuredPose`, duplicating `Odometry/Robot`.** Justified only by §7.9: if the pose is
  snapshotted at setpoint time, it is genuinely a *different sample* than `Odometry/Robot` and
  must be logged. If option (a) is chosen instead, this topic is pure duplication and should
  be cut. **The §7.9 decision determines this.**
- **`ExitReason`.** Heuristic, not measurement, and heuristics in an observability layer are
  how people end up debugging the instrument. The raw endpoint error is ground truth. Kept
  because scanning a string column beats eyeballing floats across a 15-second auto — but it
  must be Javadoc'd as inferred, never authoritative.

**Honest risks:**
- **The §7.9 one-loop skew is the real risk in this design.** It produces a number that looks
  right and is wrong by roughly the magnitude being measured. Everything else here is
  arithmetic on two poses.
- The path-tangent finite difference (§2.2) is the second-most-likely thing to be subtly
  wrong. Tests 7 and 8 exist specifically for it.
- Scope check: ~20 topics for what is fundamentally "subtract two poses." The honest core is
  4 topics — longitudinal, lateral, rotation, following. The other ~16 are aggregates,
  identity, and annotation, each defended above. If this is judged over-built, the cut line
  is: drop §2.3 aggregates and §7.7 jump detection, keep everything else. That is a ~60-line
  class. Recommend building the full version anyway — the aggregates are what make the
  scorecard readable at competition, which is where this actually gets used.

**Cost:** estimated ~150 lines of implementation, ~120 lines of tests, 2 delegating lines in
`CommandSwerveDrivetrain`, 1 line in `RobotContainer.periodic()`.
