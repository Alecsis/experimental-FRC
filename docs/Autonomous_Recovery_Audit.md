# Autonomous Recovery Audit

**Date:** 2026-07-28
**Scope:** Whether autonomous can detect, tolerate, and recover from *failures* (command
lifecycle: intake/shooter/vision/pivot never reaching a goal, NamedCommand/parallel-group
deadlock, sensor-dependent waits that may never trip) and from *physical disturbance*
(defense hits, wheel slip, beaching, pose jumps, vision loss) without a human. Explicitly
**not** drivetrain velocity architecture (`DriveRequestType`/`Slot0`/SysId — hardware-blocked,
see `docs/superpowers/plans/2026-07-28-autonomous-velocity-migration.md`) and **not** the
still-unmerged `feature/autonomous-completion-trigger-framework` worktree. Builds directly on
`docs/Autonomous_Command_Lifecycle_Audit.md` (hang-class bugs in `"Orbit"` and
`shootingSequence()` already fixed, commit `8cdcb9a`, plus the resolution guard and
`Auto/Running`/`Auto/EndedInterrupted` telemetry from that same day's continuation) — this
audit does not re-litigate those, it picks up from "every NamedCommand terminates" to ask "does
anything *notice or react* when a command's result is wrong, or when the world changes out from
under it."

**Method:** every claim below is traced to source — this repo's own files, or PathplannerLib
2026.1.2 / Phoenix 6 26.1.3 sources decompiled/extracted for this audit
(`PathplannerLib-java-2026.1.2-sources.jar`, `wpiapi-java-26.1.3-sources.jar`) — not inferred
from documentation or prior sessions' memory. No code changed this session; no bug found rose to
"silently hangs forever," which is the bar the last two sessions' fixes were held to, so nothing
here met the "critical bug, fix immediately" exception to the audit-first instruction.

## Summary

The command-lifecycle layer is now well-guarded against *hanging* (last session's audit closed
that gap). This audit finds a different, until-now-uninvestigated gap: **the robot has almost no
way to notice its own actions failed, or that it has been physically disturbed, while still
inside a bounded command.** Every timeout added so far bounds *duration*, not *success* — a
command can time out and move on having accomplished nothing, and nothing downstream is told.
Concretely:

- **Intake success is never checked.** A `hasGamePiece` sensor field exists in the IO layer and
  is populated in simulation, but is never read by any production code, and is never set at all
  by real hardware. `"Intake Start Sequence"` always runs its full 5s regardless of whether a
  fuel piece was captured in the first 0.3s or never at all.
- **Shot success is never checked.** There is no sensor of any kind confirming a fuel piece
  actually left the robot; the state machine assumes success once its feed timer elapses.
- **Shooter jam detection exists but isn't wired to anything.** `Shooter.isJammed()` mirrors
  `Intake`'s already-shipped jam-recovery pattern almost exactly, but nothing calls it from the
  state machine — `Intake`'s jam recovery was wired in; `Shooter`'s was not.
- **Trajectory tracking error is computed and logged, but nothing reads it back.** No runtime
  code ever calls `TrajectoryErrorTracker`'s own getters — the only place that number is ever
  compared against a threshold is a post-hoc JUnit-only log parser (`WpilogStallAnalyzer`) used
  for regression testing, which cannot run during an actual match.
- **A real physical disturbance (vision jump-rejection) has no recovery path.** The MegaTag2
  innovation gate that protects against a single bad vision frame has no backoff — if a genuine
  hit displaces the robot far enough, vision corrections showing the *true* pose are rejected
  right alongside bad ones, forever, with no mechanism to ever re-trust vision again.
- **PathPlanner already ships real recovery primitives this repo has never called**: an
  AD*-based dynamic pathfinder (`AutoBuilder.pathfindToPose`/`pathfindThenFollowPath`, with a
  real, populated `navgrid.json` already sitting in this repo's deploy folder — 608/1512 grid
  cells marked as obstacles, not an empty default) and `FollowPathCommand`'s own
  replan-from-current-pose-on-(re)initialize behavior. Neither is used anywhere in this codebase.
- One thing several early hypotheses assumed turned out to be **not a risk at all**: a PathPlanner
  `"path"` segment (`FollowPathCommand`) cannot hang the way NamedCommands could — its
  `isFinished()` is purely elapsed-time-based against the trajectory's own precomputed duration,
  confirmed from decompiled source. The prior audit's fixes correctly targeted the actual risk
  (NamedCommands with sensor-condition-only end states); path segments were never at risk of the
  same bug class.

## Classification key

Every finding below is tagged one of:
- **BUG** — confirmed, reproducible defect in this repo's own code.
- **RISK** — a real gap this repo's own code has, not yet observed causing a failure, but
  traceable to a concrete failure scenario.
- **LIMITATION** — an accepted tradeoff (usually: no sensor hardware exists yet) that code
  changes alone cannot close.
- **UNUSED-CAPABILITY** — something PathPlanner/Phoenix 6/WPILib already ships that this repo
  simply never calls.

---

## Findings

### F1 — Intake success is never verified (RISK, high severity, high likelihood)

`IntakeIO.java:28` declares `public boolean hasGamePiece = false;` inside `IntakeIOInputs`.
`IntakeIOSim.java:110-127` populates it from maple-sim's own `IntakeSimulation.getGamePiecesAmount()`
— real, working simulated ground truth. **`IntakeIOReal.updateInputs()` (`IntakeIOReal.java:99-118`)
never assigns `inputs.hasGamePiece` at all** — it stays permanently `false` on real hardware,
confirmed by reading every line of that method. A repo-wide grep for `hasGamePiece` shows it is
read in exactly one place, `IntakeIOSim.java:127` itself (the assignment) — **no production code
anywhere (`Superstructure.java`, `RobotContainer.java`) ever reads this field.** This was flagged
as a known gap when the field was added (`docs/superpowers/specs/2026-07-16-maple-sim-physics-design.md:50`:
*"no real beam-break/sensor exists yet; out of scope for this pass"*) and has not been revisited
since.

Consequence: `Superstructure.INTAKING` (`Superstructure.java:204-210`) has no completion
condition in `periodic()`'s switch at all — it just continuously commands roller/pivot/agitator
every tick. The bound comes entirely from `Superstructure.intakeSequence(double)`
(`Superstructure.java:176-178`, `intakeCmd().withTimeout(timeoutSeconds)`), which the auto's
`"Intake Start Sequence"` NamedCommand registers at a fixed 5.0s
(`RobotContainer.java:111-112`). The auto cannot know, and cannot react to, whether a fuel piece
was actually captured — a piece grabbed in the first 0.3s wastes 4.7s waiting out the timer, and
a piece that's *never* grabbed (missed pickup, piece bounced off, mechanical failure) also just
consumes the full 5.0s and moves on with an empty robot, indistinguishable in telemetry from a
successful intake.

- **Classification:** RISK (not yet a BUG in the sense of "reproducibly wrong behavior observed,"
  since sim and hardware both currently "work" in the sense of not hanging — but a genuine
  detection gap with a concrete failure scenario).
- **Existing telemetry:** `hasGamePiece` in sim only; nothing in real-hardware telemetry
  distinguishes success from failure.
- **Missing telemetry:** a real hardware sensor (beam-break, color sensor, or a
  current/velocity-based capture heuristic analogous to `Intake.isJammed()`) — this is a
  **LIMITATION** at the hardware layer specifically for `IntakeIOReal`; the **RISK** is that even
  the sim-available signal is wired nowhere.

### F2 — Shot success is never verified (LIMITATION, informational)

No `ShooterIO`/`ShooterIOReal`/`ShooterIOSim` field exists anywhere resembling `hasGamePiece` for
the shooter/indexer path (confirmed: `ShooterIO.java` has no such field; `Shooter.java`'s only
state signals are RPM and stator current). `Superstructure.SHOOTING`
(`Superstructure.java:231-246`) starts feeding once `mSettleDelaySeconds` elapses and returns to
`STOWED` once `mFeedTimeoutSeconds` elapses — entirely open-loop. This is a genuine hardware gap
(no exit-sensing hardware exists on this robot to close), not a code defect — listed here because
the audit explicitly asked whether "shooter never reaches readiness" is detectable, and the
honest answer is: readiness (RPM) is detected and bounded correctly
(`shooterAtSpeed()`/`mAlignTimeoutSeconds`, `Superstructure.java:226`), but *whether the shot
itself left the robot* is not detectable with current hardware. No fix proposed; flagged so a
future hardware revision (an exit beam-break) has a clear target.

### F3 — `Shooter.isJammed()` exists but is never consulted by the state machine (RISK, medium-high severity, medium likelihood)

`Shooter.java:114-117` implements `isJammed()` (index stator current above threshold and velocity
below threshold) — structurally identical to `Intake.isJammed()`
(`Intake.java:161-164`), which **is** wired into a real, already-shipped recovery behavior
(`Intake.setRoller()`'s jam-recovery pulse/cooldown state machine, `Intake.java:113-143`). The
prior lifecycle audit's own catalogue (`docs/Autonomous_Command_Lifecycle_Audit.md`, "Lower-
severity / informational findings") already flagged `Shooter.indexJam()`
(`Shooter.java:119-127`) as dead code sharing the exact unbounded-`waitUntil` pattern as the
already-fixed `"Orbit"` bug, and confirmed it's called nowhere. This audit adds the missing half
of that picture: **`Superstructure.SHOOTING` (`Superstructure.java:231-246`) never calls
`shooter.isJammed()` at all**, bounded or not — a jammed indexer during an autonomous shot simply
keeps commanding `Shooter.indexing.INDEX` voltage into the jam for the rest of
`mFeedTimeoutSeconds`, then stows as if the shot succeeded, identically to F1's intake-side
symptom. Unlike `"Orbit"`/`shootingSequence()`, this does not *hang* (still bounded by the
existing feed timeout) — that's why it wasn't caught by the prior audit's hang-focused sweep — it
silently *fails without recovering or reporting*, which is exactly this audit's different focus.

- **Existing telemetry:** `SmartDashboard.putBoolean("Shooter/Index Stall", ...)`
  (`Shooter.java:225-226`) — dashboard-only, not an `AdvantageKit` `Logger.recordOutput`, so it
  never reaches a wpilog and can't be regression-tested or reviewed post-match the way everything
  else in this audit can.
- **Missing telemetry:** an AdvantageKit-logged jam signal (mirroring
  `Intake/JamRecoveryActive`, `Intake.java:153`) plus a `Superstructure`-level consumer.
- **Smallest safe fix (not implemented this session):** port `Intake.setRoller()`'s existing
  jam-recovery pattern to `Shooter.indexControl()`/`Superstructure.SHOOTING`, reusing the already-
  proven pulse/cooldown design rather than inventing a new one.

### F4 — Trajectory tracking error is computed and logged, but has zero runtime consumers (RISK, high severity, medium likelihood)

`TrajectoryErrorTracker.periodic()` (`TrajectoryErrorTracker.java:67-103`) computes
lateral/longitudinal/heading error every cycle and logs all three plus a freshness flag
(`logAndClearFreshness()`, lines 105-111) — this already exists and already runs live in
production, wired from `RobotContainer.trajectoryTrackerPeriodic()`
(`RobotContainer.java:150-152`) called every `robotPeriodic()` (`Robot.java:105`). **But its own
getters (`getLongitudinalErrorMeters()`, `getLateralErrorMeters()`, `getHeadingErrorRadians()`,
`TrajectoryErrorTracker.java:113-123`) are never called anywhere in the repository outside their
own declarations** — confirmed by a repo-wide grep. The only code that ever reads these numbers
back out of a log and compares them to a threshold is `WpilogTrajectoryErrorReader`
(`src/test/java/frc/robot/auto/WpilogTrajectoryErrorReader.java`), which parses a **finished**
`.wpilog` file after the JUnit test process has already torn the robot down — structurally
incapable of running during a live match.

This means: a robot being defended off its path, or slipping badly enough to fall meters behind
the trajectory, currently produces a perfectly accurate, perfectly logged number describing
exactly how bad things are — and nothing on the robot ever looks at that number while it matters.
`WpilogStallAnalyzer` (`src/test/java/frc/robot/auto/WpilogStallAnalyzer.java`) is the same
story for stall detection: it re-derives a stall signal from raw pose deltas in a **post-hoc log
scan** (`checkForStall()`, lines 40-121), specifically because a live NetworkTables poll of a
single-tick freshness pulse was tried and found unreliable (documented in its own class comment,
lines 16-24) — the team already discovered and solved the *aliasing* problem for a test harness,
but never carried that solution back into a *live*, in-match detector. Both analyzers'
algorithms (rolling-window translation delta vs. threshold; error magnitude vs. threshold) are
already correct and tuned (`AutoRegressionTolerances.java`) — porting the same math to run inside
`TrajectoryErrorTracker.periodic()` itself would not require inventing a new detection strategy,
only relocating one that's already proven.

- **Existing telemetry:** `Trajectory/ErrorLateralMeters`, `Trajectory/ErrorLongitudinalMeters`,
  `Trajectory/ErrorHeadingRadians`, `Trajectory/SetpointFresh` (all logged live).
  `Drive/AppliedVoltsPerModule`/`Drive/StatorCurrentAmpsPerModule`
  (`CommandSwerveDrivetrain.java:632-641`) are also logged live every cycle and are similarly
  unconsumed by any runtime decision — currently used only for offline SysId curve-fitting.
- **Missing telemetry:** none — the raw numbers already exist. What's missing is a *consumer*: a
  boolean like `Auto/TrackingDegraded` (large sustained error) or `Auto/Stalled` (little motion
  despite a fresh, non-stationary setpoint), logged live, the same way `Auto/Running`/
  `Auto/EndedInterrupted` were added last session.

### F5 — Vision's collision-safety net (innovation gate) has no recovery path after a real disturbance (RISK, high severity, low-medium likelihood)

`Vision.fuseMeasurements()` (`Vision.java:197-241`) rejects any MegaTag2 reading whose
translation is more than `Constants.kMaxVisionJumpMeters` (`Constants.java:44`, `1.0` meter) from
the current pose estimate (`Vision.java:222-226`) — this is a deliberate, already-documented
safety feature (`Vision.java:219-221`'s own comment, "Fix B — innovation gate") protecting against
a single bad/ambiguous MegaTag2 solve. It works exactly as intended for that case. It has an
unaddressed edge: **the estimate being compared against is never itself corrected once vision is
being rejected**, and pure wheel odometry does not reliably self-correct after a real physical
hit (wheel slip during the hit itself corrupts the odometry integral that MegaTag2 is being
checked against). If a robot is hit hard enough to be genuinely displaced by more than 1.0 m,
every subsequent *correct* vision reading showing the robot's *true* location will also read as
"too far from the current (now-wrong) estimate" and be rejected — there is no backoff, no
consecutive-rejection counter, and no mechanism that ever re-trusts vision after repeated
agreement from multiple frames. The robot's belief about its own pose can become permanently
wrong for the remainder of the autonomous period (and, if `teleopInit()`'s pose reset
(`Robot.java:180`, `getPoseResetEstimate().ifPresent(...)`) also can't recover it — that path uses
a different MegaTag1-based one-shot reset, not the same gate — teleop too).

- **Existing telemetry:** `Vision/RejectedJumpMeters` (`Vision.java:239`, live) records the *size*
  of the most recent rejection each cycle, but there is no counter of *consecutive* rejections or
  *time since last accepted correction* — from this signal alone it is not possible to
  distinguish "one bad frame, working as designed" from "locked out since t=4.2s."
- **Missing telemetry:** a consecutive-rejection counter and/or seconds-since-last-accepted-vision-
  measurement, both natural additions to the same `fuseMeasurements()` loop.
- **Smallest safe fix (not implemented this session):** a bounded escape hatch — e.g., after N
  consecutive same-direction rejections within a short window, accept the reading anyway (treat
  repeated agreement across independent frames as evidence of a real disturbance rather than
  noise). This needs a design decision (what N, what window) that's out of this audit's scope to
  pick unilaterally.

### F6 — No beaching/high-centered or wheel-slip detection exists at runtime (RISK, medium severity, low likelihood)

No code anywhere compares commanded vs. actual chassis velocity, or module current vs. module
velocity, to distinguish "the robot is stationary because the path says to stop" from "the robot
is commanded to move and isn't" during live operation. `Drive/StatorCurrentAmpsPerModule`/
`Drive/AppliedVoltsPerModule` are logged (`CommandSwerveDrivetrain.java:632-641`, added for SysId
analysis) but read by no runtime consumer, only by offline analysis scripts and this session's own
grep. `WpilogStallAnalyzer` proves the *algorithm* for this (rolling-window pose-delta vs.
threshold, gated by "still within the path-following phase") already exists and is tuned
(`AutoRegressionTolerances.kStallWindowSeconds`/`kStallTranslationThresholdMeters`,
lines 22-25) — again, proven in a test-only, post-hoc context, never carried into a live detector.
Likelihood is rated low relative to F4/F5 because this field's specific game (fuel/cargo-style,
based on `PathplannerLib`/`maple-sim` fixture names) and the already-disabled bump-ramp collider
(`docs/claudex/architecture.md`'s "Simulation fidelity tradeoffs" section, `AddRampCollider=false`)
suggest beaching is a lower real-world risk than being pushed off-trajectory by defense, but the
detection gap itself is identical in shape to F4 and would reuse the same telemetry.

### F7 — `FollowPathCommand` cannot hang; the prior audit's fixes correctly targeted the actual risk (no action needed — verification, not a new finding)

Confirmed from decompiled source (`FollowPathCommand.java:180-183`):
`isFinished()` is `timer.hasElapsed(totalTime) || !Double.isFinite(totalTime)` — a pure elapsed-
time check against the trajectory's own precomputed duration, with no dependency on tracking
error, sensor state, or anything that could fail to become true in normal operation. Every `.auto`
file in this repo uses only `"named"`, `"parallel"`, `"path"`, `"sequential"`, `"wait"` node types
(confirmed by direct grep across all 13 files — no `"race"`/`"deadline"` node is used anywhere,
though `AutoCommandSafetyTest`'s JSON walker already supports parsing them). This means the actual
hang risk in this fleet was always concentrated in NamedCommands with sensor-condition-only end
states (`"Orbit"`, the old `shootingSequence()`/`intakeCmd()` pattern) — exactly what the prior
audit found and fixed — and never in the path segments themselves. Recorded here so a future
session doesn't re-spend effort re-verifying this.

### F8 (UNUSED-CAPABILITY) — PathPlanner ships a real dynamic pathfinder this repo has a navgrid for but never calls

`AutoBuilder.pathfindToPose()` / `pathfindToPoseFlipped()` / `pathfindThenFollowPath()`
(`AutoBuilder.java:287-388`) are backed by `LocalADStar`
(`pathfinding/LocalADStar.java`), a real Anytime Dynamic A* implementation running on a background
planning thread over a 0.2 m grid, with a static-obstacle grid loaded from an optional
`pathplanner/navgrid.json` (silently falling back to an open field if absent — but **this repo's
copy is not empty**: `src/main/deploy/pathplanner/navgrid.json` is a real, populated 27×56 grid at
0.3 m node size with 608 of 1512 cells marked as obstacles, presumably the field's Hub/structure
footprint). `PathfindingCommand` (the command `pathfindToPose` returns) short-circuits to a stop
if already within 0.5 m of the goal on `initialize()` (`PathfindingCommand.java:246-272`) and
continuously re-pulls fresh AD* solutions mid-execution when the planner produces one
(`execute()`, lines 286-358) — i.e., it is a genuine "replan around obstacles while already
moving" primitive, not just a one-shot point-to-point move. **Nothing in this repo calls any of
these three methods** (confirmed by grep across `RobotContainer.java`/`CommandSwerveDrivetrain.java`
and the whole `src/main/` tree). This is the single most directly applicable "rejoin the plan
from wherever the robot currently is" tool available, and it is fully unused.

### F9 (UNUSED-CAPABILITY, partial) — `FollowPathCommand` already replans from the live pose on (re)initialize; no automatic mid-execution reroute

No `enableDynamicReplanning`-style field exists anywhere in PathplannerLib 2026.1.2 (confirmed:
zero hits for `replan` anywhere in the extracted sources except one comment) — a historical
PathPlanner feature by that description is gone, not merely deprecated. What does exist and is
usable today: `FollowPathCommand.initialize()` (`FollowPathCommand.java:106-132`) checks the live
robot velocity/rotation against the path's `IdealStartingState`, and if velocity is off by more
than 0.25 m/s or (for a holonomic drive) rotation is off by more than 30°, it **regenerates the
trajectory from the robot's actual current pose/speed** instead of reusing the precomputed ideal
one. Practically: cancelling a `FollowPathCommand` and rescheduling the same command object will
produce a fresh trajectory toward the same path's waypoints from wherever the robot now is — a
real "resume after being knocked off course" mechanism — but only triggers on a fresh
`initialize()`, not automatically mid-flight the way `PathfindingCommand`'s AD* reroute does.
Nothing in this repo currently cancels and reschedules a `FollowPathCommand` for this purpose;
it's simply an available building block for whatever recovery framework gets built.

### F10 — No built-in stuck/stall signal exists anywhere in PathplannerLib or Phoenix 6 (confirms this repo's custom tooling is filling a real gap, not duplicating a library feature)

`PathPlannerLogging` exposes exactly three raw callbacks (current pose, target pose, active path)
— no error-magnitude or stall signal of any kind. `PPLibTelemetry`'s methods
(`setVelocities`/`setCurrentPose`/`setTargetPose`/`setCurrentPath`) are the same: raw values only,
intended for the PathPlanner desktop app. Phoenix 6's `getMotorStallCurrent()`
(`CoreTalonFX.java:2243-2292`) is a **static rated-motor-constant lookup** (the motor type's rated
stall current at 12V), not a live "this mechanism is currently stalled" signal — there is no
Phoenix 6 status code or fault for live stall/jam detection. This confirms `Intake.isJammed()`/
`Shooter.isJammed()`'s stator-current-plus-velocity-threshold pattern is the correct and only
available approach (no vendor library feature is being reinvented or duplicated by using it), and
that F4/F6's proposed live trajectory-error/stall consumer would likewise be original work, not a
library feature this repo is failing to discover.

### F11 — No 254/1678/6328/Phoenix6 reference example of timeout-wrapping a path-following segment was found (informational)

Searched `temp_reference/Team 254 Code`, `temp_reference/Lynk 2026` for an established pattern of
racing a `FollowPathCommand` (or `pathfindToPose`) against a timeout, the way this repo's own
prior audit added `.withTimeout()` to NamedCommands. 254's `PathfindingAutoAlignCommand.java`
calls `pathfindThenFollowPath`/`pathfindToPose` directly with no visible timeout wrapper in that
file. Lynk 2026's `LoggedCommands.java:151-152` has a generic `race(String, Command...)` logging
helper, but it's applied generically, not specifically to path-following. This repo's own
`.withTimeout()` fix from the prior audit appears to be original engineering, not an adopted
convention — worth knowing before assuming a "standard" pattern exists to copy.

### F12 — `configureAutoBuilder()`'s config-load failure is caught but silently degrades the whole autonomous stack (RISK, low likelihood, high severity if triggered)

`CommandSwerveDrivetrain.configureAutoBuilder()` (`CommandSwerveDrivetrain.java:365-393`) wraps
`RobotConfig.fromGUISettings()` and `AutoBuilder.configure(...)` in a try/catch that, on failure,
only calls `DriverStation.reportError(...)` (line 391) — construction continues normally. If
`pathplanner/settings.json` were ever malformed or unreadable, `AutoBuilder` would never be
configured, yet `RobotContainer.java:115` unconditionally calls
`AutoBuilder.buildAutoChooser()` immediately after — PathplannerLib's own `AutoBuilder` throws if
any of its static methods are called before `configure()` succeeds, which would propagate out of
`RobotContainer`'s constructor and crash robot code at boot, rather than degrading gracefully
(e.g., falling back to a teleop-only mode with a dashboard warning). Likelihood is low (the config
file is currently valid and rarely hand-edited), but the failure mode is a total boot crash, not a
graceful degradation — flagged for awareness, not a fix (this is a config-robustness question,
arguably out of this audit's "physical disturbance during a run" focus, but it is a real
autonomous-availability risk).

### F13 — Confirmed acceptable: CommandScheduler/mode-transition interruption behavior (no new finding)

Re-checked against this audit's own checklist (WaitUntilCommand conditions, parallel groups
without fallback, CommandScheduler interruption behavior) beyond what the prior lifecycle audit
already covered: no additional unbounded `waitUntil`/`parallel` construct was found in the
reachable command graph (the prior audit's inventory — `"Home Intake"`, `"Orbit"`,
`"Shooting Sequence"`/`"Quick Shooting"`, `"Intake Start Sequence"`, `"Intake Stop"` — remains
complete; this session did not find a seventh NamedCommand or a new binding). `Robot.teleopInit()`
cancelling `m_autonomousCommand` (`Robot.java:177-179`) and `testInit()`'s `cancelAll()`
(`Robot.java:192`) both remain correct mode-transition guards.

---

## What is already existing-but-unused vs. needs-implementation vs. not realistically achievable

| Capability | Status | Evidence |
|---|---|---|
| Pathfind-to-rejoin-trajectory around obstacles | **Exists, ships in PathplannerLib, unused** | F8 — `AutoBuilder.pathfindToPose`/`pathfindThenFollowPath`, real navgrid already deployed |
| Replan-from-current-pose on command re-init | **Exists, ships in PathplannerLib, unused** | F9 — `FollowPathCommand.initialize()`'s live-state check |
| Continuous mid-execution obstacle-aware reroute | **Exists (via `PathfindingCommand` only), unused** | F9 — only the AD*/pathfinding path gets this; plain `FollowPathCommand` does not |
| Live tracking-error / stall detection | **Needs implementation** (algorithm already proven in test code) | F4/F6 — port `WpilogStallAnalyzer`/`WpilogTrajectoryErrorReader`'s logic into a live `TrajectoryErrorTracker` consumer |
| Intake success detection | **Needs implementation in sim; needs new hardware for real robot** | F1 |
| Shooter jam recovery wired to state machine | **Needs implementation (pattern already proven via `Intake`)** | F3 |
| Vision-jump-lockout recovery / backoff | **Needs implementation (needs a design decision on N/window)** | F5 |
| Shot-success verification | **Not achievable without new hardware** | F2 |
| Dynamic replanning as a config flag (`enableDynamicReplanning`) | **Not achievable — removed from PathplannerLib 2026.1.2** | F9 |
| Built-in PathPlanner/Phoenix stall or stuck detection | **Not achievable — does not exist in either library at any version currently vendored** | F10 |

## Recommended implementation order (medium-term; not started this session)

1. **F4 first** — a live `Auto/TrackingDegraded`/`Auto/Stalled` telemetry consumer inside
   `TrajectoryErrorTracker`, reusing `WpilogStallAnalyzer`/`AutoRegressionTolerances`' already-
   tuned thresholds. Lowest risk (telemetry-only, no behavior change), highest leverage (every
   other recovery decision needs this signal to trigger from), and directly extends work already
   merged this same day (`Auto/Running`/`Auto/EndedInterrupted`).
2. **F3** — wire `Shooter.isJammed()` into `Superstructure.SHOOTING`, porting `Intake`'s proven
   jam-recovery pattern. Small, well-precedented, closes a real gap with an existing template.
3. **F1** — wire sim's existing `hasGamePiece` into `Superstructure.INTAKING` as an early-exit
   condition (shortens `intakeSequence` when a piece is captured early) and as a
   pass/fail telemetry signal (even without early-exit, logging "did this auto's intake actually
   succeed" is valuable on its own). Real-hardware detection is a separate, hardware-dependent
   follow-up (LIMITATION, not a code task).
4. **F5** — vision-lockout backoff. Needs a design decision (N consecutive rejections, what
   window) before implementation — recommend a short design/brainstorming pass, not a blind
   TDD start, since the tuning tradeoff (false-recovery risk vs. permanent-lockout risk) is a
   judgment call.
5. **F8/F9** — evaluate `pathfindToPose`/`pathfindThenFollowPath` as the actual recovery
   mechanism once F4's detection signal exists to trigger it from. This is the "significant
   architectural work" tier — it changes how a disturbed auto resumes, not just what it reports —
   and should go through `brainstorming`/`writing-plans` rather than a direct implementation,
   consistent with how this repo has handled comparably-sized changes before (e.g. the Autonomous
   Velocity migration plan).
6. **F12** — lowest priority; config-robustness rather than in-match recovery, low likelihood,
   fine to defer indefinitely unless `pathplanner/settings.json` ever actually becomes hand-edited
   or generated by a script that could produce malformed output.

F2, F7, F10, F11, F13 require no implementation (F2 is hardware-gated; F7/F10/F11/F13 are
verification findings, not defects).

## Proposed autonomous recovery framework (design sketch — not implemented)

Rather than inventing a new state machine from scratch, the pieces above compose into a layered
model that extends what already exists:

- **Layer 0 (exists today):** per-command bounds — every NamedCommand terminates
  (`docs/Autonomous_Command_Lifecycle_Audit.md`), every path segment terminates
  (F7). This layer answers "did this step finish," not "did it work."
- **Layer 1 (F4, recommended first):** a live degradation signal —
  `Auto/TrackingDegraded`/`Auto/Stalled`, computed the same way the existing post-hoc analyzers
  already do it, just running every cycle instead of after the fact. Read-only; changes nothing
  about robot behavior yet, purely makes the condition observable in real time (and, as a side
  effect, in every future match's wpilog without needing a special test).
- **Layer 2 (F1/F3, recommended second/third):** per-subsystem success signals
  (`hasGamePiece` wired up; `Shooter.isJammed()` wired up) feeding into `Superstructure`'s own
  state machine, the same way `shooterAtSpeed()` already gates `ALIGNING`→`SHOOTING`. These make
  individual command *outcomes* (not just their durations) observable and, where a proven
  recovery pattern already exists (`Intake`'s jam pulse), actionable.
- **Layer 3 (F5, needs a design pass):** a vision-trust supervisor sitting above
   `Vision.fuseMeasurements()`'s existing innovation gate, converting "N consecutive rejections"
   into a deliberate, bounded re-trust decision instead of an unbounded lockout.
- **Layer 4 (F8/F9, the actual architectural lift):** a top-level autonomous supervisor that,
  when Layer 1 reports sustained degradation during a `"path"` segment, cancels the current
  `FollowPathCommand` and either reschedules it (cheap: F9's automatic replan-from-current-pose)
  or calls `AutoBuilder.pathfindToPose()` toward the next waypoint (more capable: obstacle-aware,
  uses the already-deployed `navgrid.json`). This is the only layer that changes *what the robot
  does*, not just what it reports, and is explicitly out of this audit's implementation scope —
  it needs its own design/plan cycle, ideally after Layer 1's telemetry has actually been observed
  across a few matches/practice sessions so the "sustained degradation" threshold is tuned against
  real data rather than a guess.

This is intentionally not a from-scratch state machine: `Superstructure` already is one, and
`Auto/Running`/`Auto/EndedInterrupted` already established the pattern of exposing
scheduler-level truth as AdvantageKit telemetry. Layers 1-3 extend that same pattern to conditions
finer than "did the whole auto finish"; Layer 4 is the only piece that's a genuinely new
architectural component.

## Regression test plan (for when implementation begins — not written this session)

Following this repo's established pattern (RED test first, from `AutoCommandSafetyTest`/
`SuperstructureCommandRequirementsTest`/`RobotAutoTerminationTelemetryTest`):

- **F4:** a unit test feeding `TrajectoryErrorTracker` a synthetic sequence of setpoint/measured-
  pose pairs (no robot boot needed, same style as `WpilogStallAnalyzer`'s own pure-function
  design) asserting a `TrackingDegraded`-equivalent output flips true only after the sustained
  window, mirroring `AutoRegressionTolerances.kStallWindowSeconds`. A second test proves it does
  *not* trip during a legitimate stationary phase (reusing `kStalePathSetpointGraceSeconds`'s
  existing reasoning).
- **F3:** mirror `Intake`'s own jam-recovery tests (if any currently exist — verify) or the
  `forceJamConditionForTest`/`clearJamOverrideForTest` pattern (`Intake.java:168-177`) ported to
  `Shooter`; assert `Superstructure.SHOOTING` reacts to a forced jam the same way `INTAKING`
  already reacts via `Intake.setRoller()`.
- **F1:** a sim-only test using the real `IntakeSimulation` (already exercised by
  `IntakeIOSim.java`) — spawn a fuel piece in intake range, assert `Superstructure.INTAKING`
  transitions early once `hasGamePiece` flips, and a second test proving it still respects the
  timeout when no piece is ever presented.
- **F5:** a unit test on `Vision.fuseMeasurements()` (or a testable extraction of its logic) with
  an injected sequence of consecutive same-direction "jump" readings, asserting the eventual
  design's re-trust threshold behaves as specified — this test can't be written until the F5
  design decision (N, window) is made.
- **F8/F9 (Layer 4):** deferred until Layer 4 has an actual design — recommend a sim scenario
  that manually displaces `mapleSim`'s world pose mid-path (there is already a proven pattern for
  this: `SimSpawnPoseOwnershipTest`'s pose-reset plumbing) and asserts the supervisor's chosen
  recovery action fires.

## What this audit did not do

- Did not touch drivetrain velocity architecture, SysId, `Slot0` gains, or `DriveRequestType` —
  fully out of scope per instruction.
- Did not touch the still-unmerged `feature/autonomous-completion-trigger-framework` worktree.
- Did not implement any fix. Every finding above is a RISK/LIMITATION/UNUSED-CAPABILITY, not a
  hang-class BUG — none met the bar ("critical bug that clearly warrants immediate correction")
  that would justify deviating from audit-first.
- Did not pick the F5 backoff design's specific N/window, or Layer 4's specific trigger threshold
  — both are judgment calls flagged for a follow-up design pass, not decided unilaterally here.
- Did not re-verify the prior lifecycle audit's own findings beyond F7/F13's explicit
  confirmation checks.
