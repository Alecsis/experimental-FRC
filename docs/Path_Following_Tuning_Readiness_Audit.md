# Path-Following Stack Audit & September Tuning Playbook

**Scope:** Investigation only, no production code modified — verified via `git status --short` at the end of this audit. Every finding below is either (a) confirmed by reading this repo's committed source/config, (b) confirmed against current vendor documentation (PathplannerLib 2026.1.2, CTRE Phoenix 6 26.1.3, maple-sim 0.4.0-beta — versions read from `vendordeps/`), or (c) explicitly labeled a **hypothesis** where evidence is suggestive but not conclusive. No tuning values are recommended anywhere in this document.

**Assumed correct per instruction, not re-audited here:** controller output sanitization (`sanitizeAutoSpeeds`, commit `56ebba2`), the SysId infrastructure audit, the physical constants audit, and the autonomous architecture audit. Their findings are referenced where directly relevant but not re-derived.

---

## 1. PathPlanner Controller Configuration

**Current gains** (`CommandSwerveDrivetrain.java:364-368`, `configureAutoBuilder()`):
- Translation: `PIDConstants(5, 0, 0)`
- Rotation: `PIDConstants(3, 0, 0)`

Both are unchanged from a prior session's setup and have never been empirically tuned against real or SysId-characterized hardware (consistent with `CLAUDE.md`'s own tracked state). Confirmed: `LoggingHolonomicDriveController` (`utility/LoggingHolonomicDriveController.java`) only *observes* the vendor math via a logging override — it does not alter gains or clamp output.

**Feedforward usage — two independent feedforward paths, both correctly wired:**
1. PathPlanner's own per-trajectory-state field-relative velocity feedforward (`targetState.fieldSpeeds`) is summed with PID feedback inside `PPHolonomicDriveController.calculateRobotRelativeSpeeds()` with **no clamp between the two terms** (confirmed against PathplannerLib 2026.1.2 source in a prior session, re-affirmed here — this is why `sanitizeAutoSpeeds()` exists downstream).
2. Separately, PathPlanner also emits per-module **wheel force feedforwards** (Newtons), passed through untouched via `feedforwards.robotRelativeForcesXNewtons()/Y()` into CTRE's `ApplyRobotSpeeds.WithWheelForceFeedforwardsX/Y` (`CommandSwerveDrivetrain.java:362-363`). Per CTRE's own current documentation, these are applied as an added torque/current feedforward term on top of whichever closed-loop drive control mode is active, and CTRE explicitly recommends preferring PathPlanner's own precomputed forces over hand-rolled ones — **this wiring matches current best practice as documented.**

**Rotation delay distance — confirmed not applicable, not a gap.** Per PathPlanner's current docs, this parameter exists only on `AutoBuilder.pathfindToPose()`/on-the-fly pathfinding commands ("how far the robot should travel before attempting to rotate"). This repo makes **zero** calls to any pathfinding API (confirmed by grep — no `pathfindToPose`/`pathfindThenFollowPath`/`rotationDelayDistance` references anywhere) — every auto is built from statically-authored `.path` files loaded by name. There is nothing to configure here for this codebase's usage pattern.

**Goal end velocity behavior — one confirmed authoring inconsistency.** 3 of 25 `.path` files end with a non-zero `goalEndState.velocity` (0.85 m/s): `Left Trench Start.path`, `Left Bump Start.path`, `Right Bump Start.path` — all the first segment of a Neutral auto, intended to carry momentum into the next chained path rather than stopping between segments. Checking the immediately-following path in each auto's `.auto` sequence:

| Auto | First path (`goalEndState.velocity`) | Next path (`idealStartingState.velocity`) | Consistent? |
|---|---|---|---|
| LT Neutral / LT Neutral - * | `Left Trench Start.path` (0.85) | `Left Trench Neutral.path` (0.85) | **Yes** |
| LB Neutral | `Left Bump Start.path` (0.85) | `Bump Neutral Left.path` (0.85) | **Yes** |
| RB Neutral | `Right Bump Start.path` (0.85) | `Bump Neutral Right.path` (**0**) | **No — confirmed mismatch** |

`RB Neutral` is the one auto where the first path is authored to hand off momentum but the second path is authored assuming a standing start. This is a real, evidenced `.path`-authoring inconsistency — a PathPlanner GUI edit, not a gain-tuning item, and independent of the September tuning order below. Whether it's *causing* measurable tracking error hasn't been isolated (no `RB Neutral` regression golden exists yet per the Autonomous Architecture Audit), so its runtime impact is a **hypothesis**, but the inconsistency itself is a **confirmed fact**.

**Replanning configuration — confirmed removed from the vendor library, not a gap.** PathPlanner removed generalized dynamic replanning (`ReplanningConfig`) starting with the 2025 season release (confirmed via PathPlanner's own release history/community notes) specifically because a one-size-fits-all replanning solution didn't work well enough to keep. `AutoBuilder.configure()`'s call signature in this codebase (7 args, no replanning parameter) matches the current post-removal API exactly. There is nothing to audit here — the feature doesn't exist in 2026.1.2.

**Net assessment against current best practices:** the controller wiring itself (gains injection, dual feedforward paths, alliance-flip supplier, subsystem requirement) matches what PathPlanner's current docs describe as correct usage. The one concrete best-practice gap found in this section's scope is downstream of the controller, in §4 (discretization).

---

## 2. Simulation Fidelity

**MapleSim drivetrain model** (`utility/simulation/MapleSimSwerveDrivetrain.java`): `DriveTrainSimulationConfig.Default()` is customized with real robot mass/bumper size/module locations and a `SwerveModuleSimulationConfig` built from `moduleConstants[0]` — i.e., **the first module's gear ratios/friction voltages/wheel radius/steer inertia are used to configure all four simulated modules.** This is not a bug (this drivetrain's four modules share identical specs in `TunerConstants`, already confirmed in the Physical Constants Audit), but it's worth stating explicitly: the sim has no mechanism to represent a physically asymmetric drivetrain even if one existed.

**A confirmed, deliberate sim/real divergence in steer control:** `regulateModuleConstantsForSimulation()` (`:159-172`, real-hardware-guarded via `RobotBase.isReal()` early return) overrides the **simulated** steer motor's closed-loop gains to `kP=70, kD=4.5`, replacing `TunerConstants`' real steer `Slot0` (`kP=100, kD=0.5`). The in-code comment explains this is for numerical stability at the 200 Hz sim timestep, not an attempt to match real behavior. **Confirmed implication:** any steer-loop tracking behavior observed in sim today is running under different gains than what will actually be deployed to the real robot — sim steer-response characteristics are not representative and should not be used to judge real steer tuning quality. Sim-only drive/steer friction voltages (0.1 V / 0.15 V) and steer inertia (0.05 kg·m²) are also placeholder constants in this same override block, not derived from this robot's characterization.

**Wheel slip / traction ceiling:** per the already-assumed-correct prior finding, maple-sim clamps propelling force to `gravityForceOnModule × wheelCOF`; `wheelCOF = 0.8` remains unmeasured (carried over from the Physical Constants Audit, not re-litigated here).

**Battery simulation:** `TalonFXMotorControllerSim.updateControlSignal()` reads `SimulatedBattery.getBatteryVoltage()` per module — this **does** model voltage sag under aggregate current draw rather than assuming a constant 12 V, which is more realistic than a naive sim. No code anywhere in this repo customizes `SimulatedBattery`'s internal resistance/capacity away from maple-sim's library default (confirmed by grep — no `SimulatedBattery.` configuration calls found). **Unverified, not confirmed wrong:** whether that default matches this team's actual battery + PDH breaker configuration.

**Motor model:** `DCMotor.getKrakenX60(1)` for both drive and steer — confirmed consistent with mentor-confirmed real hardware (Kraken X60 throughout the drivetrain, per `CLAUDE.md`'s Backlog closure). The `(1)` argument (one motor per module, not a shared gearbox) is correct for this drivetrain's topology.

**Gyro simulation — confirmed zero noise/drift/bias.** `MapleSimSwerveDrivetrain.update()` sets `Pigeon2SimState`'s raw yaw and angular velocity directly from maple-sim's rigid-body physics truth every tick — reading the full file confirms no random/Gaussian perturbation is ever applied. This is a "perfect gyro," structurally the same finding class as vision simulation below: real Pigeon2 hardware will show small drift/bias/noise that this simulation currently cannot reproduce, so any part of the pose/heading stack that's sensitive to gyro imperfection is untested in sim today.

**Vision simulation:** `VisionIOSim` (already flagged in the Autonomous Architecture Audit) also returns a "perfect" synthetic MegaTag2 measurement every cycle with no noise/dropout/jump — directly relevant here too, since it feeds the same pose estimate that path-following tracks against.

**Consolidated sim-vs-real divergence list:**
1. Steer closed-loop gains (sim: `kP=70/kD=4.5`, real: `kP=100/kD=0.5`, deliberately different).
2. Drive/steer friction voltages and steer inertia used only in sim (placeholders, unmeasured).
3. Gyro noise/drift/bias: 0 in sim.
4. Vision measurement noise/dropout: 0 in sim.
5. Wheel-ground traction ceiling (`wheelCOF=0.8`): unmeasured on the real robot, used as-is in sim.
6. Battery internal-resistance/capacity model: vendor default, not verified against this team's real pack.
7. CAN/signal latency: sim explicitly does not attempt to fully match real CAN-FD/CANivore timing (the `kSimOdometrySignalHz` bump is documented in-code as narrowing, not eliminating, this gap).

---

## 3. Pose Estimation

**Odometry update order:** CTRE's generated `SwerveDrivetrain` base class runs its own background odometry thread (250 Hz per this drivetrain's CAN-FD configuration) independent of the 20 ms main robot loop; `getState().Pose` is a synchronized read of that thread's latest fused estimate. This is vendor-internal, thread-safety-guaranteed machinery not further auditable from this repo's source — treated as correct per standard, widely-deployed CTRE swerve usage.

**Estimator update timing — confirmed correct.** `Vision.periodic()` calls `io.setRobotOrientation(drivetrain.getRawGyroYawDegrees())` (the **raw**, unfused gyro yaw) *before* `io.updateInputs(inputs)` each tick. The in-code comment explains why this ordering matters: feeding the vision-corrected fused heading back into MegaTag2's own orientation input would create a self-reinforcing feedback loop. Confirmed by reading the code — this is implemented correctly, not merely claimed.

**Vision fusion timing — confirmed correct, capture-time-compensated.** `LimelightHelpers.getBotPoseEstimate()` (`utility/LimelightHelpers.java:931`) computes `adjustedTimestamp = (NetworkTables-receive-time) - (pipeline latency)` — standard, correct latency compensation, verified by reading the actual computation rather than assuming the vendored helper does the right thing. `CommandSwerveDrivetrain.addVisionMeasurement()` then routes that timestamp through `Utils.fpgaToCurrentTime()` before handing it to CTRE's internally-clocked pose buffer — this is CTRE's documented required pattern for externally-timestamped measurements and is correctly applied here.

**Alliance transforms — confirmed internally consistent, no double-flip found.** Multiple independent alliance-aware code paths exist (`POI.java`'s per-enum blue/red pose pairs, `Vision.java`'s alliance-branched hub/pass AprilTag ID sets, `FieldConstants.java`'s blue/red start poses, `CommandSwerveDrivetrain`'s operator-perspective rotation, and `configureAutoBuilder()`'s `shouldFlipPath` supplier for PathPlanner's own trajectory mirroring). Each has a distinct, non-overlapping responsibility: operator-perspective only affects human-joystick forward direction; PathPlanner's internal flip only affects trajectory waypoints; the fused pose itself (`getState().Pose`) is always blue-origin-absolute and is never separately re-flipped anywhere. Verified no code path applies two of these transforms to the same value. This is a genuinely well-architected area, worth stating as a confirmed-good finding rather than just an absence of bugs.

**Stale pose sources — none found beyond what's already fixed.** The pose-ownership doctrine already documented in-code and pinned by `SimSpawnPoseOwnershipTest` (MapleSim owns the disabled sim spawn only, AutoBuilder owns auto-init, Vision owns corrections) was re-checked against this session's trajectory-execution reading and holds. One nuance worth naming: `getSimulatedGroundTruthPose()` is a deliberately separate, non-fused pose stream (physics ground truth, used only as `VisionIOSim`'s synthetic camera source) — correctly kept apart from `getState().Pose` specifically to avoid a self-referential loop, not a second competing "real" pose.

---

## 4. Trajectory Execution — Scaling, Transforms, Discretization, Saturation

Traced the full path: `AutoBuilder.configure()` → `LoggingHolonomicDriveController`/`PPHolonomicDriveController` → the consumer lambda in `configureAutoBuilder()` → `sanitizeAutoSpeeds()` → `SwerveRequest.ApplyRobotSpeeds` → CTRE's generated module control.

**Confirmed finding — no discretization is ever applied to the commanded chassis speeds.** `grep` across the entire `src/` tree for `discretize` returns zero matches. CTRE's own current API documentation for `SwerveRequest.ApplyRobotSpeeds` states explicitly: *"Users must manually discretize these speeds if appropriate"* and *"this request does not automatically discretize the provided ChassisSpeeds."* PathPlanner's `FollowPathCommand`/`PPHolonomicDriveController` only auto-discretizes when paired with `SwerveSetpointGenerator` — confirmed unused anywhere in this repo (grep). **This is a confirmed gap in the pipeline**, not a hypothesis: continuous-time `ChassisSpeeds` (vx, vy, omega all computed independently for "this instant") are sent straight to the modules every loop with no correction for the well-documented skew that occurs when translating and rotating simultaneously within the same control period.

Whether this measurably contributes to the already-documented tracking error (`CLAUDE.md`'s ~6.9 m peak lateral error on `LT Neutral`) is a **hypothesis, not confirmed** — it hasn't been isolated from the other two already-known variables (uncharacterized drive `Slot0` gains, unclamped `kP=5` chassis feedback) via a controlled comparison this session. It's plausible the effect is present given several paths combine translation with active rotation targets (e.g. the 78–90° rotation targets seen in the sampled `.path` files), which is exactly the scenario discretization exists to correct — but plausibility is not proof.

**Scaling:** `sanitizeAutoSpeeds()` uses this drivetrain's own `getKinematics()` symmetrically in both directions (states→bounded-ChassisSpeeds) — self-consistent, all SI units (m/s, rad/s) confirmed throughout by reading the unit imports; no unit-mismatch found.

**Coordinate transforms:** `LoggingHolonomicDriveController`'s field→robot-relative conversion (`ChassisSpeeds.fromFieldRelativeSpeeds`) is confirmed logging-only instrumentation — it computes a separate value for `Logger.recordOutput` calls and does not alter the `total` ChassisSpeeds actually returned to the caller. No double-transform found in the control path itself.

**Saturation:** `sanitizeAutoSpeeds()` bounds chassis speeds once before `setControl()`; CTRE's `ApplyRobotSpeeds` desaturates wheel speeds again downstream by default (assumed-true per the prior session's decompiled-source finding). This is double-bounding via the same kinematics-based method at two points and was already confirmed idempotent by a prior A/B test — not a new bug. Worth flagging for whoever eventually implements discretization: discretizing changes the ChassisSpeeds numerically (introduces a small coupling term into vx/vy from omega), so it should be sequenced *before* `sanitizeAutoSpeeds()`, not after, or the two operations' effects could interact in a way that hasn't been characterized.

**Latency:** Inherent CAN/signal latency between `setControl()` on the main loop and actual module response isn't independently auditable further without decompiling CTRE's internal CAN stack; the `kSimOdometrySignalHz` mitigation from §2 is sim-only and doesn't apply to real hardware.

**Hidden unit conversions:** none found beyond the already-audited wheel-radius/gear-ratio chain (Physical Constants Audit). Both `sanitizeAutoSpeeds()` and `LoggingHolonomicDriveController` operate in consistent SI units throughout.

**Unverified, flagged rather than assumed:** `m_pathApplyRobotSpeeds` (`CommandSwerveDrivetrain.java:98`) never has `.withDriveRequestType()`/`.withSteerRequestType()` explicitly called on it — autonomous drive/steer control mode is whatever CTRE's `ApplyRobotSpeeds` class default is. This session could not confirm that default from CTRE's public API documentation (the field pages describe the fields but not their default values without decompiling the class, which wasn't done this session). **This should be confirmed before September** — it determines whether autonomous path-following is actually running through the drive `Slot0` closed loop that's about to be characterized and retuned, which is a precondition the whole tuning order below depends on.

---

## 5. Logging Sufficiency for September Tuning

**What already exists and is genuinely strong** (`Telemetry.java`, `TrajectoryErrorTracker.java`, `LoggingHolonomicDriveController.java`, `CommandSwerveDrivetrain.logDriveMotorVoltages()`/`sanitizeAutoSpeeds()`):

| Need | Existing telemetry | Sufficient? |
|---|---|---|
| Translation/rotation PID tuning | `Trajectory/CommandedTotalSpeeds`, `CommandedFeedforwardSpeeds`, `CommandedFeedbackSpeeds` (exact FF/FB split, robot-relative `ChassisSpeeds` including omega for rotation) | Yes |
| Path error | `Trajectory/ErrorLateralMeters`, `ErrorLongitudinalMeters` | Yes |
| Heading error | `Trajectory/ErrorHeadingRadians` | Yes |
| Controller output | `Trajectory/RawCommandedSpeed(s)`, `BoundedCommandedSpeed(s)`, `CommandedSpeedSaturated` | Yes |
| Module state error | `SwerveStates/Measured` vs `SwerveStates/Setpoints` (both logged, same struct shape) | Partial — both arrays exist but no per-module *error* is computed/logged directly; has to be derived manually in AdvantageScope every session |

**What's missing that would make September tuning significantly easier**, ranked by how directly it targets the tuning order below:

1. **The actual applied `DriveRequestType`/closed-loop mode is never logged.** Directly closes the unverified-default gap from §4 — a single logged enum/string would let a mentor confirm from any wpilog, without decompiling anything, that autonomous is really running through the drive `Slot0` loop about to be characterized.
2. **No motor-controller-level closed-loop error is logged** (distinct from module-level `Measured` vs `Setpoints`) — e.g. the drive `TalonFX`'s own reported closed-loop velocity error. This is the single highest-value addition for distinguishing "the `Slot0` gains are wrong" from "the trajectory/chassis controller is demanding something unreasonable," which is precisely the ambiguity the whole characterization effort exists to resolve.
3. **No per-module velocity/angle error computed directly** — `SwerveStates/Measured` and `SwerveStates/Setpoints` both exist but a direct `SwerveStates/ErrorPerModule` would remove a manual AdvantageScope-math step from every tuning iteration.
4. **No discretization-related telemetry**, because discretization isn't performed (§4) — if it's added later, logging pre/post-discretization commanded speeds would make its effect directly visible instead of inferred.
5. **No explicit path-segment-boundary markers** in the live log — nothing distinctly flags "path N ended / path N+1 began" the way the offline `WpilogTrajectoryErrorReader`/`WpilogStallAnalyzer` do post-hoc inside the JUnit harness only. This would make it much easier to visually confirm or rule out the `RB Neutral` goal-end-velocity mismatch (§1) against a live AdvantageScope trace during a real bring-up session.

---

## 6. September Tuning Playbook

This order follows directly from `CLAUDE.md`'s already-established fix sequence (bounded-output sanitization → SysId → drive `Slot0` → chassis PID), expanded here with concrete paths, acceptance criteria, and telemetry per step. No gain values are recommended — acceptance criteria are described structurally; the mentor sets the actual numeric thresholds.

**Step 0 — Pre-flight verification (once, before touching any gain):**
- Confirm the CANivore is actually named `"Swerve"` in Phoenix Tuner X (flagged unverified in the Physical Constants Audit — a mismatch here presents as "the whole drivetrain is dead").
- Confirm wheel radius with calipers against the installed wheel (flagged 4"-vs-3.5" discrepancy in the Physical Constants Audit).
- Confirm the actual `DriveRequestType` applied during autonomous (§4/§5 gap) — via Phoenix Tuner X's live signal view if the logging gap isn't closed first.
- Confirm `Drive/CancoderBootReadRotations` (already logged) match `TunerConstants`' configured offsets on boot.
- *Telemetry:* `Drive/CancoderBootReadRotations/*`, Phoenix Tuner X live view.

**Step 1 — SysId characterization (translation):** Execute the existing quasistatic/dynamic forward/reverse routines per `docs/SysId_Characterization_Checklist.md`. Not path-based — uses the dedicated `sysid` controller bindings, no `.path`/`.auto` involved. *Acceptance criteria:* SysId analyzer tool reports acceptable fit quality (R²); resulting `kV` is consistent with the theoretical value derivable from `kSpeedAt12Volts`/gear ratio/wheel radius; `kA` is nonzero and physically plausible for the characterized mass. *Move on when:* fit quality and sanity checks both pass. *Telemetry:* Phoenix hoot log via the SysId/Log Extractor pipeline (per the SysId checklist), not AdvantageKit.

**Step 2 — Apply characterized `kS`/`kV`/`kA` to `TunerConstants.driveGains`, verify open-loop-adjacent behavior before touching PathPlanner at all.** Run a short manual drive or SysId dynamic step at a few fixed speeds; confirm velocity tracking is flat/accurate before any closed-loop retuning. *Telemetry:* `Drive/AppliedVoltsPerModule` vs `SwerveStates/Measured` per-module speed. *Move on when:* steady-state speed error at constant commanded speed is acceptably small and consistent across modules.

**Step 3 — Retune drive `Slot0` closed-loop gains (`kP`/`kI`/`kD`) now that feedforward is characterized.** Use short step-response tests (manual `SwerveRequest.Velocity` command, not yet a full PathPlanner auto) to isolate module-level tracking from chassis-level PID. *Acceptance criteria:* settling time/overshoot the mentor finds acceptable at commanded-speed steps, without stator current pinning at the 120 A slip-current ceiling during routine (non-defense) tracking. *Telemetry:* `SwerveStates/Measured` vs `Setpoints` (module speed), `Drive/StatorCurrentAmpsPerModule`. *Move on when:* module-level velocity tracking is solid in isolation, before any chassis-level controller is involved.

**Step 4 — Re-verify the steer loop on real hardware.** Note from §2: sim's steer gains (`kP=70/kD=4.5`) never apply to real hardware, which runs `TunerConstants`' actual `kP=100/kD=0.5` — sim steer behavior observed to date is not representative of what real hardware will show. Watch especially during simultaneous translate+rotate motion, since this is also where the missing-discretization gap (§4) would be most visible even without any code change yet. *Telemetry:* `SwerveStates/Measured` vs `Setpoints` (module angle). *Move on when:* steer tracking is solid on real hardware specifically (not inferred from sim).

**Step 5 — Only now revisit `PPHolonomicDriveController`'s chassis-level gains** (current placeholders: translation `kP=5`, rotation `kP=3`). Start with `LT Neutral` — it already has the most existing characterization history and a (currently stale, to be re-baselined per the Autonomous Architecture Audit) regression golden to compare against. Validate in sim first against the existing regression suite before any real-hardware run, then progress from straight-line-dominant paths to rotation-heavy ones (Bump crossings, U-turns) last, deliberately, since those are where the discretization gap would show up most. *Acceptance criteria:* `Trajectory/ErrorLateralMeters`/`ErrorLongitudinalMeters`/`ErrorHeadingRadians` within mentor-set bounds per path; watch `Trajectory/CommandedFeedbackSpeeds` relative to `CommandedFeedforwardSpeeds` — a persistently large feedback term signals the chassis controller (or the trajectory itself) still isn't matched to real dynamics, not just an undertuned gain. *Telemetry:* `Trajectory/Error*`, `Trajectory/Commanded*Speeds`, `Trajectory/*CommandedSpeed(s)`.

**Step 6 — Only after Steps 1–5 hold on real hardware, evaluate whether discretization (§4) is worth adding.** This is a code change and out of this audit's scope, but flagged as the next concrete implementation candidate once the other two known confounds (uncharacterized feedforward, untuned chassis PID) are no longer in the way of isolating its real contribution. Recommend an A/B (with vs. without a manually-inserted `ChassisSpeeds.discretize()` call ahead of `sanitizeAutoSpeeds()`) specifically on the rotation-heavy paths identified in Step 5.

**Step 7 — Expand the auto regression suite** to the remaining autos (already tracked in `CLAUDE.md`'s backlog) only once the above is stable, so new goldens are established against genuinely-tuned behavior rather than baking in pre-characterization drift. **Fix the `RB Neutral` `goalEndState`/`idealStartingState` velocity mismatch (§1) at any point in this window** — it's a PathPlanner GUI edit, independent of the gain-tuning order, and cheap to do whenever convenient.

**Paths/telemetry quick-reference by step:**

| Step | Path(s) used | Primary telemetry |
|---|---|---|
| 0 | none | `Drive/CancoderBootReadRotations`, Tuner X |
| 1 | none (SysId routine) | Phoenix hoot log |
| 2 | none / simple manual drive | `Drive/AppliedVoltsPerModule`, `SwerveStates/Measured` |
| 3 | none / manual velocity step | `SwerveStates/Measured` vs `Setpoints`, `Drive/StatorCurrentAmpsPerModule` |
| 4 | none / manual steer step | `SwerveStates/Measured` vs `Setpoints` (angle) |
| 5 | `LT Neutral` first, then Bump/U-Turn/Recollect | `Trajectory/Error*`, `Trajectory/Commanded*` |
| 6 | rotation-heavy paths from Step 5 | Same as Step 5, A/B with/without discretization |
| 7 | remaining 12 autos | Auto regression suite goldens |

---

## Summary of Confirmed vs. Hypothesis Findings

**Confirmed (evidenced directly from repo source or vendor docs):**
- No `ChassisSpeeds.discretize()` call exists anywhere in the pipeline; CTRE's `ApplyRobotSpeeds` does not do it automatically.
- `RB Neutral`'s path chain has a real `goalEndState`/`idealStartingState` velocity mismatch; `LT Neutral`/`LB Neutral`'s equivalent chains do not.
- Rotation delay distance and replanning configuration are both genuinely not applicable to this codebase's current PathPlanner usage (not gaps, feature-absence-by-design/vendor-removal).
- Simulated steer gains, friction voltages, gyro, and vision are all confirmed to diverge from real-hardware behavior in specific, named ways.
- Vision fusion timing, alliance transforms, and estimator update ordering are all confirmed correct.
- Existing AdvantageKit logging already covers FF/FB split, path error, heading error, and controller saturation well.

**Hypothesis (plausible, not isolated/proven this session):**
- Missing discretization measurably contributes to the currently-documented tracking error.
- The `RB Neutral` velocity-mismatch authoring inconsistency causes a visible tracking artifact.
- maple-sim's default battery model is a meaningful mismatch against this team's real pack.

**Unverified (flagged as an open question, not asserted either way):**
- The actual default `DriveRequestType`/`SteerRequestType` applied by `m_pathApplyRobotSpeeds` during autonomous — needs confirming before the September tuning order can be trusted to be characterizing the control path that's actually in use.
