# Autonomous Architecture Audit & Engineering Roadmap

**Scope:** Investigation only, no production code modified. Verified via `git status --short` at the end of this audit — only this document was added.

**Context:** No physical robot until September 2026. All findings are evidence-based, sourced from the 13 `.auto` files, 24 `.path` files, `Superstructure.java`, `Vision.java`, `Intake.java`, `Shooter.java`, `CommandSwerveDrivetrain.java`, `RobotContainer.java`, `OperatorControls.java`, the existing test suite (`src/test/java/...`), and `docs/claudex/history.md`/`CLAUDE.md`'s prior-session findings. No numbers were copied from `temp_reference/` or invented.

**Status update 2026-07-30:** This audit is a 2026-07-28 snapshot. The auto regression suite has since been expanded to all 13 routes and the final verification run passed 65 tests. Coverage claims below that say only `LT Neutral` has a golden are superseded; the mechanism-success, vision-modeling, and recovery-design findings remain relevant.

---

## Executive Summary

The autonomous stack is architecturally sound at the pattern level: the 254-style centralized `Superstructure` state machine, 1678-style IO isolation, and AdvantageKit logging are all correctly and consistently applied, matching `CLAUDE.md`'s stated design intent. One real production bug (the `intakeCmd()` hang-forever-inside-`parallel` deadlock) was already found and fixed in a prior session, and that fix is durable and well-documented in code.

The gaps that remain are concentrated in three places, and all three point the same direction — **the autonomous stack has almost no event-driven or sensor-driven feedback; it is built entirely from fixed-duration timeouts and PathPlanner's own trajectory completion**:

1. **No note/fuel-presence sensing anywhere in the codebase.** Every intake and shooting sequence is time-bounded, not outcome-verified. A missed note or a stuck shot looks identical to a successful one from the code's point of view.
2. **Vision's trust filter (jump-gate, std-dev floor) is real and well-built, but its output never influences autonomous timing or behavior**, and — more importantly — **it has never been exercised by any simulation run**, because `VisionIOSim` deliberately returns a "perfect" measurement every cycle with no noise, dropout, or jump injection.
3. **Test coverage is now broad for route execution:** all 13 autos have recorded regression baselines. The ALIGNING-timeout race, mechanism-success behavior, and end-to-end noisy vision modeling remain separate coverage gaps.

None of this is a "the code is broken" finding — the auto that has been characterized runs, and the one historical deadlock bug is fixed and regression-tested. It is a "the code cannot currently tell you when something has silently gone wrong" finding, which is exactly the class of problem that bites teams for the first time at competition, under defense, when a note is missed on attempt one and nobody can prove it from the logs afterward.

All of the roadmap items below are software-only and executable in simulation before September — none require the physical robot, though several (note-presence sensing in particular) will ultimately need a hardware decision (beam-break vs. color sensor vs. current-signature-only) that a mentor should make deliberately rather than by default.

---

## 1. Autonomous Command Architecture

**NamedCommands (6 total, registered in `RobotContainer.java:75-94`):** `Home Intake`, `Orbit`, `Shooting Sequence` (5.0s), `Quick Shooting` (3.0s), `Intake Start Sequence` (5.0s), `Intake Stop`. Small, curated set — no duplication, no dead registrations.

**Command reuse:** Good. `shootingSequence(double)`/`intakeSequence(double)` are both thin, timeout-bounded bridges over the same `shootCmd()`/`intakeCmd()` hold-forever primitives used by teleop bindings — one implementation, two callers (`OperatorControls`/`RobotContainer` for teleop, `NamedCommands` for auto). `trackHub()`/`trackPassTarget()` correctly factor through a shared `trackTarget()` helper (`CommandSwerveDrivetrain.java:500-560`) rather than duplicating the FunctionalCommand shape.

**Lifecycle correctness — the one real historical bug, now fixed:** `Superstructure.intakeCmd()` is `Commands.startEnd(...)` (hold-forever). Every `.auto` file wraps it in a PathPlanner `"parallel"` block (compiles to `ParallelCommandGroup`, which requires *every* branch to finish). A hold-forever command inside that block silently hung every in-scope auto after its first path segment — a real, competition-relevant bug, found and fixed in a prior session via `intakeSequence(timeoutSeconds)` (`intakeCmd().withTimeout(timeoutSeconds)`), registered as `"Intake Start Sequence"` at 5.0s. The fix is durable and documented in-code (`Superstructure.java:161-170`). No other hold-forever Command is registered as a NamedCommand or used inside a `"parallel"` block in any of the 13 `.auto` files (verified by inspecting all of them) — `"Orbit"` is `trackHub(..., finishOnAlign=true)`, which is genuinely self-finishing.

**Interruption/cancellation safety:** Verified by `SuperstructureEjectingTest` — cancelling `ejectCmd()` correctly routes through `requestStow()` and stops the roller/indexer/agitator. `setState()` (`Superstructure.java:104-113`) correctly cancels `mAgitateCommand` whenever the state machine leaves `ALIGNING`/`SHOOTING`, so a mid-shot interruption doesn't leave the pivot agitating forever.

**Command ownership / scheduler interactions — one real, narrow race identified:** `Superstructure` never declares `Intake`/`Shooter` as command requirements (correct, per the 254-style direct-command pattern this project has deliberately adopted) — but `ensureAgitating()` (`Superstructure.java:116-121`) *does* schedule a real `Command` (`intake.agitatePivot()`) that requires `Intake`. Separately, `RobotContainer.java:73` binds `RobotModeTriggers.teleop().onTrue(intake.homing())`, which also requires `Intake`. If auto ends while `Superstructure` is still in `ALIGNING`/`SHOOTING` (plausible — the last phase of every auto is exactly this state, and a real DS auto→teleop transition doesn't force-cancel subsystem-`periodic()`-driven behavior), `homing()`'s scheduling will cancel `mAgitateCommand`, and the very next `Superstructure.periodic()` tick will see `mAgitateCommand.isScheduled() == false` and immediately reschedule `agitatePivot()`, which cancels `homing()` in turn. This is self-resolving within `homing()`'s own 1-second timeout (`Intake.java:239`), not a permanent deadlock, but for that ~1s window the pivot will visibly fight between the homing raise and the agitate cycle. Low competition risk (bounded, short, cosmetic-to-moderate), but worth naming since it's a genuine ownership gap, not a hypothetical one.

---

## 2. Autonomous Reliability

**Timing assumptions — the dominant pattern in this codebase:** Every single `.auto` file starts with `Home Intake` → `wait 0.125s` → `parallel(path, Intake Start Sequence)`. All 24 `.path` files have `"eventMarkers": []` — **PathPlanner event markers are entirely unused across the whole deploy.** Synchronization between path progress and mechanism state is achieved exclusively through the coarse `"parallel"`/`"sequential"` block structure (start together, wait for both) and fixed-second timeouts (`intakeSequence(5.0)`, `shootingSequence(5.0)` vs. `shootingSequence(3.0)` for `"Quick Shooting"`, `mAlignTimeoutSeconds=1.0`, `mSettleDelaySeconds=0.3`). None of these durations are derived from a measured event (note fully seated, shot count reached, path percentage complete) — they're hand-picked per auto.

**Note pickup / missed intake — no recovery, because there is no detection.** `Intake`'s jam recovery (`isJammed()`/`jamRecoveryActive`, tested by `IntakeJamRecoveryTest`) detects a *mechanical roller stall* (high current + low velocity) and is well-built. But there is **no note-presence sensor anywhere in the codebase** (no beam-break, no color sensor, nothing in `IntakeIO`/`ShooterIO`). If `"Intake Start Sequence"` times out at 5.0s having picked up nothing — note not where the path assumed, note bounced away, defense knocked it aside — the auto has no way to know, and proceeds to the next path segment and eventually `"Shooting Sequence"` exactly as if the note were loaded. The shooter will spin up, the indexer will run for up to 5s, and nothing will fire, silently.

**Scoring failure — same gap, and one piece of dead code that would have helped.** `Shooter.indexJam()` (`Shooter.java:119-127`) is a real, already-written jam-recovery sequence for the indexer/feed mechanism — structurally similar to `Intake`'s roller recovery — but it is **never called anywhere** (not registered as a NamedCommand, not invoked from `Superstructure`). `Superstructure`'s `SHOOTING` case (`Superstructure.java:223-238`) drives the indexer directly (`indexControl(INDEX)`) with no jam awareness at all. If the indexer itself jams mid-shot in autonomous, nothing detects or recovers it — the mechanism sits stalled until `mFeedTimeoutSeconds` elapses and `Superstructure` gives up and stows, having never actually scored.

**Mechanism-delay handling is a deliberate, bounded tradeoff — worth naming explicitly, not a bug.** The `ALIGNING → SHOOTING` transition condition is `shooterAtSpeed(...) || timeInState >= mAlignTimeoutSeconds` (`Superstructure.java:218`) — an OR. In autonomous, a slow shooter spin-up gets force-fed after 1.0s regardless of whether it actually reached target RPM. This is a reasonable engineering call under a fixed 15s auto budget, but it means "shot fired" and "shot fired at correct RPM" are not the same guarantee, and nothing distinguishes the two after the fact in the logs today (see §5 for the recommended fix — logging alone would close most of this gap cheaply).

**Race conditions:** the `Intake`-requirement fight described in §1. No other scheduler-requirement race was found. The previously-documented cross-thread `CommandScheduler` race in the *test harness itself* (`AutoRegressionTestBase`, believed mostly fixed by the `stepTiming()`-before-`resumeTiming()` pattern, resurfaced once per `CLAUDE.md`) is test infrastructure, not production autonomous code — out of scope here but worth keeping on the radar.

**Deadlocks:** none found beyond the already-fixed `intakeCmd()` case. See §5 for a recommendation to add a *structural* test that prevents this exact bug class from being reintroduced by a future NamedCommand.

---

## 3. Auto Recovery Analysis

| Scenario | Current behavior | Graceful recovery today? |
|---|---|---|
| **Defense contact** | Pose estimator keeps running; `Vision`'s jump-gate (`kMaxVisionJumpMeters`) will *reject* a large post-collision vision correction because it looks identical to a bad reading (see §6) — meaning a real defense-induced drift is specifically the case this gate is least able to fix. | No. Odometry silently continues on a stale/wrong pose; `trackTarget()`'s alignment check (`§`) only checks *believed* heading against target, so it can report "aligned" while actually pointed wrong. |
| **Small localization drift** | Same jump-gate/std-dev-floor mechanism (`Vision.java:219-231`) — appropriately conservative for small, gradual drift. | Reasonable as-is for genuinely small drift; this is the one scenario the current design handles well. |
| **Intake misses** | No detection (§2). | No. |
| **Mechanism delays** | Bounded timeout, force-proceeds (§2). | Partial — bounded but not verified. |
| **Delayed shooter readiness** | Same OR-timeout as above. | Partial. |
| **Partial path completion** (e.g. wedged against another robot) | `"Intake Start Sequence"` self-times-out at 5s regardless of path progress; the outer sequential auto then starts the *next* path segment assuming the previous one geometrically finished. | No — the auto has no concept of "did the path actually get where it thought it did," only "did the timeout elapse." |
| **Interrupted commands** | `shootCmd()`/`intakeCmd()`/`ejectCmd()` require only the `Superstructure` subsystem, not the drivetrain, so a following `path` step never interrupts an in-flight mechanism sequence via scheduler requirements. Sequencing bugs of that shape were not found. | Yes, this specific failure mode is already handled correctly. |

**Where graceful recovery could be added without overcomplicating autos** (expanded on in the roadmap): (a) a cheap **note-presence signal** (even current-draw-based, not necessarily a new sensor) that lets `intakeSequence()` finish early on success and lets a downstream shot get skipped/logged as "nothing loaded" instead of firing blind; (b) **logging** `shooterAtSpeed()` vs. timeout-forced transitions distinctly, so a stale-RPM shot is visible after the fact even before any behavior changes; (c) a **vision-confidence signal** surfaced out of `Vision` (e.g., consecutive-rejected-correction count) that `Superstructure`/tracking commands could use to widen tolerances or flag a shot as low-confidence, rather than trusting the pose estimate identically regardless of how much vision has actually been trusted recently.

---

## 4. PathPlanner Usage (maintainability only — no gain-tuning recommendations)

**Path/auto organization:** `settings.json`'s `pathFolders`/`autoFolders` (`"Bump Paths"`/`"Trench Paths"`, `"Complex Autos"`/`"Simple Autos"`) are GUI-organizational metadata, not actual directory structure (all 24 `.path` files sit flat in `paths/`) — this is normal PathPlanner behavior, not a hazard. Each `.auto` file's `"folder"` field is consistently applied and matches its actual complexity (the three "Recollect"/one "U Turn"-with-recollect-shape autos are `"Complex Autos"`; everything else is `"Simple Autos"`) — good, self-documenting convention.

**AutoBuilder usage:** Single `configureAutoBuilder()` call (`CommandSwerveDrivetrain.java:351-396`), single `RobotConfig.fromGUISettings()` read — no duplicated/hand-constructed `RobotConfig` elsewhere. Clean.

**Event markers:** confirmed unused in all 24 `.path` files (`"eventMarkers": []` everywhere). This is the single biggest maintainability opportunity in the PathPlanner layer — see §2 and the roadmap.

**Path constraints:** `globalConstraints` are uniform across paths and match `settings.json`'s defaults (6.0 m/s, 4.5 m/s², 540°/s, 720°/s²) — no divergent per-path override found in the files sampled, aside from occasional `constraintZones` (e.g. `Left Trench Start.path`) used sparingly and appropriately for local slow-zones, not as a hidden global override.

**Path naming:** consistent `<Side> <Location> <Start/End/Neutral/Depot>` convention — genuinely good for a human maintainer to navigate without opening PathPlanner.

**Trajectory reuse:** `Depot`/`Depot - Score` are correctly shared between `LB Depot` and `LT Depot` rather than duplicated under different names — no redundant path definitions found.

**Repo hygiene (carried over from the prior Drivetrain Physical Constants Audit, still present, still deploy-bloat rather than a PathPlanner-usage bug):** stray stale duplicate `navgrid.json` files under `src/main/deploy/pathplanner/autos/deploy/` and an empty `autos/docs/superpowers/specs/` directory — recommend deletion, not performed here (investigation-only scope).

---

## 5. Simulation Coverage — What's Untested

**Coverage today, concretely:**
- The route regression guard now covers all 13 autos. Each route has a recorded golden and a concrete regression test. These are observation baselines, not tuning targets; route completion and tracking questions remain separate from the existence of coverage.
- `IntakeJamRecoveryTest` covers roller-jam detection/recovery well (pulse, cooldown, abort-mid-pulse via `requestStow()`).
- `SuperstructureEjectingTest` covers `EJECTING` → cancel → `STOWED` cleanly.
- `CommandSwerveDrivetrainSanitizeSpeedsTest` covers the auto-speed sanitizer.
- Nothing covers: the `ALIGNING` timeout-vs.-at-speed race, `indexJam()` (dead code, so untestable as-is), the `Intake`-requirement scheduler race from §1, or any vision-confidence/defense-contact scenario.

**The vision model remains idealized end-to-end.** `VisionIOSim`'s own class doc states it "returns a single perfect synthetic MegaTag2-style measurement" every cycle (`VisionIOSim.java:12-16`) — no noise, dropout, or injected jump. `AutonomousHealthMonitorTest` covers the rejection-streak bookkeeping with synthetic inputs, but no simulation test currently drives a noisy camera measurement through `Vision.fuseMeasurements()` and its jump gate. This remains the highest-value vision-modeling gap before autonomous recovery behavior is designed.

**Recommended new tests, ranked by "would this have caught a failure this codebase has already had":**
1. **Keep the structural/generic NamedCommand safety test current:** every NamedCommand referenced inside a `"parallel"` block must be provably self-finishing. This guard now exists; retain it as protection against reintroducing the historical `intakeCmd()` deadlock class.
2. **Keep the 13-route regression suite healthy:** the suite and recorded baselines now exist for all routes. Investigate route behavior separately from baseline maintenance; do not silently rewrite goldens.
3. **A `Vision` innovation-gate test**: extend `VisionIOSim` (or add a test-only variant) to inject a synthetic large pose jump mid-run and assert the fused pose does *not* teleport — mirrors the existing `SimSpawnPoseOwnershipTest` style/pattern already in this repo.
4. **A `Superstructure`-level unit test for the `ALIGNING` OR-timeout race**: force a shooter that never reaches target RPM (already possible via the existing `shooterTuningModeEnable`/mock pattern used elsewhere) and assert `SHOOTING` is still entered via the timeout branch, with the feed still occurring — currently this branch is not exercised anywhere.
5. **A deterministic REPLAY-mode test**: `Robot.java` already switches on `Constants.currentMode` and supports `WPILOGReader`; a REPLAY test against one of the already-recorded `.wpilog` files would lock in current logged behavior bit-for-bit as a cheap regression net, independent of and much faster than the live-sim regression harness.
6. **`indexJam()`**: either wire it into `Superstructure.SHOOTING` and write an `IntakeJamRecoveryTest`-style test for it, or delete it as dead code — leaving it unregistered and untested is the worst of both options (see roadmap).

---

## 6. Vision Integration

**Pose fusion is well-built and appropriately cautious.** `Vision.fuseMeasurements()` (`Vision.java:197-241`) gates on angular velocity (`kMaxOmegaRadPerSec`), tag count, max tag distance, a translation jump threshold (rejects >`kMaxVisionJumpMeters` corrections), and a minimum std-dev floor so a single close-in high-tag-count read can't be treated as perfectly trustworthy. This is genuine, already-committed trust-filter logic, not a gap.

**But "trusted appropriately" has a real caveat: the jump-gate can't tell a bad reading from a real large drift**, and defense contact is exactly the scenario that produces the latter (see §3). There is no escalation path (e.g., N consecutive same-direction rejections ⇒ trust the correction anyway) — worth flagging as a deliberate design tradeoff to revisit, not a bug to fix reflexively.

**Aiming (`getHubPosition()`/`calculateRPM()`) does not use live vision at all.** `getHubPosition()` averages *static* AprilTag field-layout geometry (`fieldLayout.getTagPose(id)`), filtered only by alliance — it never reads a camera frame. `calculateRPM()` feeds off `drivetrain.getState().Pose` (the fused pose estimate) to compute hub distance. So "vision integration" for aiming is entirely indirect: aiming accuracy depends on pose-estimator accuracy, which depends on vision fusion being trusted, but there is no additional aiming-specific safeguard. **Autonomous does not currently react differently when vision quality changes** — a robot shooting on dead-reckoning-only odometry after several rejected vision corrections computes and commits to an RPM with the same confidence as one running on fresh, accepted vision fixes. `rejectedJumpMeters`/`acceptedStdDevMeters` are logged (`Vision.java:239-240`) but nothing reads them to change behavior.

**Simulation-based validation opportunity:** as detailed in §5, `VisionIOSim` currently models a perfect sensor. The single highest-leverage vision-related task available before September is making `VisionIOSim` optionally inject noise/dropout/jumps, then using that to actually exercise the trust filter and (once built) any confidence-aware behavior change.

---

## 7. Software Architecture

**What's already right, and matches the project's own stated intent:** the 254-style centralized `Superstructure.periodic()` arbitration and 1678-style IO isolation (`VisionIO`/`IntakeIO`/`ShooterIO` + Real/Sim implementations) are both correctly and consistently applied — this isn't aspirational in `CLAUDE.md`, it's actually how the code is built. AdvantageKit logging is pervasive. Singleton pattern is followed everywhere it's supposed to be.

**Minor coupling note, not a bug:** `Superstructure`'s constructor calls `Vision.getInstance(drivetrain)` itself rather than receiving `Vision` injected (`Superstructure.java:62-64`). Harmless today because `Vision` is also a singleton and construction order is already correctly documented in `RobotContainer`'s field-ordering comments, but it means a future reader has to know that construction order matters rather than seeing it enforced by the type system.

**A small state-machine-consistency opportunity:** `Intake`'s roller jam-recovery (`isJammed()`/`jamRecoveryActive`/timestamp+cooldown fields, `Intake.java:49-51,113-143`) is a well-tested but hand-rolled micro-state-machine embedded in `setRoller()`. It's structurally inconsistent with the project's own preferred pattern (an enum + `periodic()` switch, as `Superstructure` uses). Not worth refactoring purely for consistency today, but if `indexJam()` gets wired in (§2/§5 recommendation) as a *third* copy of this same hand-rolled shape, that's the point to consider promoting it into one small reusable pattern rather than a third bespoke copy.

**Reusable-abstraction note:** `driveToPOI()` (`CommandSwerveDrivetrain.java:447-495`) and `trackTarget()` (`:511-560`) are both "PID-to-a-target-and-hold" `FunctionalCommand`s but don't share an implementation, unlike `trackHub()`/`trackPassTarget()` which correctly do share `trackTarget()`. Minor; only worth factoring if a third PID-to-pose need appears.

**Benchmarked against 254/6328/2910/971/1678, where relevant (not chasing changes without a clear benefit, per the ask):**
- **254 (centralized superstructure) and 1678 (IO isolation):** already correctly adopted — no gap.
- **6328/Mechanical Advantage:** known for heavy AdvantageKit logging (already adopted here) *and* for event/trigger-driven autonomous composition rather than fixed-duration waits. This is the clearest specific place this codebase is furthest from a strong peer benchmark — see §2/§4's event-marker finding. This is the one comparison worth actually acting on.
- **971 (Spartan Robotics):** known for rigorous state estimation with explicit confidence propagation through the control stack. The "vision confidence doesn't propagate beyond `Vision` itself" gap (§6) is the clearest divergence from that benchmark.
- **2910:** known for tightly-tuned live defense/collision compensation (bumper-force sensing, aggressive live replanning). Explicitly **not recommended** to chase — it's high-complexity, physical-robot-dependent, and disproportionate for a team without hardware access until September. Noted only to explain why it's excluded, not overlooked.

---

## Prioritized Roadmap

### High Impact / Low Risk (recommend starting here)

| # | Recommendation | Benefit | Complexity | Competition Risk | Sim Validation Strategy | Before Sept? |
|---|---|---|---|---|---|---|
| 1 | Maintain the structural test asserting every NamedCommand used inside a PathPlanner `"parallel"` block is self-finishing/bounded | Directly prevents recidivism of the exact bug class already found and fixed once (`intakeCmd()` hang) | Low — parse `.auto` JSON + a small known-bounded registry | None (test-only) | Runs as a plain JUnit test, no sim needed | **Yes** |
| 2 | Distinctly log whether `ALIGNING→SHOOTING` fired via `shooterAtSpeed()` or via `mAlignTimeoutSeconds` timeout | Makes "shot fired at wrong RPM" visible after the fact instead of indistinguishable from a good shot | Low — one `Logger.recordOutput` call at the existing branch | None | Immediately visible in any existing sim/regression run's wpilog | **Yes** |
| 3 | Maintain the 13-route regression suite and investigate route behavior independently of golden updates | Keeps all routes covered without turning observed behavior into an implicit tuning decision | Ongoing — the harness and baselines already exist | Route-specific failures still require triage | Direct — this is the current sim validation surface | **Yes** |
| 4 | Add noise/dropout/jump injection to `VisionIOSim`, then a test exercising the jump-gate's reject path | The trust filter is real, committed safety logic; the current tests cover health bookkeeping but not the full noisy measurement path | Medium — extend `VisionIOSim`, mirror `SimSpawnPoseOwnershipTest`'s style | None (test-only) | Direct | **Yes** |
| 5 | Delete `Shooter.indexJam()` or wire it into `Superstructure.SHOOTING`, decide explicitly rather than leaving it as unregistered dead code | Removes ambiguity between "unused" and "forgotten safety feature" | Low (delete) or Medium (wire in + test) | Low either way | If wired in: `IntakeJamRecoveryTest`-style test | **Yes** |

### High Impact / Medium Risk

| # | Recommendation | Benefit | Complexity | Competition Risk | Sim Validation Strategy | Before Sept? |
|---|---|---|---|---|---|---|
| 6 | Introduce PathPlanner event markers for at least the intake-start/intake-stop transitions, replacing the fixed `0.125s`/`5.0s`/`3.0s` waits with path-position-driven or outcome-driven triggers where a real signal exists | Directly targets the dominant reliability gap identified in §2/§4 (fixed-duration waits standing in for event-driven sequencing); the 6328-style benchmark this codebase is furthest from | Medium — requires deciding what signal drives the trigger (path % vs. a future note sensor) before implementing | Medium — changes real auto timing, needs careful regression-suite validation before trusting | Auto regression suite (item 3) is the direct validation mechanism — do this after, not before, expanding the suite | **Design now, implement once a note-presence signal exists or once path-position triggers are chosen** |
| 7 | Decide on and implement a note/fuel-presence signal (even a cheap current-draw heuristic, not necessarily a new physical sensor) | Closes the root cause behind §2's "missed intake looks identical to successful intake" and §3's "scoring failure has no recovery" findings — the single most consequential reliability gap in the whole audit | Medium-High — depends on the chosen sensing approach; a beam-break/color sensor needs a hardware decision, a current-signature heuristic could ship sim-only first | High if unaddressed going into competition — this is the class of failure that's invisible until it happens live | Can be developed and unit-tested in sim first (mock the signal), hardware-validated in September | **Design/sim-mock now; hardware wiring requires the physical robot** |
| 8 | Surface a vision-confidence signal out of `Vision` (e.g. consecutive-rejected-correction count) and use it in `Superstructure`/tracking commands to widen tolerances or flag a low-confidence shot | Closes §6's "confidence doesn't propagate beyond Vision" gap — the clearest divergence from the 971-style benchmark | Medium | Low-Medium — purely additive, existing behavior unaffected until the signal is acted on | Depends on item 4 (needs a non-perfect `VisionIOSim` to be testable at all) | **Yes**, once item 4 lands |
| 9 | Fix the `Intake`-requirement scheduler race between `homing()` and `ensureAgitating()` on the auto→teleop boundary | Removes a bounded-but-real ~1s mechanism fight at a mode transition | Low-Medium | Low (bounded, cosmetic-to-moderate today) | `IntakeJamRecoveryTest`-style targeted unit test forcing the transition mid-`SHOOTING` | **Yes** |

### Long-Term Improvements

| # | Recommendation | Benefit | Complexity | Competition Risk | Sim Validation Strategy | Before Sept? |
|---|---|---|---|---|---|---|
| 10 | Add a deterministic REPLAY-mode regression test against a checked-in `.wpilog` | Cheap, fast, bit-for-bit regression net independent of the live-sim harness's real-time cost | Low-Medium — `Robot.java` already supports `REPLAY` mode | None | Direct | **Yes** |
| 11 | If `indexJam()` is wired in (item 5) and a note-presence signal exists (item 7), consider promoting the hand-rolled jam-recovery pattern (`Intake`'s roller version, a wired-in indexer version, potentially others) into one small reusable shape instead of two-to-three bespoke copies | Consistency with the project's own stated state-machine-first design philosophy | Low, but only worth doing once there are genuinely 2-3 copies, not preemptively | None | N/A (refactor, covered by existing jam tests) | **Yes**, but only after items 5/7 make it worth it |
| 12 | Factor `driveToPOI()` and `trackTarget()` onto a shared PID-to-pose-and-hold implementation | Minor duplication cleanup | Low | None | Existing drivetrain tests | **Yes**, low priority |
| 13 | Clean up stray duplicate `navgrid.json`/`autos/docs` deploy-bloat (carried over from the prior Drivetrain Physical Constants Audit) | Removes a "which file is real" trap from the deploy payload | Trivial | None | N/A | **Yes**, low priority, do opportunistically |

---

## Items Requiring the Physical Robot

Everything in this document is simulatable and actionable before September **except**:
- The final sensing-technology choice for note/fuel presence (item 7) — a beam-break, color sensor, or current-signature-only decision should be validated against the real mechanism's actual electrical/optical behavior, not assumed from sim.
- Confirming that any newly-added event-marker-driven timing (item 6) still holds once real drivetrain characterization (SysId, per the separate `docs/SysId_Characterization_Checklist.md`) replaces the current unedited CTRE Tuner X gains — path timing assumptions made against today's simulated dynamics may shift once real `kS`/`kV`/`kA` are known.
- Live confirmation that `Vision`'s trust filter behaves the same against real camera noise/latency as it does against the injected synthetic noise from item 4 — the sim work closes the *testing* gap, not the *real-camera-behavior* gap.

Everything else — the structural NamedCommand-safety test, distinct RPM-transition logging, expanding the regression suite, vision noise injection and confidence propagation, the scheduler race fix, dead-code disposition, and the maintainability cleanups — is software-only and can be fully designed, implemented, and validated in simulation now.
