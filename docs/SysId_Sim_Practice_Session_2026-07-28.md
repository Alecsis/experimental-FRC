# SysId Simulation Practice Session — 2026-07-28

> ⚠️ **THIS IS NOT HARDWARE CHARACTERIZATION.** Every number below comes from a headless JUnit
> **simulation** run against maple-sim's own unmeasured friction placeholders
> (`kDriveFrictionVoltage`/`kSteerFrictionVoltage`, `TunerConstants.java:98-99`). None of it may be
> pasted into `TunerConstants.driveGains`, and Phase 2 of
> `docs/superpowers/plans/2026-07-28-autonomous-velocity-migration.md` is **not** complete or
> advanced by this session. This document exists to validate the **operator procedure and tooling
> pipeline**, nothing more — same purpose and same warning as
> `docs/SysId_Sim_Workflow_Validation.md`, which this document extends.

## Purpose

Mentor-requested manual practice session: exercise the full SysId workflow — all **four**
directions (quasistatic forward/reverse, dynamic forward/reverse) — in simulation, to validate the
operator procedure before physical-robot characterization. Phase 1 Task 1.1
(`CommandSwerveDrivetrainSysIdSimWorkflowTest`, commit `7518244`) already proved the **forward**
direction of both routines; this session adds the two directions it didn't cover.

## What was run

1. **`CommandSwerveDrivetrainSysIdSimWorkflowTest`** (pre-existing, unmodified) —
   `sysIdQuasistatic(kForward)` for 3s, then `sysIdDynamic(kForward)` for 1.5s, translation SysId
   routine, teleop-enabled.
2. **`CommandSwerveDrivetrainSysIdReverseSimWorkflowTest`** (new file, this session) — identical
   structure, `sysIdQuasistatic(kReverse)` then `sysIdDynamic(kReverse)`.

Both boot the robot in sim via the same `Robot()`-thread pattern `AutoRegressionTestBase` uses,
schedule the routine through `CommandScheduler` (simulating an operator's SysId-button press), let
it run for a bounded window, cancel it, then read the resulting wpilog.

**Why two separate test classes, not one with four `@Test` methods:** attempted the natural
single-class version first and it failed with `IllegalThreadStateException` at `Logger.start()`
(AdvantageKit's logger is a JVM-wide singleton; `build.gradle`'s `forkEvery = 1` forks a new JVM per
test *class*, not per method — the same reason every `AutoRegressionTestBase` subclass is its own
one-test class). Confirmed by reproducing the failure, not assumed. Splitting into two classes
matches this codebase's own established convention and fixed it.

## Whether the procedure works

**Yes, for all four directions.** Both test classes pass, each asserting real evidence the routine
actually drove the simulated motors, not just that the command completed silently:

| Direction | Class | Result | Evidence |
|---|---|---|---|
| Quasistatic forward | `CommandSwerveDrivetrainSysIdSimWorkflowTest` | PASS | `Drive/AppliedVoltsPerModule` max abs 4.0V, >3 distinct values, >10 samples |
| Dynamic forward | (same class, same run) | PASS | same telemetry, forward-direction voltage |
| Quasistatic reverse | `CommandSwerveDrivetrainSysIdReverseSimWorkflowTest` | PASS | `Drive/AppliedVoltsPerModule` **min −4.0V** (signed, not just magnitude), >3 distinct values, >10 samples |
| Dynamic reverse | (same class, same run) | PASS | same telemetry, reverse-direction voltage |

The reverse test asserts a **signed** minimum voltage below −0.5V specifically — proving the
routine commanded genuinely negative-direction voltage, not merely "some nonzero value" (which a
magnitude-only check, as the forward test already used, would not have caught if reverse silently
behaved like forward). This is the one piece of new verification logic this session added, beyond
re-running what Task 1.1 already built.

### Analysis leg — reverse direction

Task 1.2 (commit `2ac441a`) already proved the analysis pipeline (wpilog → matched telemetry →
least-squares fit) on forward-direction data. As part of this session's practice run, the same
one-off least-squares fit (`V = kS·sign(vx) + kV·vx`) was re-run against the **reverse** wpilog to
confirm the pipeline isn't direction-dependent:

| Quantity | Forward (Task 1.2, `kS=0.1849, kV=1.7293`, R²=0.9198) | Reverse (this session) |
|---|---|---|
| Paired samples | 151 | 149 |
| `vx` range | 0.0000 – 1.6133 m/s | −1.6113 – 0.0000 m/s |
| Applied-volts range | 0.0000 – 4.0000 V | −4.0000 – 0.0000 V |
| `kS` (placeholder) | 0.1849 V | 0.1708 V |
| `kV` (placeholder) | 1.7293 V·s/m | 1.7428 V·s/m |
| R² | 0.9198 | 0.9149 |

Both directions fit cleanly (R² ≈ 0.91–0.92) and produce comparable `kS`/`kV` magnitudes — the
pipeline behaves symmetrically, as expected of a mirrored routine. **Same placeholder caveat as
Task 1.2 applies**: these numbers prove the analysis pipeline works, nothing about the real robot.
Not committed as a reusable script (one-off, consistent with Task 1.2's own scope).

## Expected operator workflow

Based on running this end-to-end (in test-harness form, standing in for an operator pressing bound
buttons on a real driver station), the procedure per
`docs/SysId_Characterization_Checklist.md` §4 is:

1. Deploy code, connect driver station, enter **Teleop or Test** mode (SysId's `setControl()` calls
   are ignored while disabled — confirmed by this session's own `DriverStationSim.setEnabled(true)`
   requirement, not assumed).
2. Select the routine to characterize (`useTranslationSysId()` — this repo also has a rotation SysId
   routine, untouched by this session).
3. Run, in order, holding each button until the routine naturally completes or the robot approaches
   its safe travel limit:
   - Quasistatic forward
   - Quasistatic reverse
   - Dynamic forward
   - Dynamic reverse
4. Stop logging, pull the `.hoot` file (real hardware) or wpilog (sim) off the robot/laptop.
5. Run the WPILib SysId analyzer (real hardware) against the `.hoot` log to get `kS`/`kV`/`kA`.

On the physical robot this is a **4-button, sequential procedure** with real travel-distance limits
per direction (per the checklist's own field-space warning) — nothing about this session changes
that guidance, it only confirms the underlying command lifecycle behaves as the checklist assumes.

## Sim limitations vs. real hardware

Carried forward from Task 1.1/1.2, reconfirmed this session (ran 3/3 more times across both new and
old test classes, consistent both times):

- **No `.hoot` file is produced.** Phoenix's `SignalLogger` needs a real CAN bus/CANivore to attach
  to; this headless JUnit harness has none. `SignalLogger.start()` and the state-callback
  `writeString()` calls never throw, but no file lands in `logs/`. Confirmed again this session for
  the reverse routine specifically (`hootLogProduced: false`, both runs). **This means the real
  analyzer tool (WPILib SysId / Phoenix Tuner X) cannot be exercised in this harness at all** — only
  the AdvantageKit wpilog + a hand-rolled regression fit stand in for it, which is a materially
  different (if numerically similar) analysis path than what will run on the physical robot.
- **Sim-derived `kS`/`kV` are circular by construction.** maple-sim's friction model
  (`kDriveFrictionVoltage`/`kSteerFrictionVoltage`) is itself an unmeasured placeholder — a sim
  "characterization" mostly recovers what was already fed into the sim, not real motor/gearbox/wheel
  friction. Real characterization requires the physical robot.
  Notably, the reverse and forward `kS`/`kV` values are close in this session's fit (0.1708 vs
  0.1849, 1.7428 vs 1.7293) — a real robot could plausibly show more asymmetry (wiring, gearbox
  slop, weight distribution), which sim's symmetric friction model wouldn't surface.
- **No physical travel-limit constraint.** The checklist's real-world "watch the robot, stop before
  it hits something" concern doesn't exist in this harness — sim windows are just bounded by
  wall-clock time (3s quasistatic, 1.5s dynamic), not distance.
- **No real CAN bus timing/latency.** Sim runs on simulated periodic ticks with no real bus
  contention, `CAN message is stale` warnings, or the current-limiting behavior a real Kraken X60
  under `kSlipCurrent=120A` would show under sustained SysId driving.

## Full verification gates run this session

- `./gradlew compileJava` — BUILD SUCCESSFUL.
- `python SKILLS/run_headless_sim.py --run-seconds 12` — PASS.
- New reverse test: 1/1 pass, re-run twice more alongside the forward test for a total of 3
  consistent passes across both classes.
- Full `./gradlew test`: 13 tests, 1 failure — `LtNeutralAutoRegressionTest.regressionCheck()`,
  the same already-documented pre-existing flaky stall check
  (`docs/claudex/sessions/2026-07-28.md`'s Task 0.1/1.1 updates), this run at "moved 0.027m over the
  preceding 0.98s (threshold 0.050m)". **Not caused by this session** — `git status --short` shows
  the only change this session is the new
  `CommandSwerveDrivetrainSysIdReverseSimWorkflowTest.java` file; zero production code was touched,
  so there is no plausible causal path from this session's change to that auto's stall-timing
  behavior.

## Whether we are ready for hardware characterization

**Procedurally, yes — the workflow, command lifecycle, and telemetry pipeline are all proven to
work correctly for all four SysId directions, not just forward.** This session closes the one gap
Task 1.1 left open (reverse-direction coverage). Nothing found here changes Phase 2's blocker: real
hardware access is still required, per `docs/SysId_Characterization_Checklist.md` §4, and no
sim-derived number from this or any prior sim session may be used as production characterization —
the `.hoot`-log gap above means the *actual analyzer tool* still hasn't been exercised at all,
sim or otherwise, only substituted for. When physical-robot access is available, run the real
4-direction procedure with Phoenix Tuner X / the WPILib SysId analyzer per the checklist, not this
harness's hand-rolled fit.

## Files

- New: `src/test/java/frc/robot/subsystems/CommandSwerveDrivetrainSysIdReverseSimWorkflowTest.java`
  — reverse-direction companion to `CommandSwerveDrivetrainSysIdSimWorkflowTest`.
- This document.
- No `TunerConstants.java` or other production file touched.
