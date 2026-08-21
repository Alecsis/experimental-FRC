# PathPlanner Auto Regression Suite — Design

## Why

Before September we want confidence that PathPlanner autos are mechanically valid and that
regressions get caught without needing the physical robot. This is the "replay/regression
testing" milestone named (but not built) in the tenth session's autos-reliability roadmap
(`docs/claudex/history.md`) — Milestone 1 (`TrajectoryErrorTracker`, eleventh session) built the
error-computation foundation this milestone consumes; this milestone is the first thing that
actually runs autos unattended and reports pass/fail.

This is a **regression** suite, not an accuracy suite: it answers "does this auto still behave
like the last known-good run," not "is this auto's trajectory objectively correct." Accuracy work
(PID retune) is explicitly out of scope and blocked on the sim-timing non-determinism documented
in `docs/claudex/design/sim-timing-determinism.md` — this suite is built entirely inside that
doc's tier 3 ("full MapleSim → route completion, collision, and integration testing," which
tolerates jitter that would poison a fine PID comparison), and deliberately avoids tier 2
territory (no golden-final-pose-as-ground-truth, no gain comparison).

## Scope (v1)

Priority: trench autos (`LT*`/`RT*`, the most-run family in competition) plus the Depot autos
(the fallback used when trench wasn't trusted). 11 of the repo's 13 `.auto` files:

| Auto | Paths (in order) | Crosses | Risk notes |
|---|---|---|---|
| `LT Neutral` | Left Trench Start → Left Trench Neutral → Left Trench End | Trench (L) | Baseline trench auto |
| `LT Neutral - Bump Recollect` | ...Left Trench... → Left Bump Recollect Start | Trench (L) + Bump | Depends on the Bump/ramp collider fix (`AddRampCollider=false`) staying in place |
| `LT Neutral - Bump Route` | Left Trench Start → Bump Neutral Left → Left Bump End | Trench (L) + Bump | Same collider dependency |
| `LT Neutral - Depot Recollect` | ...Left Trench... → Backup into Depot | Trench (L) + tight scoring area | |
| `LT Neutral - Trench Recollect` | ...Left Trench... → Left Trench Recollect Start | Trench (L), double pass | |
| `LT Neutral - U Turn` | Left Trench Start → Left Trench Neutral - U Turn → Left Trench End | Trench (L), tight U-turn | Tightest maneuvering of the trench family |
| `LT Depot` | Left Trench to Depot → Depot → Depot - Score | Trench (L) + tight scoring area | Depot fallback |
| `RT Neutral` | Right Trench Start → Right Trench End | Trench (R) | Baseline trench auto |
| `RT Neutral - Trench Recollect` | ...Right Trench... → Right Trench Recollect Start | Trench (R), double pass | |
| `RT Neutral - U Turn` | Right Trench Start - U Turn → Right Trench End | Trench (R), tight U-turn | |
| `LB Depot` | Left Bump Depot → Depot → Depot - Score | Bump + tight scoring area | Depot fallback; bump-prefixed but in scope per the Depot rule. Has two open findings this suite should track going forward: the bump-collider fix (twelfth session) and an unresolved **~1.8m peak longitudinal tracking lag** (eleventh session, still undiagnosed) |

**Deferred to a later expansion:** `LB Neutral`, `RB Neutral` — pure bump-neutral autos with no
Depot fallback, lowest competition-usage priority per your call.

Real competition history motivates this ordering independent of the above: the postmortem in the
separate `frc-2026/2026` competition repo (tenth session) found the trench *return* leg drifting
into the field wall in some runs, tied to path asymmetry (simple outbound leg, curved multi-target
return leg) rather than any known code defect on this platform. This suite is what would have
caught a regression in that behavior.

## Architecture

### File layout (all new, zero production files touched)

```
src/test/java/frc/robot/auto/
  AutoRegressionTestBase.java       # abstract harness: setup/teardown, run loop, assertions
  AutoRegressionTolerances.java     # one place for the placeholder tolerance constants
  WpilogTrajectoryErrorReader.java  # minimal pure-Java .wpilog reader, scoped to 2 entries
  LtNeutralAutoRegressionTest.java
  LtNeutralBumpRecollectAutoRegressionTest.java
  LtNeutralBumpRouteAutoRegressionTest.java
  LtNeutralDepotRecollectAutoRegressionTest.java
  LtNeutralTrenchRecollectAutoRegressionTest.java
  LtNeutralUTurnAutoRegressionTest.java
  LtDepotAutoRegressionTest.java
  RtNeutralAutoRegressionTest.java
  RtNeutralTrenchRecollectAutoRegressionTest.java
  RtNeutralUTurnAutoRegressionTest.java
  LbDepotAutoRegressionTest.java

src/test/resources/autoRegression/
  LT_Neutral.json
  LT_Neutral_Bump_Recollect.json
  ... (one golden file per auto, 11 total)
```

Each per-auto class is ~3 lines: it extends `AutoRegressionTestBase` and supplies the PathPlanner
auto name (and the golden-file name). All harness logic — HAL/DriverStationSim/SimHooks setup,
the run loop, log correlation, assertions — lives once in the base class.

### Why one class per auto, not one parameterized class

`build.gradle:89-106` sets `forkEvery = 1` specifically because Vision/Intake/Shooter/
Superstructure are singletons with no reset hook — reusing a JVM across test *classes* would leak
state. `RobotLifecycleTest`'s own comment documents the same constraint for its single-`@Test`-
method design. A `@ParameterizedTest` looping over 11 autos inside one class would run all 11 in
one JVM/one set of singletons — exactly the leak `forkEvery=1` exists to prevent. 11 small classes
is more boilerplate but is the isolation the codebase already committed to.

### Per-test flow (in `AutoRegressionTestBase`)

**`@BeforeEach`** — identical to `RobotLifecycleTest`: `HAL.initialize`, `DriverStationSim.
resetData()`, `SimHooks.pauseTiming()`, construct `new Robot()`, start `startCompetition()` on a
daemon thread, `SimHooks.waitForProgramStart()`. Also snapshots the newest-`.wpilog`-by-mtime in
`logs/` (mirrors `SKILLS/run_headless_sim.py`'s existing convention) so the test can later identify
which log file its own run produced.

**Invariant this structurally guarantees:** `Robot()`'s constructor builds `RobotContainer`
synchronously — `NamedCommands.registerCommand(...)` and `configureAutoBuilder()`'s callback
registration both complete before `new Robot()` returns, i.e. before `@BeforeEach` itself returns,
i.e. before the `@Test` method (and therefore `AutoBuilder.buildAuto(...)`) ever runs. No
additional synchronization needed; documented here as a real invariant rather than left implicit,
per your explicit callout.

**The `@Test` method (single method, inherited by all 11 subclasses):**
1. `DriverStationSim.setAutonomous(true); setEnabled(true); notifyNewData();`
2. Build and schedule the auto directly: `CommandScheduler.getInstance().schedule(AutoBuilder.
   buildAuto(autoName()))` — matching the eleventh session's own validated approach (not puppeting
   the private `SendableChooser` over NetworkTables). PathPlanner's existing `resetPose` callback
   in `configureAutoBuilder()` seeds the correct starting pose automatically the moment the first
   path command runs; the harness does not seed pose itself.
3. Step `SimHooks.stepTiming(0.02)` in a loop, capped at **15.0s simulated time** (the real
   autonomous period — doubles as a non-arbitrary timeout). Each tick:
   - sample `RobotContainer.drivetrain.getState().Pose` (first sample kept as start pose,
     diagnostic only)
   - check `CommandScheduler.getInstance().isScheduled(cmd)`; the tick it goes false marks
     `completed = true` and the loop ends early
   - **stall detector:** over a rolling window (e.g. last 1.0s of samples), if translation moved
     less than a threshold (e.g. 0.05m) while `cmd` is still scheduled, mark `stalled = true` and
     end the loop early — this is the "collision prevents progress" signal; there is no direct
     collision flag anywhere in MapleSim/CTRE, so sustained non-movement while still commanded is
     the proxy
4. Record: `completed`, `stalled`, `runtimeSeconds` (simulated time elapsed), final pose
   (diagnostic only), and — via `WpilogTrajectoryErrorReader` against the log file identified in
   `@BeforeEach` — `maxLateralErrorMeters`/`maxLongitudinalErrorMeters` (max-abs over the whole run
   of `Trajectory/ErrorLateralMeters`/`ErrorLongitudinalMeters`, which `TrajectoryErrorTracker`
   already logs; this suite reads that existing log, it does not touch the tracker).

**`@AfterEach`** — identical to `RobotLifecycleTest`: `endCompetition()`, thread join, `robot.
close()` (flushes the `WPILOGWriter` so the log is complete before the reader touches it),
`SimHooks.resumeTiming()`.

### `WpilogTrajectoryErrorReader`

Pure Java, test-scope only, no new production dependency. Reimplements just enough of
`SKILLS/parse_akit_log.py`'s already-working WPILOG-record parsing (magic header, varint-length
records, `double` entry decode) to scan one file for two named entries and return max-abs. Chosen
over shelling out to the Python script so `./gradlew test` stays a self-contained JVM gate (no
Python-on-PATH dependency inside a Gradle test fork, no subprocess fragility).

### Golden metadata (not final-pose snapshots)

Per your correction: the primary regression signal is *behavioral metadata*, not a strict
final-pose match. Final pose is stored but only as diagnostic information a human can look at —
it is never asserted against.

```json
{
  "autoName": "LT Neutral",
  "recordedAtGitSha": "d7833f4",
  "completed": true,
  "runtimeSeconds": 8.42,
  "maxLateralErrorMeters": 0.29,
  "maxLongitudinalErrorMeters": 0.51,
  "diagnosticStartPose": { "xMeters": 1.62, "yMeters": 5.23, "thetaRadians": 0.0 },
  "diagnosticFinalPose": { "xMeters": 7.10, "yMeters": 5.05, "thetaRadians": 0.03 }
}
```

**Pass/fail against the golden** (constants centralized in `AutoRegressionTolerances`, all
explicitly PLACEHOLDER values per your "loose now, tighten later" call, since the sim-timing doc
already measured ~0.3m RMS run-to-run jitter on *identical* code — max error will scatter more
than RMS does):
- `completed` must equal the golden's `completed` (bool equality, not toleranced — a golden that
  says `true` and a run that comes back `false` is an unconditional fail)
- `stalled` is an independent, non-toleranced fail condition (a completed run is never stalled by
  definition, but a run can fail via stall before reaching the 15s cap)
- `runtimeSeconds` within golden **± 3.0s** (placeholder — generous relative to the 15s window)
- `maxLateralErrorMeters` within golden **+ 0.75m** absolute headroom (placeholder)
- `maxLongitudinalErrorMeters` within golden **+ 1.0m** absolute headroom (placeholder — wider than
  lateral since the one measured longitudinal outlier, LB Depot's ~1.8m lag, is longitudinal)

  All additive, not multiplicative — e.g. LB Depot's golden will already carry its known ~1.8m
  longitudinal lag; the tolerance's job is "don't let it get worse," not "fix it." These three
  numbers are the only tuning knobs the "tighten later" fast-follow needs to touch, and they live
  in exactly one file (`AutoRegressionTolerances.java`), not scattered across 11 test classes.

### Golden update flow (explicit double-confirmation, per your requirement)

Default mode is always assert-only. Recording/overwriting a golden file requires **both**
`-DupdateAutoGolden=true -DconfirmGoldenUpdate=true` on the same `./gradlew test` invocation. If
exactly one is set (a likely typo scenario), the test fails loudly with a message naming the
missing flag, rather than silently falling back to normal assert-mode — a silent fallback could
mask that an intended update never actually happened. Golden files are checked into git like any
other source file, so an update is a normal reviewable diff (a human looks at the new numbers
before merging, same as reviewing any other regression baseline change).

### Edge cases handled

- **First run for a given auto, no golden file yet:** fails loudly with a clear "no golden found,
  run with -DupdateAutoGolden=true -DconfirmGoldenUpdate=true to establish a baseline" message,
  rather than silently passing or crashing on a null-pointer.
- **Multiple test classes writing to `logs/` across one `./gradlew test` invocation:** `forkEvery
  =1` means each auto's test runs in its own JVM. `build.gradle`'s `test {}` block (lines 89-106)
  sets no `maxParallelForks`, so Gradle's default of 1 applies — confirmed absent, not assumed —
  meaning these also run sequentially, so at most one `WPILOGWriter` is ever active at a time —
  "newest file by
  mtime, snapshotted before this class's `Robot()` construction" (the same technique `SKILLS/
  run_headless_sim.py` already uses) is sufficient to identify the right file, no directory-diff
  machinery needed.
- **`.wpilog` not yet flushed:** `@AfterEach`'s `robot.close()` runs (and blocks) before the
  reader ever opens the file, matching `RobotLifecycleTest`'s existing teardown order.

## What this explicitly does not do

No production robot code changes, no PID constants, no PathPlanner settings/path edits, no
telemetry instrumentation changes (`TrajectoryErrorTracker` is read via its existing log output,
never modified), no collision physics changes, no real-hardware anything, no final-pose-as-ground-
truth assertions, no attempt to fix the LB Depot longitudinal lag or the trench return-path drift
finding — this suite's job is to make the *next* regression on any of these visible, not to fix
the ones already known.
