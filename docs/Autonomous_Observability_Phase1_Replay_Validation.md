# Autonomous Observability Phase 1, Stage A — Replay Workflow Validation

**Update, later same-day session (2026-07-28): the blocker described in §4/§8/§9 below is resolved.**
`Constants.currentMode` now reaches `Mode.REPLAY` automatically when the `AKIT_LOG_PATH` environment
variable is set — no `Constants.java` edit-run-revert needed. Fixed via TDD
(`Constants.resolveCurrentMode(isReal, envVar, simMode)` + `ConstantsReplayModeTest`, 3 pure tests),
verified with a real end-to-end replay run (console confirmed `Mode.REPLAY` was reached, a real
`_sim` log was produced containing both `/RealOutputs/...` and new `/ReplayOutputs/...` entries).
Full detail and verification evidence: `SKILLS/replay-testing-agent.md`'s Prerequisite section (now
rewritten to describe the fix) — not duplicated here to avoid two sources of truth. This document's
body below is preserved as the accurate historical record of the blocked state that motivated the
fix; treat §4, §8, and §9's "not fully complete" verdict as superseded by this note, not current.

**Date:** 2026-07-28
**Purpose:** Close out the remaining Stage A item from `docs/Autonomous_Observability_Phase1_Plan.md`
— validating the replay-based analysis workflow (`/replay`, `SKILLS/replay-testing-agent.md`,
`SKILLS/parse_akit_log.py`) end-to-end. **This document is verification/tooling only.** It does not
implement recovery behavior, does not touch `AutonomousHealthMonitor`, does not change any threshold,
and does not modify drivetrain/PID code. Where the workflow is blocked, that blocker is documented
plainly rather than worked around.

---

## 1. Purpose (expanded)

`docs/Autonomous_Observability_Phase1_Plan.md`'s Stage A lists "a replay-based before/after
comparison workflow" as the one item not yet closed after the F1/F3 signal-validation work
(`docs/Autonomous_Observability_Phase1_StageA_Validation.md`). This document validates whether that
workflow — actual AdvantageKit replay, plus manual two-log comparison via
`SKILLS/parse_akit_log.py` — is usable today, using real commands and real `.wpilog` evidence rather
than assuming the documented `SKILLS/replay-testing-agent.md` prerequisite still holds.

## 2. Environment / prerequisites

- Historical branch `refactor/hybrid`, working tree at the state left by the prior Stage A session (doc-only
  addition since; no `src/` changes between that session and this one).
- `JAVA_HOME=/c/Users/Public/wpilib/2026/jdk` (Temurin 17.0.16) — required per `CLAUDE.md`, the
  PATH `java` is JDK 25 and cannot run this Gradle version.
- Existing `.wpilog` files under `logs/` (368 files at the time of this session, spanning 2026-07-17
  through 2026-07-28) — no new sim/replay run was required to obtain comparison material; two logs
  produced by this same session's own `./gradlew test` run (see §5) were used directly.

## 3. Task 1 — Inspecting the replay workflow's prerequisite before assuming it works

Per `/replay`'s own instructions, the presence of `Robot.java`'s `case REPLAY:` and `build.gradle`'s
`replayWatch` task is not proof replay is executable. Read each piece directly rather than trusting
the skill doc's own prior claim:

**`Constants.java:16-18`:**
```java
public static final Mode simMode = Mode.SIM;
public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
```
`simMode` is a hardcoded `public static final Mode`, not read from any environment variable, system
property, or config file — confirmed by reading the entire file (30 lines, no other reference to
`Mode.REPLAY` or any env-var lookup anywhere in the class). `currentMode` resolves to `Mode.REPLAY`
if and only if `simMode == Mode.REPLAY`, since `RobotBase.isReal()` can only route to `REAL` or
`simMode` — there is no third path. Today, `simMode = Mode.SIM`, so `currentMode` can never be
`REPLAY` under the current source, regardless of any runtime flag.

**`Robot.java:78-90`** (`robotInit()`'s mode switch):
```java
case SIM:
  Logger.addDataReceiver(new WPILOGWriter("logs"));
  Logger.addDataReceiver(new NT4Publisher());
  break;

case REPLAY:
  setUseTiming(false); // Run as fast as possible
  String logPath = LogFileUtil.findReplayLog();
  Logger.setReplaySource(new WPILOGReader(logPath));
  Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
  break;
```
This branch is real, correctly structured AdvantageKit replay wiring — but it is switched on
`Constants.currentMode`, so with `currentMode` pinned to `SIM` as shown above, this branch is
unreachable dead code as the source stands today.

**`build.gradle:56-59`:**
```groovy
task(replayWatch, type: JavaExec) {
    mainClass = "org.littletonrobotics.junction.ReplayWatch"
    classpath = sourceSets.main.runtimeClasspath
}
```
`replayWatch` only launches AdvantageKit's own `ReplayWatch` utility against the existing
`simulateJava` classpath — it sets no system property or environment variable that `Constants.java`
reads (`Constants.java` reads none at all, per above), and it does not modify `Constants.java`. Even
if `AKIT_LOG_PATH` is set before running it, the robot code that boots still evaluates
`Constants.currentMode == Mode.SIM` and takes the `SIM` branch, never `REPLAY`.

**Conclusion of Task 1: this repo's own prior finding holds and was verified independently this
session by reading the current source directly, not by trusting `SKILLS/replay-testing-agent.md`'s
claim at face value.** Nothing in `Constants.java`, `Robot.java`, or `build.gradle` has changed since
that finding was recorded.

## 4. Task 2 — The blocker, stated plainly

**Replay cannot execute as-is.** `Constants.simMode` is hardcoded to `Mode.SIM`. Reaching
`Mode.REPLAY` requires a source change to `Constants.java` (setting `simMode = Mode.REPLAY` and
rebuilding) — there is no environment variable, Gradle property, or CLI flag that can flip it, because
nothing in the current source reads one. `Constants.java` is production code. Per this task's explicit
scope ("Verification/tooling only... do not modify production code") and `/replay`'s own rule ("the
one narrow exception is a `Constants.java` `simMode` toggle the user has explicitly authorized for a
single run, immediately reverted afterward"), **no such authorization was given for this task**, so no
edit was made and no replay was executed. This is not worked around, and no output is described as
"replayed" anywhere in this document.

## 5. Task 3 — Real replay execution: not performed (blocked, per §4)

Per `/replay`'s explicit rule — "do not proceed as if a replay happened, do not silently fall back to
parsing the input log and call it a replay result" — this task step is honestly reported as **not
done**, not substituted with a look-alike. No `_sim`-suffixed log was produced this session, because
no replay ran.

**What *was* done instead (see §6), and why it is not a substitute for §5:** `SKILLS/parse_akit_log.py`
was used to directly inspect two ordinary `.wpilog` files already on disk from this session's own
`./gradlew test` run. This is standard post-hoc log analysis (the same operation this project has
always used the tool for — see `SKILLS/log-analysis-agent.md`), not a replay. It exercises and
validates half of the replay-testing skill's toolchain (`parse_akit_log.py` reading real AdvantageKit
output) but not the other half (`Logger.setReplaySource`/`WPILOGReader` re-driving robot code from a
recorded log).

## 6. Task 4 — Two-log comparison workflow validation

**No automated diff harness exists in this repository.** Confirmed by reading
`SKILLS/replay-testing-agent.md`'s own Safety constraints section and by grepping for any
comparison/diff script under `SKILLS/` — `parse_akit_log.py` is the only log-analysis tool present,
and it operates on one log per invocation. This matches the skill's own documented stance; nothing new
was discovered here, but it was checked rather than assumed.

**Logs used (real, not fabricated):** this session's own `./gradlew test` run (§7) executed the
existing `frc.robot.recovery` disturbance-test package (`NoDisturbanceControlTest`,
`SmallDisplacementDisturbanceTest`, `MediumDisplacementDisturbanceTest`,
`SevereDisplacementDisturbanceTest` — all four pre-existing from the Autonomous Disturbance Simulation
experiment, unmodified this session), each running the same isolated PathPlanner path
(`"Left Trench Neutral"`). Matched each test class to its `.wpilog` by cross-referencing JUnit XML
`timestamp` attributes against `logs/` file mtimes (both monotonically ordered the same way):

| Test class | JUnit `timestamp` (UTC) | Matched log |
|---|---|---|
| `MediumDisplacementDisturbanceTest` | `2026-07-29T03:22:52` | `logs/akit_26-07-28_23-22-53.wpilog` |
| `NoDisturbanceControlTest` | `2026-07-29T03:23:01` | `logs/akit_26-07-28_23-23-02.wpilog` |
| `SevereDisplacementDisturbanceTest` | `2026-07-29T03:23:11` | `logs/akit_26-07-28_23-23-12.wpilog` |
| `SmallDisplacementDisturbanceTest` | `2026-07-29T03:23:20` | `logs/akit_26-07-28_23-23-21.wpilog` |

Both the control and severe logs report `WPILOG v1.0, 220 entries` and `span 0.00s → 6.36s` (via
`parse_akit_log.py`'s summary header, no `--dump`/`--grep`), and both contain 318 data records under
`/RealOutputs/Trajectory/ErrorLateralMeters` — apples-to-apples in duration and sample count, as
expected for the same path run under `PathDisturbanceSimTestBase`.

**Commands run (exact, `MSYS_NO_PATHCONV=1` needed under Git Bash so the leading `/` in the entry
name isn't mangled into a Windows path):**
```
python SKILLS/parse_akit_log.py logs/akit_26-07-28_23-23-02.wpilog \
    --dump "/RealOutputs/Trajectory/ErrorLateralMeters" --limit 500

python SKILLS/parse_akit_log.py logs/akit_26-07-28_23-23-12.wpilog \
    --dump "/RealOutputs/Trajectory/ErrorLateralMeters" --limit 500
```

**Raw output, last 5 samples of each (both real, literal tool output):**

Control (`NoDisturbanceControlTest`, `23-23-02.wpilog`):
```
t=    6.283s  -0.7313595830872888
t=    6.303s  -0.7972648702367285
t=    6.323s  -0.8770400848385657
t=    6.343s  -0.9536968010168886
t=    6.363s  -1.0277329178011048
```

Severe (`SevereDisplacementDisturbanceTest`, `23-23-12.wpilog`):
```
t=    6.283s  -0.0019316658297427658
t=    6.303s  -0.002047858747523782
t=    6.323s  -0.0021208413154004823
t=    6.343s  -0.002232063963248897
t=    6.363s  -0.002242724074965699
```

**Manual comparison (computed by hand from the two dumps above, excluding one leading `t=0.002s nan`
sample present in both logs — the tracker's own first-cycle no-prior-pose transient, not a parsing
error):** max `|ErrorLateralMeters|` over the full 6.36s run — **control: 2.219m**, **severe: 0.724m**.
Both raw value sets are cited above and in the full dump; the 2.219/0.724 figures are direct
`max(abs(x))` reductions over those printed values, not independently sourced numbers.

**This reproduces, on fresh data from this session, the same anomaly the original**
`docs/Autonomous_Disturbance_Simulation_Report.md` (Finding 4) flagged and left explicitly
unresolved: the more-disturbed run shows *less* final tracking error than the undisturbed control, the
opposite of what a real disturbance-recovery signal should look like. This document does not
re-investigate that anomaly (out of scope — Stage A is signal/tooling validation, not a rerun of the
Disturbance Report) but treats it as a useful sanity check: two independent sessions, using the same
manual `parse_akit_log.py`-based method, produced the same qualitative result on two different
same-day test runs, which is evidence the comparison method itself is reproducible, not just a
one-off reading.

**Workflow conclusion:** the manual two-log comparison workflow described in
`SKILLS/replay-testing-agent.md`/`.claude/commands/replay.md` works as documented — `parse_akit_log.py
--dump <entry>` against two logs, diffed by hand — and required no code change to exercise, since it
operates on already-produced `.wpilog` files regardless of whether they came from `SIM`, `REAL`, or
(if it were reachable) `REPLAY` mode.

## 7. Regression gates (re-run this session, doc-only change)

No production, test, or Gradle file was touched this session (only this document and its companion
`.wpilog`-analysis commands were added/run) — `git diff --stat -- src/ build.gradle vendordeps/`
returns empty. Gates re-run anyway per this task's own instruction:

- **Gate 1 (`./gradlew compileJava`):** BUILD SUCCESSFUL.
- **Gate 2 (`python SKILLS/run_headless_sim.py --run-seconds 12`):** PASS — robot init reached, 12s
  of disabled-mode periodic loops, no exceptions.
- **No targeted tests re-run** — no test code was touched this session. The two comparison logs used
  in §6 came from this session's own incidental `./gradlew test` invocation (43 tests, 1 failure —
  the same pre-existing `LtNeutralAutoRegressionTest` flaky stall-check signature documented in
  `docs/Autonomous_Observability_Phase1_StageA_Validation.md`'s §5, unrelated to this document's
  scope), not a fresh run performed specifically for this task.

## 8. Limitations

- **Actual AdvantageKit replay (`Logger.setReplaySource`/`WPILOGReader` re-driving robot code from a
  recorded log) was not exercised this session.** It remains blocked exactly as
  `SKILLS/replay-testing-agent.md` already documented — `Constants.simMode` must be `Mode.REPLAY`,
  and it is `Mode.SIM`. Unblocking it requires an explicitly-authorized, single-run,
  edit-Constants.java-then-revert — not performed here since this task did not carry that
  authorization.
- **The two-log comparison validated in §6 used two ordinary `SIM`-mode logs, not two `_sim`-suffixed
  replay outputs.** The comparison mechanics (`parse_akit_log.py --dump` on the same entry name
  against two logs) are identical either way — nothing about `--dump` cares whether its input came
  from `SIM` or `REPLAY` — but this session did not prove the specific case of comparing two replay
  outputs of the same source log under different code revisions, since no replay ran at all.
- **No automated comparison tooling was built, and none should be inferred from this document.** Per
  `/replay`'s own instruction, a repeatedly-needed comparison harness would be a separate,
  explicitly-scoped implementation task — not something to build inside a validation pass.
- **The Finding-4-matching anomaly noted in §6 is reported as a reproducibility observation about the
  comparison method, not a new investigation result.** Diagnosing *why* the severe run's error
  collapses remains exactly as unresolved as `docs/Autonomous_Disturbance_Simulation_Report.md` and
  `docs/Autonomous_Recovery_Readiness_Assessment.md` already left it (most likely a
  `setSimulationWorldPose()` velocity-reset artifact interacting with the chassis-PID bug) — this
  document does not add or subtract confidence in that explanation.

## 9. Stage A acceptance criteria — final status

Per `docs/Autonomous_Observability_Phase1_Plan.md`'s Stage A scope (F1, F3, and the replay/comparison
workflow):

| Item | Status |
|---|---|
| F1 — `Intake.hasGamePiece()` signal validation | **Done** (`docs/Autonomous_Observability_Phase1_StageA_Validation.md`) |
| F3 — `Shooter.isJammed()` signal validation | **Done** (`docs/Autonomous_Observability_Phase1_StageA_Validation.md`) |
| Replay-based validation workflow | **Partially done.** The manual two-log comparison method (the part of the workflow that does not require `REPLAY` mode) is validated end-to-end with real evidence (§6). Actual AdvantageKit replay execution remains blocked on the pre-existing, already-documented `Constants.simMode` prerequisite (§4) and was not authorized to be bypassed this session. |

**Stage A is not fully complete.** The outstanding gap is narrow and specific: someone with authority
over `Constants.java` needs to either (a) explicitly authorize a scoped edit-run-revert to prove real
replay execution once, matching the one precedent already in this project's history (the 2026-07-28 AI
tooling validation pass), or (b) decide the manual-comparison validation in §6 is sufficient evidence
that the workflow's non-replay-dependent half works, and accept the `REPLAY`-mode gap as a standing,
already-documented limitation rather than a Stage A blocker. Neither decision was made in this
session — it is surfaced here for the mentor to choose, not resolved unilaterally.

**Everything else in this document stays within scope:** `AutonomousHealthMonitor.java` was not read
or referenced as a target of any change; no threshold constant was touched; no drivetrain/PID file was
edited; the only files added this session are this document and its now-superseded scratch dump files
(`/tmp/control_lateral.txt`, `/tmp/severe_lateral.txt` — outside the repository, in the session
scratch area, not committed).
