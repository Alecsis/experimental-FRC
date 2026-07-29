# Replay Testing Agent

## Purpose
Re-run a previously recorded `.wpilog` through AdvantageKit's replay mode so logged inputs drive the robot code again deterministically, instead of live hardware/sim I/O.

## When to use
- Investigating a bug seen in a real or sim run without needing to reproduce it live.
- Re-deriving telemetry (e.g. re-running an analysis) from a log already on disk.
- **Not** for A/B comparison of two behaviors unless you're prepared to do the diffing manually — see Safety constraints.

## Prerequisite — verify REPLAY is actually reachable before claiming replay is available

**As of 2026-07-28 (later session), REPLAY is reachable without any source edit.**
`Constants.currentMode` now resolves via `Constants.resolveCurrentMode(isReal, envVar, simMode)`,
which returns `Mode.REPLAY` whenever the `AKIT_LOG_PATH` environment variable is set (and the robot
isn't real) — matching AdvantageKit's own `LogFileUtil.findReplayLog()`/`ReplayWatch`, which already
treat that exact variable as their first-priority log source. `Constants.simMode` (still `Mode.SIM`)
is only the fallback when `AKIT_LOG_PATH` is unset, so every existing sim/test invocation is
unaffected. Pure-function coverage: `ConstantsReplayModeTest`. **This supersedes the older finding
below, which is kept only as the historical record of why the fix was needed.**

**How to run a real replay today:** set `AKIT_LOG_PATH` to an existing log's path, then run
`simulateJava` (directly or via `replayWatch`) — no `Constants.java` edit needed. Example, verified
working this session:
```
AKIT_LOG_PATH="logs/akit_26-07-28_23-23-02.wpilog" ./gradlew.bat simulateJava -Pheadless=true --console=plain --no-daemon
```
Console confirms the branch was actually taken (not assumed): `[AdvantageKit] Replaying log from
AKIT_LOG_PATH environment variable: "logs/akit_26-07-28_23-23-02.wpilog"`, followed by
`[AdvantageKit] Logging to "logs\akit_26-07-28_23-23-02_sim.wpilog"`. The process exits on its own
(`System.exit(0)`, from `Logger.java`'s replay-exhausted path) once the source log is fully replayed
— no manual kill needed. The resulting `_sim` log contains both the original `/RealOutputs/...`
entries (re-emitted from the replayed input) and new `/ReplayOutputs/...` entries (e.g.
`ReplayOutputs/LoggedRobot/FullCycleMS`) proving the robot code actually re-executed against the
replayed data, not just copied the file.

**Historical finding (why this was broken until the above fix):** `Robot.java` having a
`case REPLAY:` branch and `build.gradle` having a `replayWatch` task were not, by themselves, proof
that replay would run. Both were gated on `Constants.currentMode` actually resolving to
`Mode.REPLAY` at runtime, and `currentMode` was derived from `Constants.simMode` alone — a hardcoded
`public static final` field with no environment-variable consultation anywhere in the class. With
`simMode = Mode.SIM`, `case REPLAY:` was dead code: setting `AKIT_LOG_PATH` alone did nothing,
confirmed empirically — the run hit `case SIM:` and wrote a fresh log instead of replaying the input
one. Fixing this required a small, explicitly-scoped production-code change to
`Constants.java` (not a one-off edit-run-revert) — done via TDD, zero real/sim/autonomous behavior
change since the env var is unset in every pre-existing invocation path.

## Required tools
- `Constants.currentMode = REPLAY` (reached automatically when `AKIT_LOG_PATH` is set — see
  Prerequisite above) — the `REPLAY` case calls `setUseTiming(false)` (runs as fast as possible),
  `Logger.setReplaySource(new WPILOGReader(logPath))`, and writes a new log via
  `WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))`.
- `LogFileUtil.findReplayLog()` — AdvantageKit's own file picker; how it resolves "which log" depends on its standard prompt/argument behavior (see AdvantageKit docs), not anything custom in this repo.
- The `replayWatch` Gradle task (`build.gradle`): `task(replayWatch, type: JavaExec) { mainClass = "org.littletonrobotics.junction.ReplayWatch"; classpath = sourceSets.main.runtimeClasspath }` — AdvantageKit's log-watching/replay-loop utility, run via `./gradlew replayWatch`.
- `SKILLS/parse_akit_log.py` to inspect either the source log or the `_sim`-suffixed replay output afterward — see log-analysis-agent.md.

## Expected inputs
- An existing `.wpilog` under `logs/` (produced by a REAL, SIM, or prior REPLAY run).

## Expected outputs
- A second `.wpilog` (the original path with a `_sim` suffix) containing whatever `Logger.recordOutput` calls execute during the replay.
- No live hardware or physics simulation runs during replay — only the logged inputs are replayed; anything not captured in the original log (e.g. a code path that depends on new instrumentation not on the original run) will not exercise correctly.

## Safety constraints
- **Verify `AKIT_LOG_PATH` was actually set (and the console printed the "Replaying log from AKIT_LOG_PATH" line) before claiming a replay ran** — see Prerequisite above. `Constants.currentMode` now reaches `REPLAY` automatically when that env var is set, but a run without it still silently falls back to `Mode.SIM`; confirm from the console/log, don't assume.
- **No baseline-vs-modified comparison harness exists in this repository today.** There is no script that runs two logs (or two code revisions replaying the same log) and diffs the results automatically. If a comparison is needed, it must be done manually: replay once per code revision, producing two `_sim` logs, then use `parse_akit_log.py --dump <entry>` on each and compare the printed values by hand (or with a one-off throwaway script, as was done for the SysId sim-workflow kS/kV fits — see `docs/SysId_Sim_Workflow_Validation.md`). Do not imply an automated comparison tool exists.
- Replay never touches real hardware — safe to run against any log, including ones captured on a real robot.
- Do not treat a clean replay as proof of correctness beyond what the original log already captured; replay re-executes logged inputs, it does not re-derive ground truth.
