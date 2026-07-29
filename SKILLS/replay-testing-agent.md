# Replay Testing Agent

## Purpose
Re-run a previously recorded `.wpilog` through AdvantageKit's replay mode so logged inputs drive the robot code again deterministically, instead of live hardware/sim I/O.

## When to use
- Investigating a bug seen in a real or sim run without needing to reproduce it live.
- Re-deriving telemetry (e.g. re-running an analysis) from a log already on disk.
- **Not** for A/B comparison of two behaviors unless you're prepared to do the diffing manually — see Safety constraints.

## Required tools
- `Constants.currentMode = REPLAY` (or the equivalent robot-mode switch already wired in `Robot.java`) — the `REPLAY` case calls `setUseTiming(false)` (runs as fast as possible), `Logger.setReplaySource(new WPILOGReader(logPath))`, and writes a new log via `WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))`.
- `LogFileUtil.findReplayLog()` — AdvantageKit's own file picker; how it resolves "which log" depends on its standard prompt/argument behavior (see AdvantageKit docs), not anything custom in this repo.
- The `replayWatch` Gradle task (`build.gradle`): `task(replayWatch, type: JavaExec) { mainClass = "org.littletonrobotics.junction.ReplayWatch"; classpath = sourceSets.main.runtimeClasspath }` — AdvantageKit's log-watching/replay-loop utility, run via `./gradlew replayWatch`.
- `SKILLS/parse_akit_log.py` to inspect either the source log or the `_sim`-suffixed replay output afterward — see log-analysis-agent.md.

## Expected inputs
- An existing `.wpilog` under `logs/` (produced by a REAL, SIM, or prior REPLAY run).

## Expected outputs
- A second `.wpilog` (the original path with a `_sim` suffix) containing whatever `Logger.recordOutput` calls execute during the replay.
- No live hardware or physics simulation runs during replay — only the logged inputs are replayed; anything not captured in the original log (e.g. a code path that depends on new instrumentation not on the original run) will not exercise correctly.

## Safety constraints
- **No baseline-vs-modified comparison harness exists in this repository today.** There is no script that runs two logs (or two code revisions replaying the same log) and diffs the results automatically. If a comparison is needed, it must be done manually: replay once per code revision, producing two `_sim` logs, then use `parse_akit_log.py --dump <entry>` on each and compare the printed values by hand (or with a one-off throwaway script, as was done for the SysId sim-workflow kS/kV fits — see `docs/SysId_Sim_Workflow_Validation.md`). Do not imply an automated comparison tool exists.
- Replay never touches real hardware — safe to run against any log, including ones captured on a real robot.
- Do not treat a clean replay as proof of correctness beyond what the original log already captured; replay re-executes logged inputs, it does not re-derive ground truth.
