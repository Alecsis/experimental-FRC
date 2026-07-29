# Replay Testing Agent

## Purpose
Re-run a previously recorded `.wpilog` through AdvantageKit's replay mode so logged inputs drive the robot code again deterministically, instead of live hardware/sim I/O.

## When to use
- Investigating a bug seen in a real or sim run without needing to reproduce it live.
- Re-deriving telemetry (e.g. re-running an analysis) from a log already on disk.
- **Not** for A/B comparison of two behaviors unless you're prepared to do the diffing manually — see Safety constraints.

## Prerequisite — verify REPLAY is actually reachable before claiming replay is available

`Robot.java` having a `case REPLAY:` branch and `build.gradle` having a `replayWatch` task are **not**
proof that replay will run. Both are gated on `Constants.currentMode` actually resolving to
`Mode.REPLAY` at runtime, and in this repo `currentMode` is derived from `Constants.simMode`
(`Constants.java`) — a hardcoded `public static final` field. As of 2026-07-28, `simMode = Mode.SIM`,
so `case REPLAY:` is **dead code**: setting the `AKIT_LOG_PATH` environment variable alone (what
`replayWatch`/`ReplayWatch.launchReplay()` actually does — it just re-runs `simulateJava` with that
env var set, it does not touch `Constants.java`) does nothing, confirmed empirically — the run still
hits `case SIM:` and writes a fresh empty log instead of replaying the input one.

**Before claiming a replay ran or is available:** confirm `Constants.currentMode` will actually
evaluate to `Mode.REPLAY` for the run in question — in practice, that today means confirming
`Constants.java`'s `simMode` field is currently set to `Mode.REPLAY`, since nothing else flips it.
If it is not, **do not silently work around this or claim replay executed** — report the blocker
honestly: replay infrastructure exists but is not currently reachable without a source change to
`Constants.java`, and changing that file is a production-code edit outside this agent's/`/replay`'s
own read-only scope unless the user explicitly authorizes a scoped, reverted edit-run-revert (as was
done once, with explicit approval, during the 2026-07-28 AI tooling validation pass — see
`docs/claudex/sessions/2026-07-28.md`). This preserves the same "do not fabricate tooling output"
philosophy as `log-analysis-agent.md`'s own historical-bug warning below — a `_sim`-suffixed log that
was never actually produced by a real replay must never be described as if it was.

## Required tools
- `Constants.currentMode = REPLAY` (or the equivalent robot-mode switch already wired in `Robot.java`) — the `REPLAY` case calls `setUseTiming(false)` (runs as fast as possible), `Logger.setReplaySource(new WPILOGReader(logPath))`, and writes a new log via `WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))`. **This branch is only reached if `Constants.currentMode` resolves to `REPLAY` — see Prerequisite above; do not assume it does.**
- `LogFileUtil.findReplayLog()` — AdvantageKit's own file picker; how it resolves "which log" depends on its standard prompt/argument behavior (see AdvantageKit docs), not anything custom in this repo.
- The `replayWatch` Gradle task (`build.gradle`): `task(replayWatch, type: JavaExec) { mainClass = "org.littletonrobotics.junction.ReplayWatch"; classpath = sourceSets.main.runtimeClasspath }` — AdvantageKit's log-watching/replay-loop utility, run via `./gradlew replayWatch`.
- `SKILLS/parse_akit_log.py` to inspect either the source log or the `_sim`-suffixed replay output afterward — see log-analysis-agent.md.

## Expected inputs
- An existing `.wpilog` under `logs/` (produced by a REAL, SIM, or prior REPLAY run).

## Expected outputs
- A second `.wpilog` (the original path with a `_sim` suffix) containing whatever `Logger.recordOutput` calls execute during the replay.
- No live hardware or physics simulation runs during replay — only the logged inputs are replayed; anything not captured in the original log (e.g. a code path that depends on new instrumentation not on the original run) will not exercise correctly.

## Safety constraints
- **Verify `Constants.currentMode` actually resolves to `Mode.REPLAY` before claiming a replay ran or is available** — see Prerequisite above. Do not assume the presence of `case REPLAY:`/`replayWatch` means replay is executable right now.
- **No baseline-vs-modified comparison harness exists in this repository today.** There is no script that runs two logs (or two code revisions replaying the same log) and diffs the results automatically. If a comparison is needed, it must be done manually: replay once per code revision, producing two `_sim` logs, then use `parse_akit_log.py --dump <entry>` on each and compare the printed values by hand (or with a one-off throwaway script, as was done for the SysId sim-workflow kS/kV fits — see `docs/SysId_Sim_Workflow_Validation.md`). Do not imply an automated comparison tool exists.
- Replay never touches real hardware — safe to run against any log, including ones captured on a real robot.
- Do not treat a clean replay as proof of correctness beyond what the original log already captured; replay re-executes logged inputs, it does not re-derive ground truth.
