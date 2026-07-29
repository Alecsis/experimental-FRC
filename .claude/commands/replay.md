---
description: Guide AdvantageKit replay analysis of one or two .wpilog files.
---

# /replay

Analyze a `.wpilog` (or compare two) via AdvantageKit replay. Read-only against robot code.

Extra context from the user (may be empty, ideally names one or two log paths): $ARGUMENTS

## Prerequisite

**As of 2026-07-28 (later session), replay is reachable via the `AKIT_LOG_PATH` environment variable — no `Constants.java` edit needed.** `Constants.currentMode` is derived through `Constants.resolveCurrentMode(isReal, envVar, simMode)`, which returns `Mode.REPLAY` whenever `AKIT_LOG_PATH` is set (matching AdvantageKit's own `LogFileUtil`/`ReplayWatch`, which already treat that variable as their first-priority log source) and falls back to `Constants.simMode` (`Mode.SIM`) otherwise — so every pre-existing invocation that doesn't set the variable is unaffected. Pure-function coverage: `ConstantsReplayModeTest`. See `SKILLS/replay-testing-agent.md`'s Prerequisite section for the verified end-to-end run (console output, produced `_sim` log, `ReplayOutputs` entries) and the historical record of why this was previously dead code.

**Still confirm, don't assume:** check the console for `[AdvantageKit] Replaying log from AKIT_LOG_PATH environment variable: "..."` before claiming a replay ran — a run without `AKIT_LOG_PATH` set still silently falls back to `Mode.SIM`. If that line is absent, say so plainly and stop; do not proceed as if a replay happened or fall back to parsing the input log and calling it a replay result.

## Approach

Follow `SKILLS/replay-testing-agent.md` for the replay mechanics (`Constants.currentMode = REPLAY`, `LogFileUtil.findReplayLog()`, the `replayWatch` Gradle task) and `SKILLS/log-analysis-agent.md` for reading the resulting logs with `SKILLS/parse_akit_log.py`.

**Single log:** replay it, then use `parse_akit_log.py --dump`/`--pose-entry`/`--grep` on the resulting `_sim`-suffixed log to answer the question asked.

**Baseline + modified log both supplied:** **no automated comparison harness exists in this repository.** Do not imply one does. Instead, run the manual workflow:
1. Replay (or directly parse, if no replay is needed) each log independently.
2. Use `parse_akit_log.py --dump <entry>` on the same entry name against both logs.
3. Diff the printed values by hand, or write a small one-off scratchpad script if the comparison needs arithmetic (this repo's own SysId sim-workflow kS/kV fit — `docs/SysId_Sim_Workflow_Validation.md` — is the precedent for a throwaway analysis script; it was never promoted into `SKILLS/`).
4. Report the comparison with both raw values cited, not just a delta claim.

## Output

- The specific answer or comparison the user asked for, with literal command output quoted.
- If comparison tooling is genuinely needed repeatedly, note that as a gap — don't build it inside `/replay` itself; that would be a separate, explicitly-scoped implementation task.

## Rules

- Never edit robot code, tests, or `build.gradle` while replaying — replay itself needs none, since setting `AKIT_LOG_PATH` is sufficient (see Prerequisite above).
- Never claim a replay ran, or imply `Constants.currentMode` reached `REPLAY`, without having actually verified it via the console's `AKIT_LOG_PATH` line — see Prerequisite above.
- Never fabricate a diff or comparison number that wasn't actually computed from both logs.
