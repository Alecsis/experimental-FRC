---
description: Guide AdvantageKit replay analysis of one or two .wpilog files.
---

# /replay

Analyze a `.wpilog` (or compare two) via AdvantageKit replay. Read-only against robot code.

Extra context from the user (may be empty, ideally names one or two log paths): $ARGUMENTS

## Prerequisite

Replay infrastructure existing (`Robot.java`'s `case REPLAY:`, `build.gradle`'s `replayWatch` task) does **not** mean replay is executable right now. Both are gated on `Constants.currentMode` resolving to `Mode.REPLAY`, which in turn is derived from `Constants.simMode` — a hardcoded field in `Constants.java`. **Before doing anything else, check what `Constants.java`'s `simMode` is currently set to.** If it is not `Mode.REPLAY`, replay cannot run as-is: setting `AKIT_LOG_PATH` alone does nothing (`replayWatch` just re-runs `simulateJava` with that env var set; it never touches `Constants.java`), confirmed empirically during this workflow's own validation pass. See `SKILLS/replay-testing-agent.md`'s Prerequisite section for the full detail and the one precedent for a scoped, explicitly-authorized edit-run-revert.

**If REPLAY cannot be activated, say so plainly and stop** — do not proceed as if a replay happened, do not silently fall back to parsing the input log and call it a replay result. `Constants.java` is production code; do not edit it without the user's explicit authorization for that specific run, and always revert the edit immediately afterward if they do authorize it, confirming a clean `git diff` before finishing.

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

- Never edit robot code, tests, or `build.gradle` while replaying — the one narrow exception is a `Constants.java` `simMode` toggle the user has explicitly authorized for a single run, immediately reverted afterward with a confirmed clean `git diff`.
- Never claim a replay ran, or imply `Constants.currentMode` reached `REPLAY`, without having actually verified it — see Prerequisite above.
- Never fabricate a diff or comparison number that wasn't actually computed from both logs.
