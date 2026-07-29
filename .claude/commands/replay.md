---
description: Guide AdvantageKit replay analysis of one or two .wpilog files.
---

# /replay

Analyze a `.wpilog` (or compare two) via AdvantageKit replay. Read-only against robot code.

Extra context from the user (may be empty, ideally names one or two log paths): $ARGUMENTS

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

- Never edit robot code, tests, or `build.gradle` while replaying.
- Never fabricate a diff or comparison number that wasn't actually computed from both logs.
