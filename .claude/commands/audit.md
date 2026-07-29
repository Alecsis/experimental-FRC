---
description: Read-only, evidence-backed investigation of robot code, producing an audit document under docs/.
---

# /audit

Investigate a question about the codebase and produce a sourced audit document. Read-only — never modify robot code, tests, Gradle files, or config while auditing.

Extra context from the user (may be empty): $ARGUMENTS

## Approach

Follow this project's audit-first philosophy (see the many `docs/*_Audit.md` precedents — e.g. `docs/Autonomous_Command_Lifecycle_Audit.md`, `docs/Autonomous_Recovery_Audit.md`, `docs/Drive_Slot0_Retuning_Audit.md`): **compile findings before drawing conclusions.** Read the actual source (production code, decompiled vendor sources if a vendor-API question is in play, `.wpilog` evidence via `SKILLS/parse_akit_log.py` — see `SKILLS/log-analysis-agent.md`) rather than reasoning from memory or assumption.

Useful supporting agents, depending on the question:
- `SKILLS/robot-description-agent.md` — physical constants and architecture rules.
- `SKILLS/game-knowledge-agent.md` — auto/field/NamedCommand intent.
- `SKILLS/log-analysis-agent.md` — log-backed evidence for a runtime claim.
- `SKILLS/simulation-agent.md` — if a fresh `.wpilog` is needed to gather evidence.

## Output

- One audit document under `docs/`, named `<Topic>_Audit.md` matching the existing naming convention.
- Every finding cited with a file:line or a concrete piece of evidence (log dump, test output) — not asserted from memory.
- State assumptions and open questions explicitly; don't paper over gaps.
- If the audit's scope implies a fix, note it as a recommendation — do not implement it as part of `/audit`. That is a separate, explicitly-requested task.

## Rules

- Never edit `src/`, `build.gradle`, or vendordeps.
- Never copy numbers out of `temp_reference/` or the external reference corpus (per `CLAUDE.md`) — patterns only, never values.
- If something referenced doesn't exist in this repo, say so — don't invent it (per `CLAUDE.md`'s "Don't Invent Files" rule).
