---
description: End-of-session sync for the local repository.
---

# /recap

Synchronize today's work into the repository's local memory.

## Always

- Gather evidence from git/status/tests before writing.
- Update `CLAUDE.md` with only:
  - current state
  - blockers
  - next safe action
- Append completed work to `docs/claudex/history.md`.
- Update `docs/claudex/architecture.md` only if an architectural decision changed.
- Create or append today's session note under `docs/claudex/sessions/YYYY-MM-DD.md`.
- Point `CLAUDE.md`'s `## Session Handoff` section at that note's path — this is what `/bootstrap` reads next time, so it must always name today's note (or the correct dated section if you appended to an existing one).

## Session Note

Include only:

- What changed
- Why it changed
- Verification performed
- Remaining unknowns
- Next safe action

Finish with:

## Handoff

- Current state
- Unverified items
- Next safe step
- Things not to change

## Rules

- Evidence over memory.
- Don't invent work.
- Don't update files unnecessarily.
- Don't touch the Obsidian vault.

Report exactly which files changed.
