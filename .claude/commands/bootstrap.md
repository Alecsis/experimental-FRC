---
description: Rebuild context after a compaction or new session — read-only, no recap, no implementation.
---

# /bootstrap — Session Initialization

Rebuild Claude's context after a compaction or a new session. That is the *only* job — this is not a recap, it does not reconstruct months of history, and it does not touch any files.

Extra context from the user (may be empty): $ARGUMENTS

## Step 0 — Where is the tree right now

```bash
git -C "C:/Users/xdm/Claude/experimental-FRC" log --oneline -5
git -C "C:/Users/xdm/Claude/experimental-FRC" status --short
git -C "C:/Users/xdm/Claude/experimental-FRC" branch --show-current
```

Note the HEAD SHA and clean/dirty state. Compare against what the latest session note claims — if they disagree, something changed since the last `/recap`.

## Step 1 — Read, in this order, and stop as soon as you have enough

1. **`CLAUDE.md`** — read it fully, including the `## Session Handoff` pointer to the latest session note.
2. **The latest session note** at the path `CLAUDE.md` just pointed to (`docs/claudex/sessions/YYYY-MM-DD.md`).
3. **That note's `## Handoff` section specifically** — this is the source of truth for "where things stand," not the narrative above it.
4. **`docs/claudex/architecture.md`** — only if the handoff or the task references an architecture decision.
5. **`docs/claudex/verification-loop.md`** — only if the task involves claiming something is verified/passing.
6. **`docs/claudex/history.md`** — only if the handoff is insufficient and you genuinely need older context. This is the exception, not the default step — do not read it top-to-bottom every session.

## Step 2 — Summarize

Report to the user, concisely:

- **Current objective** — what the latest handoff says is in flight.
- **Current branch / state** — SHA, clean/dirty, and whether it agrees with the session note (call out drift).
- **Verified facts** — what the handoff marked as confirmed/tested.
- **Remaining unknowns** — what the handoff marked unverified.
- **Next safe action** — the handoff's stated next step, plus anything it flagged as "avoid" / "don't change."

## Rules

- **Do not modify any files.** Not `CLAUDE.md`, not the vault, not code.
- **Do not begin implementation.** Stop after the summary and wait for the next instruction — unless `$ARGUMENTS` already named the task, in which case proceed into it with the model you just built.
- **Don't re-derive `/recap`'s job.** If the handoff is missing or looks stale, say so — don't fall back to reconstructing state from `history.md` by default.
- Team is **4935**. The `6767` in the vault path is a naming meme, not a team number.
