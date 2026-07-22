---
description: Distillation pass — scan un-promoted session notes, promote recurring patterns into Best Practices & Upgrades.md, and perform vault maintenance
---

# /tldr — Distillation

Lift the accumulated, per-session logs into durable knowledge. Where `/recap` records *one* session, `/tldr` reads *across* sessions, promotes the patterns that have recurred enough to be worth institutionalizing, and cleans up the vault. Run this periodically — not every session.

Extra context from the user (may be empty): $ARGUMENTS

## Step 0 — Scan for un-promoted session notes

Find the session logs that haven't yet been distilled. Read the tail of `C:\Users\xdm\6767 frc bible\Projects\Session *.md` — newest first — and the recent entries of `docs/claudex/history.md`.

```bash
ls -t "C:/Users/xdm/6767 frc bible/Projects/" | head -20
```

Establish a watermark: what's the last date already reflected in `Best Practices & Upgrades.md`? Prefer an explicit marker in that file (e.g. a `<!-- distilled through YYYY-MM-DD -->` line) over guessing. Distill only what's newer than the watermark; do not re-promote what's already there.

If nothing is un-promoted, say so and stop — this is a real outcome, not a failure.

## Step 1 — Find the patterns worth promoting

A single session note stays a log. Promote a fact to `Best Practices & Upgrades.md` only when it has earned it:

- **It recurred.** The same class of bug, workaround, or peer-benchmarked pattern shows up across two or more sessions (e.g. the injection-pattern watch, the JDK-25/Gradle env fault, the "never copy reference numbers" trap).
- **It's a standing rule now.** A one-off decision has hardened into "we always do it this way" (a sanctioned exception, a verification-gate requirement, an architecture boundary).
- **It's a confirmed calibration or constant** that other work will depend on — those may already have a topic-note home (PID Gains Registry, Swerve Offsets); promote the *lesson*, and cross-link the number rather than copying it.

Do **not** promote raw session narrative, still-open hypotheses, or anything unverified. Distillation is for what's been proven and repeated.

## Step 2 — Promote into `Best Practices & Upgrades.md`

Read the file first, then edit surgically — it is cumulative and peer-facing.

- Add each promoted pattern as a concise entry: the rule, the evidence (session date / commit SHA / file path), and a `[[link]]` to the owning topic note or session log.
- If a new pattern refines or supersedes an existing entry, **edit that entry in place** — don't stack a contradictory duplicate.
- Advance the distilled-through watermark to the newest session you processed.

## Step 3 — Vault maintenance

While you have the whole vault in view, tidy it:

- **Dead links:** check `[[wikilinks]]` in the notes you touched resolve to real files. Fix or remove danglers (a link to a note that was renamed, a session that was never written).
- **Stale hypotheses:** a hypothesis that a later session confirmed or disproved should no longer sit unqualified in a topic note. Resolve it — **delete it if it was wrong.** Stale wrong warnings are worse than none.
- **Index hygiene:** ensure `00 Index.md` still points at the key topic notes, and that newly-significant notes are reachable from it.
- **Duplication:** if two topic notes now cover the same fact, consolidate into the owning note and leave a link from the other.

Keep edits surgical. Never rewrite a note wholesale — the vault's value is its cumulative, edited-in-place history.

## Rules

- **Promote proven, recurring patterns only.** One-offs and open hypotheses stay in session notes / topic notes; `/tldr` is the high bar.
- **Evidence travels with the pattern.** Every promoted entry cites the session date, SHA, or file path that earned it.
- **Edit in place; never overwrite.** Both `Best Practices & Upgrades.md` and topic notes are cumulative.
- **Never copy numbers from reference dirs** (`temp_reference/`, `frc-steal-from-the-best/`) into the vault as if they were ours — promote the pattern, cross-link the number.
- `/recap` writes today; `/tldr` distills the accumulation. Don't do `/recap`'s per-session logging here.
- Team is **4935**. The `6767` in the vault path is a naming meme.

## Finally

Report to the user: which session notes were scanned, which patterns were promoted into `Best Practices & Upgrades.md` (with their evidence), what vault maintenance was performed (dead links fixed, hypotheses resolved, index updates), and the new distilled-through watermark.
