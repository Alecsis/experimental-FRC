---
description: Session start — discover the tree state, read the persistent brain in priority order, and report engineering readiness before touching code
---

# /bootstrap — Session Initialization

Establish an accurate mental model of *this* repo at *this* moment before doing any work. Read the persistent state, verify it against the live tree, and surface anything stale. Do **not** start editing until this is done.

Extra context from the user (may be empty): $ARGUMENTS

## Step 0 — Dynamic discovery (never assume the tree matches memory)

The persistent notes describe the tree as it was *last* session. Discover what it is *now* — do not trust the recorded commit/date until you've checked:

```bash
git -C "C:/Users/xdm/Claude/experimental-FRC" log --oneline -10
git -C "C:/Users/xdm/Claude/experimental-FRC" status --short
git -C "C:/Users/xdm/Claude/experimental-FRC" branch --show-current
```

Note the HEAD SHA and whether the tree is clean. You will compare this against the `*Verified against the tree on <date> at commit <sha>*` line in `CLAUDE.md` — if they disagree, uncommitted work or a new commit landed since the last recap, and the recorded task state may be stale.

## Step 1 — Read the persistent brain, in priority order

Read top-down; earlier files set the frame for later ones. Stop reading a branch once it stops being relevant to what the user is here to do.

1. **`CLAUDE.md`** — the current-state snapshot. The `## 🕒 Current Task State` section (`Next up` / `Backlog`) is where the *actionable* present lives. Read the architecture rules and Karpathy guidelines every time.
2. **`docs/claudex/architecture.md`** — the standing decisions: sanctioned exceptions, closed/open vendor leaks, the Superstructure rule. Read before assuming any boundary is or isn't a violation.
3. **`docs/claudex/history.md`** — the changelog. Skim the most recent session entries for evidence behind the current `Next up`. Don't re-read the whole thing; jump to the tail.
4. **`docs/claudex/`** design notes (`sim-timing-determinism.md`, `robotmotor-refactor.md`, `trajectory-error-instrumentation.md`) and **`verification-loop.md`** — read on demand when the task touches their subject.
5. **Vault entry point** `C:\Users\xdm\6767 frc bible\00 Index.md`, then the topic note that owns today's subject (e.g. `Control Loops & Math/PID Gains Registry.md`, `Auto & Pathing/PathPlanner.md`). The vault is the durable brain; `docs/claudex/` is the code-local mirror.

## Step 2 — Construct the mental model

Reconcile what you read with what you discovered in Step 0. Hold these explicitly:

- **Where we are:** branch, HEAD SHA, clean/dirty, and whether that matches `CLAUDE.md`'s verified line.
- **What's in flight:** the top 1–3 `Next up` items and what's blocking each.
- **What's load-bearing but fragile:** open hypotheses, stale goldens, deferred fixes — anything the notes flag as "do not simplify / do not act on the obsolete instruction."
- **The rails:** Strict Hardware Isolation, Singleton subsystems, centralized Superstructure states, "never copy numbers from reference dirs," and the Trust Boundary. These bind every task.

If a note references a file/flag/method, treat it as *last-known* — verify it still exists in `src/` before recommending action on it.

## Step 3 — Engineering readiness output

Report to the user, concisely:

- **Position:** branch @ SHA, tree clean/dirty, and whether that agrees with `CLAUDE.md`'s verified-tree line (call out drift).
- **Ready to resume:** the top `Next up` item(s), stated as a concrete next action.
- **Watch-outs:** any stale golden, deferred fix, open hypothesis, or uncommitted carry-over that would trip up the obvious next step.
- **Build state is unknown** until a gate runs — do not claim the tree compiles from memory. Offer to run gate 1 (`./gradlew compileJava` with `JAVA_HOME=/c/Users/Public/wpilib/2026/jdk`) if the task needs it.

Then stop and wait for direction, unless the user's `$ARGUMENTS` already named the task — in which case proceed into it with the model you just built.

## Rules

- **Discover before you trust.** The recorded SHA/date is a claim to verify, not a fact.
- **Read, don't rewrite.** `/bootstrap` is read-only. It never edits `CLAUDE.md`, the vault, or code.
- **Surface staleness, don't paper over it.** A drifted verified-line or a superseded instruction is the most useful thing you can report.
- Team is **4935**. The `6767` in the vault path is a naming meme, not a team number.
