---
description: End-of-session sync — guard against zero-work, classify changes into local repo memory vs. the 6767 vault, update CLAUDE.md, and write today's session log
---

# /recap — Exit Protocol

Sync this session's work back into the two places that persist: local repo memory (`CLAUDE.md`, `docs/claudex/`) and the central Obsidian vault at `C:\Users\xdm\6767 frc bible`.

Extra context from the user (may be empty): $ARGUMENTS

## Step 0 — Establish what actually happened (evidence, not recall)

Do **not** write the summary from memory of the conversation alone. Gather evidence first:

```bash
git -C "C:/Users/xdm/Claude/experimental-FRC" log --oneline -15
git -C "C:/Users/xdm/Claude/experimental-FRC" status --short
git -C "C:/Users/xdm/Claude/experimental-FRC" diff --stat HEAD
```

Confirm the build state before claiming anything about it:

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"   # PATH java is JDK 25; Gradle 8.11 cannot run it
cd "C:/Users/xdm/Claude/experimental-FRC" && ./gradlew compileJava
```

If `compileJava` was not run this session, say so — do not write "compiling cleanly" on faith.

## Step 1 — Zero-work guardrail

Before writing anything, decide whether there is anything to record. If the diff is empty, no decision was made, and no hypothesis was confirmed or disproven this session:

- **Do not fabricate progress.** Do not refresh the verified-tree line to today's date just to look busy, do not invent a session note, and do not append an empty entry to the changelog.
- Report "nothing to sync — no code change, no decision, no confirmed/disproven hypothesis this session" and stop.

An honest "we ruled out X" or "we confirmed Y is still broken" **is** work worth recording — the guardrail only stops empty churn, not negative results.

## Step 2 — Classify each change: local repo memory vs. vault

Sort what happened into the right home before writing. The two stores have different jobs:

| Change | Home | File |
|---|---|---|
| What to do next / current blocker | Local | `CLAUDE.md` → `## 🕒 Current Task State` (`Next up` / `Backlog`) |
| A completed item + its verification evidence | Local | `docs/claudex/history.md` (append to bottom) |
| A new/revised architecture decision | Local | `docs/claudex/architecture.md` |
| A measured constant, gain, offset, calibration | Vault | the owning topic note (see Step 4 table) |
| A confirmed/disproven hypothesis | Vault | the owning topic note — **delete the hypothesis if it was wrong** |
| A peer-benchmarked pattern | Vault | `Best Practices & Upgrades.md` |
| The narrative log of the session | Vault | `Projects/Session YYYY-MM-DD.md` |

Local memory is the code-adjacent snapshot for the next session; the vault is the cumulative, durable brain. A fact can go to both (e.g. a tuned gain updates `PID Gains Registry.md` *and* is noted in `Next up` if it unblocks work) — but decide deliberately, don't dump everything into one.

## Step 3 — Update local repo memory (`CLAUDE.md`, `docs/claudex/`)

`CLAUDE.md` is a slim current-state snapshot, not the changelog. Edit surgically:
- In `## 🕒 Current Task State`: update **Next up** so the next session knows where to start, add anything new to **Backlog** / **Static Calibrations Needed**, and refresh the `*Verified against the tree on <date> at commit <sha>*` line to the real current date and SHA. Do **not** add a `Done` list here.
- In `docs/claudex/history.md`: append finished items to the **bottom** of the changelog, with the file paths and verification evidence that prove them.
- In `docs/claudex/architecture.md`: update only if the session established or revised an architecture decision. Leave it alone otherwise.

Leave the rest of `CLAUDE.md` alone unless the user asked otherwise.

## Step 4 — Write today's session log to the vault

Create `C:\Users\xdm\6767 frc bible\Projects\Session YYYY-MM-DD.md`.

If a note for today already exists, **append a new section rather than overwriting it** — more than one session can happen in a day.

```markdown
---
type: session
date: YYYY-MM-DD
project: experimental-FRC
commit_start: <sha>
commit_end: <sha>
build: pass | fail | not run
---
# Session YYYY-MM-DD

## What changed
- <change> — `path/to/File.java:line`

## Fixes
- <symptom → root cause → fix>. Link the subsystem note, e.g. [[Shooter]].

## New parameters & hardware quirks
- <constant/gain/offset + value + where it came from>
- Anything measured on the real robot goes here — that is the highest-value content in the vault.

## Peer-benchmarked discoveries
- <pattern seen in 254/1678/6328/Lynk and what we concluded>. Link [[Best Practices & Upgrades]].

## Open questions for next session
- <question>
```

Then propagate durable facts into the owning topic note — a session note is a log, the topic notes are the brain:

| New fact | Goes in |
|---|---|
| A gain changed / was tuned | `Control Loops & Math/PID Gains Registry.md` |
| CANcoder offset / CAN ID / module geometry | `Control Loops & Math/Swerve Offsets & Kinematics.md` |
| Shooter RPM calibration point | `Control Loops & Math/Shooter RPM Regression.md` |
| Sim constant measured or motor confirmed | `WPILib & Simulation/Simulation Setup.md` |
| Alliance / field-space logic | `WPILib & Simulation/Field & Alliance Coordinates.md` |
| Auto, path, or named command | `Auto & Pathing/PathPlanner.md` |
| Architecture debt resolved or found | `Best Practices & Upgrades.md` |
| Hardware mapping | the matching `Subsystems & Hardware/` note |

Add a link to the new session note from the topic note where it adds context.

> Cross-session pattern promotion (spotting the same lesson across *many* session notes and lifting it into `Best Practices & Upgrades.md`) is **`/tldr`'s** job, not this one. `/recap` records today; `/tldr` distills the accumulation.

## Rules

- **Evidence over recall.** Every claim cites a file path, line, commit SHA, or command output. If it wasn't verified, mark it *unverified hypothesis*.
- **A confirmed or disproven hypothesis is the most valuable thing you can record.** Update the owning note and **delete the hypothesis if it was wrong** — stale wrong warnings are worse than none.
- **Never overwrite vault notes wholesale.** Read, then edit surgically. The vault is cumulative.
- **Never copy numbers from reference dirs** (`temp_reference/`, `frc-steal-from-the-best/`) into our notes as if they were ours.
- Team is **4935**. The `6767` in the vault path is a naming meme.

## Finally

Report to the user: which `CLAUDE.md` / `docs/claudex/` files changed, the vault session-note path, which topic notes were touched, and the build state.
