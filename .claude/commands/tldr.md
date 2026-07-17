---
description: End-of-session sync — update CLAUDE.md task state and write a session note to the 6767 Obsidian vault
---

# /tldr — Exit Protocol

Sync this session's work back into the two places that persist: local `CLAUDE.md` and the central Obsidian vault at `C:\Users\xdm\6767 frc bible`.

Extra context from the user (may be empty): $ARGUMENTS

## Step 0 — Establish what actually happened

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

## Step 1 — Update `CLAUDE.md`, `docs/claudex/history.md`, and `docs/claudex/architecture.md`

`CLAUDE.md` is a slim current-state snapshot, not the changelog — as of 2026-07-17 the `Done` log and the Verification Loop mechanics live under `docs/claudex/`. Edit surgically:
- In `CLAUDE.md`'s `## 🕒 Current Task State`: update **Next up** so the next session knows where to start, add anything new to **Backlog** / **Static Calibrations Needed**, and refresh the `*Verified against the tree on <date> at commit <sha>*` line to the real current date and SHA. Do **not** add a `Done` list here.
- In `docs/claudex/history.md`: append finished items to the **bottom** of the changelog, with the file paths and verification evidence that prove them. This is where "Done" entries go now.
- In `docs/claudex/architecture.md`: update this if the session established or revised an architecture decision (a new sanctioned exception, a closed vendor leak, a Superstructure rule change, a newly-accepted or newly-closed gap). Leave it alone otherwise.

Leave the rest of `CLAUDE.md` alone unless the user asked otherwise.

## Step 2 — Write the session note to the vault

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

## Step 3 — Propagate to the topic notes

A session note is a log; the topic notes are the brain. If this session established anything durable, update the note that owns it — don't leave the fact stranded in a dated file:

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

Add a link to the new session note from the relevant topic note where it adds context.

## Rules

- **Evidence over recall.** Every claim cites a file path, line, commit SHA, or command output. If it wasn't verified, mark it *unverified hypothesis* — the vault already distinguishes these and that distinction is the point.
- **A hypothesis that got confirmed or disproven is the most valuable thing you can record.** If the BR CANcoder offset, `kA = 50`, the POI hub pair, or the Motion Magic units got tested this session, update the owning note and **delete the hypothesis if it was wrong**. Stale wrong warnings are worse than none.
- **Never overwrite vault notes wholesale.** Read, then edit surgically. The vault is cumulative.
- **Don't invent progress.** If the session was exploration with no code change, write that. An honest "we ruled out X" is real content.
- **Never copy numbers from reference dirs** (`temp_reference/`, `frc-steal-from-the-best/`) into our notes as if they were ours.
- Team is **4935**. The `6767` in the vault path is a naming meme.

## Finally

Report to the user: which `CLAUDE.md` sections changed, the vault note path, which topic notes were touched, and the build state.
