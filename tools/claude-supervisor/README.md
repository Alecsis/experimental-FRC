# Claude Code Remote Supervisor (Discord)

A local, single-user daemon that lets you run long autonomous Claude Code sessions
on Windows 11 and drive them from Discord: it alerts you when Claude goes idle or
hits a checkpoint, and lets you approve / deny / reply / stop from your phone.

> **Trust model:** this owns a live Claude Code session's stdin and lets a Discord
> account inject text and Ctrl+C into it. It is gated on **one** hardcoded Discord
> user ID. Run it only to control **your own** machine, keep `config.yaml` (which
> holds the bot token) out of git, and treat the authorized ID as a credential.

---

## Phase 1 — Integration architecture (the design decision)

The spec asked to evaluate three integration paths in priority order. Here's the
result, and why this implementation is a **hybrid** rather than pure ConPTY:

| Need | Chosen mechanism | Why |
|------|------------------|-----|
| **Detect "waiting for human"** | **Native Claude Code hooks** (priority #1) | The `Notification` event fires exactly on `permission_prompt` / `idle_prompt` / `agent_needs_input` / `agent_completed`, delivered as **structured JSON** (`session_id`, `cwd`, `notification_type`, `message`). `Stop` gives `last_assistant_message`. No ANSI screen-scraping, no heuristic "did output stop?" guessing. |
| **Track session / project / branch** | Hook JSON + local git read | `session_id`, `cwd` come from every hook payload; branch is read from `.git/HEAD`. |
| **Inject input back into the session** | **ConPTY via `pywinpty`** | Hooks are **one-way** — they can block/approve/add-context but *cannot* type into the session. The only way to write to Claude's stdin is to own its PTY, so the supervisor launches `claude` inside a pseudo-terminal. |

**Why not log-file tailing (priority #2)?** The transcript JSONL is written for
completed turns; it lags the live "is a prompt on screen right now" question that
`Notification` answers instantly. Hooks win.

**Detection is decoupled from I/O.** The PTY exists purely to own stdin (for
injection) and to keep a rolling log buffer for `!logs`. State transitions come
from hooks. This means the fragile part (parsing a TUI) never gates correctness.

### Data flow

```
                 hook JSON (POST 127.0.0.1)
  ┌── claude (in PTY) ──► hook_forward.py ──────────────┐
  │        ▲                                            ▼
  │        │ stdin inject                        event sink (aiohttp)
  │        │                                            │
  │   PtySession ◄──── Supervisor (state machine + timers)
  │                                                     │
  └── stdout ──► your console                    Discord bot (alerts + commands)
                                                        ▲
                                              you, from Discord ─┘
```

### How the edge cases are handled

- **ANSI stripping** — `strip_ansi()` removes CSI + OSC sequences before anything
  hits the log buffer or Discord.
- **Discord 2000-char limit** — snippets over `max_discord_msg_len` (default 1800)
  upload as a `claude_logs.txt` attachment instead of a message.
- **Windows line endings / signals** — injection sends `\r` for Enter and `\x03`
  for Ctrl+C; arrow/nav keys from the local console are mapped to ANSI sequences.
- **Generalized checkpoints** — not just yes/no. `Notification.notification_type`
  and a keyword classifier on `Stop` text map to `permission | clarification |
  milestone | review | risk | failure`.
- **ConPTY limits** — local input is proxied best-effort via `msvcrt`; a full TUI
  (mouse, bracketed paste) may not round-trip perfectly. Remote control via hooks +
  injection is unaffected by this.

---

## State machine

```
RUNNING ──(Notification / Stop)──► WAITING_FOR_HUMAN ──(timer expires)──► DISCORD_ALERT_SENT
   ▲                                     │                                      │
   │                                     │ (UserPromptSubmit / PreToolUse)       │ (reminder loop)
   └─────────────────────────────────────┴──────────────────────────────────────┘
                        (Discord command) ──► INJECTING_INPUT ──► RUNNING
```

---

## Setup

### 1. Install dependencies

```powershell
cd C:\Users\xdm\Claude\experimental-FRC\tools\claude-supervisor
python -m pip install -r requirements.txt
```

### 2. Create the Discord bot

1. https://discord.com/developers/applications → New Application → **Bot**.
2. Copy the **bot token**.
3. Under **Privileged Gateway Intents**, enable **Message Content Intent**
   (needed for `!` text commands).
4. Invite the bot to your server with the `bot` scope and **Send Messages** +
   **Read Message History** permissions.
5. In Discord (with Developer Mode on), right-click your target channel → **Copy
   Channel ID**, and right-click your own name → **Copy User ID**.

### 3. Configure

```powershell
copy config.example.yaml config.yaml
```

Edit `config.yaml`: set `bot_token`, `channel_id`, confirm `authorized_user_id`
(your Discord ID — the sole account allowed to inject input), and set `ipc_token`
to any random string.

### 4. Register the hooks

```powershell
python install_hooks.py            # global (~/.claude/settings.json)
# ...or, to scope to this repo only:
python install_hooks.py --project  # ./.claude/settings.json
```

The forwarder **no-ops** unless a session is launched under the supervisor, so a
global install does not affect ordinary `claude` sessions. Remove with
`python install_hooks.py --uninstall`.

### 5. Run

From the project directory you want Claude to work in:

```powershell
python C:\Users\xdm\Claude\experimental-FRC\tools\claude-supervisor\run.py
```

This launches `claude` under supervision. Use it normally in the terminal; when it
idles past `initial_timeout_seconds` you'll get a Discord alert. Exit Claude
normally and the supervisor shuts down. Bot logs go to `supervisor.log`, not the
console (the console is reserved for Claude's TUI).

---

## Discord commands (authorized ID only)

| Command | Effect |
|---------|--------|
| `!status` | State, project, branch, elapsed wait, PID |
| `!approve` / `!continue` | Inject "continue" + Enter |
| `!deny` | Send the configured deny keystroke (Esc) |
| `!reply <text>` | Inject arbitrary text + Enter (multi-line supported) |
| `!logs` | Last 50 lines of cleaned output (code block, or `.txt` if large) |
| `!context` | Fresh context snapshot: repo, branch, completed, current task, changed files, tests, diff, output tail |
| `!pause` / `!resume` | Hold / release the inactivity alert timer |
| `!stop` | Send Ctrl+C to Claude |

Alert messages also carry **Approve / Deny / Logs / Stop** buttons.

> **Permission dialogs are best-effort.** Claude Code's permission prompt is an
> interactive menu, not a readline y/n. `Approve` sends Enter (accept the
> highlighted default) and `Deny` sends Esc. For anything nuanced, use
> `!reply`, which is reliable for the normal "waiting for your message" state.

---

## Context snapshots

Every time Claude enters `WAITING_FOR_HUMAN`, the supervisor captures a snapshot
(git work runs in an executor, off the event loop) and embeds it in the alert.
`!context` returns a fresh one on demand. Collected:

| Field | Source |
|-------|--------|
| Project | `git rev-parse --show-toplevel` basename (fallback: cwd) |
| Branch | `git rev-parse --abbrev-ref HEAD` |
| Changed files | `git status --porcelain` |
| Diff summary | `git diff --stat HEAD` |
| Output tail | last 100 lines of Claude's PTY output (ANSI-stripped) |
| Needs | current `waiting_reason` |
| Question | last prompt / assistant message |
| **Completed** | sidecar `completed`, else recent commit subjects (labelled "Recent commits") |
| **Current task** | sidecar `current_task`, else `—` |
| **Tests** | sidecar `tests`, else heuristic scan of the output tail (`PASS`/`FAIL`/`unknown`) |

The last three can't come from Claude Code hooks, so they're read from an optional
**sidecar file** at `supervisor.status_file` (default `.claude-supervisor-status.json`
in the session cwd). Have your workflow — or Claude itself, via a CLAUDE.md
instruction — keep it current:

```json
{
  "current_task": "Implementing autonomous speed clamp",
  "completed": ["Fixed sim spawn ownership", "Added trajectory telemetry"],
  "tests": "PASS"
}
```

When absent, the snapshot degrades gracefully (commit subjects + output heuristics)
and never fabricates a passing test result — unproven tests read `unknown`.

## Running it in the background as a task

The supervisor must share a console with Claude's TUI, so a fully headless
Windows *service* is not the right fit. To keep it running unattended, launch it
in a dedicated terminal (e.g. Windows Terminal) and, if you want auto-start,
register a **Task Scheduler** task ("At log on", program `python`, arguments the
full path to `run.py`, "Start in" the project dir). Reattach to that terminal to
interact locally; otherwise drive it from Discord.

---

## Verification

- `python tests/test_core.py` — state machine, reason classification, timers,
  reminder loop, injection dispatch, and snapshot refresh (PTY stubbed; no
  Discord/PTY deps needed). **22/22 pass.**
- `python tests/test_snapshot.py` — snapshot collector: test heuristics, sidecar
  overrides, graceful git failure, `!context` rendering. **19/19 pass.**
- `python -m py_compile run.py hook_forward.py install_hooks.py claude_supervisor/*.py`
  — syntax check of every module.
- Live check once configured: start `run.py`, let Claude idle, confirm the Discord
  alert arrives, press **Approve**, confirm Claude resumes. `!logs` should return
  recent output; an unauthorized user's commands should be silently ignored (see
  `supervisor.log`).
