"""Configuration loading for the supervisor.

Reads config.yaml (PyYAML). Secrets live here and this file is gitignored.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

import yaml


@dataclass
class DiscordConfig:
    bot_token: str
    authorized_user_id: str
    channel_id: int


@dataclass
class TimersConfig:
    initial_timeout_seconds: int = 300
    reminder_interval_seconds: int = 1800


@dataclass
class BehaviorConfig:
    auto_strip_ansi: bool = True
    max_discord_msg_len: int = 1800
    log_level: str = "INFO"
    # Local-input path PtySession uses ("legacy" = today's msvcrt/console-API
    # translation, "vt" = the new VT-input relay backend). See pty_session.py.
    local_input_backend: str = "legacy"


@dataclass
class SupervisorConfig:
    # Local event sink the hook forwarder POSTs to. 127.0.0.1 only.
    host: str = "127.0.0.1"
    port: int = 8787
    # Command launched inside the PTY. The default assumes `claude` is on PATH.
    claude_command: list[str] = field(default_factory=lambda: ["claude"])
    # Rolling terminal buffer size (lines) kept for the !logs command.
    log_buffer_lines: int = 500
    # Keystrokes sent for permission-dialog buttons (Claude Code's TUI is a menu,
    # not a readline y/n prompt -- these are best-effort accept/deny).
    approve_keys: str = "\r"       # Enter -> accept highlighted default (usually "Yes")
    deny_keys: str = "\x1b"        # Esc  -> dismiss / decline
    continue_text: str = "continue"
    # Optional sidecar JSON (relative to the session cwd) the workflow can maintain
    # for current_task / completed / tests, which hooks cannot supply.
    status_file: str = ".claude-supervisor-status.json"


@dataclass
class Config:
    discord: DiscordConfig
    timers: TimersConfig
    behavior: BehaviorConfig
    supervisor: SupervisorConfig
    # Shared secret injected into the child env; the hook forwarder echoes it back
    # so the sink can reject spoofed localhost POSTs.
    ipc_token: str = ""


def load_config(path: str | os.PathLike) -> Config:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(
            f"Config not found: {p}. Copy config.example.yaml to config.yaml and fill it in."
        )
    raw = yaml.safe_load(p.read_text(encoding="utf-8")) or {}

    d = raw.get("discord", {})
    discord = DiscordConfig(
        bot_token=str(d.get("bot_token", "")).strip(),
        authorized_user_id=str(d.get("authorized_user_id", "")).strip(),
        channel_id=int(d.get("channel_id", 0)),
    )
    timers = TimersConfig(**{k: v for k, v in (raw.get("timers") or {}).items()})
    behavior = BehaviorConfig(**{k: v for k, v in (raw.get("behavior") or {}).items()})

    sup_raw = raw.get("supervisor") or {}
    supervisor = SupervisorConfig(**{k: v for k, v in sup_raw.items()})

    cfg = Config(
        discord=discord,
        timers=timers,
        behavior=behavior,
        supervisor=supervisor,
        ipc_token=str(raw.get("ipc_token", "")).strip(),
    )
    _validate(cfg)
    return cfg


def _validate(cfg: Config) -> None:
    problems = []
    if not cfg.discord.bot_token or cfg.discord.bot_token.startswith("YOUR_"):
        problems.append("discord.bot_token is unset")
    if not cfg.discord.authorized_user_id.isdigit():
        problems.append("discord.authorized_user_id must be a numeric Discord user ID")
    if not cfg.discord.channel_id:
        problems.append("discord.channel_id is unset")
    if cfg.behavior.local_input_backend not in ("legacy", "vt"):
        problems.append(
            "behavior.local_input_backend must be 'legacy' or 'vt', got %r"
            % (cfg.behavior.local_input_backend,)
        )
    if problems:
        raise ValueError("Invalid config.yaml: " + "; ".join(problems))
