"""Idempotently register (or remove) the supervisor's hook forwarder in Claude
Code's settings.json.

    python install_hooks.py             # install into ~/.claude/settings.json (global)
    python install_hooks.py --project   # install into ./.claude/settings.json (this repo only)
    python install_hooks.py --uninstall # remove our hook entries

The forwarder no-ops unless a session is launched under the supervisor, so a
global install does not disturb ordinary Claude Code sessions.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HOOK_EVENTS = [
    "SessionStart",
    "SessionEnd",
    "UserPromptSubmit",
    "PreToolUse",
    "Notification",
    "Stop",
]

_HERE = Path(__file__).resolve().parent
HOOK_SCRIPT = _HERE / "hook_forward.py"
COMMAND = f'"{sys.executable}" "{HOOK_SCRIPT}"'
MARKER = str(HOOK_SCRIPT)  # identifies our entries for idempotent add/remove


def _entry() -> dict:
    return {"hooks": [{"type": "command", "command": COMMAND}]}


def _is_ours(group: dict) -> bool:
    return any(MARKER in h.get("command", "") for h in group.get("hooks", []))


def install(settings_path: Path) -> None:
    settings = _load(settings_path)
    hooks = settings.setdefault("hooks", {})
    changed = False
    for event in HOOK_EVENTS:
        groups = hooks.setdefault(event, [])
        if not any(_is_ours(g) for g in groups):
            groups.append(_entry())
            changed = True
    if changed:
        _save(settings_path, settings)
        print(f"Installed supervisor hooks into {settings_path}")
    else:
        print(f"Supervisor hooks already present in {settings_path}")
    print("Command:", COMMAND)


def uninstall(settings_path: Path) -> None:
    settings = _load(settings_path)
    hooks = settings.get("hooks", {})
    changed = False
    for event in HOOK_EVENTS:
        groups = hooks.get(event, [])
        kept = [g for g in groups if not _is_ours(g)]
        if len(kept) != len(groups):
            changed = True
        if kept:
            hooks[event] = kept
        else:
            hooks.pop(event, None)
    if changed:
        _save(settings_path, settings)
        print(f"Removed supervisor hooks from {settings_path}")
    else:
        print(f"No supervisor hooks found in {settings_path}")


def _load(path: Path) -> dict:
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))
    return {}


def _save(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        backup = path.with_suffix(path.suffix + ".bak")
        backup.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--project", action="store_true", help="use ./.claude/settings.json")
    ap.add_argument("--uninstall", action="store_true")
    args = ap.parse_args()

    if args.project:
        settings_path = Path.cwd() / ".claude" / "settings.json"
    else:
        settings_path = Path.home() / ".claude" / "settings.json"

    if not HOOK_SCRIPT.exists():
        print(f"ERROR: {HOOK_SCRIPT} not found", file=sys.stderr)
        return 1

    if args.uninstall:
        uninstall(settings_path)
    else:
        install(settings_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
