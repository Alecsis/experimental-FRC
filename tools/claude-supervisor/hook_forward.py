#!/usr/bin/env python
"""Claude Code hook forwarder -- stdlib only, zero dependencies, fast to start.

Registered on Notification/Stop/UserPromptSubmit/PreToolUse/SessionStart/SessionEnd.
Reads the hook JSON on stdin and POSTs it to the supervisor's localhost sink.

SAFE TO REGISTER GLOBALLY: if CLAUDE_SUPERVISOR_PORT is not set (i.e. Claude was
NOT launched under the supervisor), this exits 0 immediately and does nothing, so
ordinary Claude Code sessions are unaffected.

Always exits 0 -- a supervisor that is down must never break the Claude session.
"""

import json
import os
import sys
import urllib.request


def main() -> int:
    port = os.environ.get("CLAUDE_SUPERVISOR_PORT")
    if not port:
        return 0  # not running under the supervisor -> no-op

    host = os.environ.get("CLAUDE_SUPERVISOR_HOST", "127.0.0.1")
    token = os.environ.get("CLAUDE_SUPERVISOR_TOKEN", "")

    try:
        raw = sys.stdin.read()
    except Exception:
        return 0
    if not raw.strip():
        return 0

    # Pass the hook JSON through untouched (adds nothing, parses defensively).
    try:
        payload = json.loads(raw)
    except Exception:
        payload = {"hook_event_name": "Unknown", "_raw": raw[:2000]}

    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        f"http://{host}:{port}/event",
        data=data,
        headers={"Content-Type": "application/json", "X-Supervisor-Token": token},
        method="POST",
    )
    try:
        urllib.request.urlopen(req, timeout=2).read()
    except Exception:
        pass  # supervisor down / busy -> never disturb the session
    return 0


if __name__ == "__main__":
    sys.exit(main())
