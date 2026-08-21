"""State machine primitives and session metadata for the Claude Code supervisor.

Kept dependency-free (stdlib only) so it can be imported from tests or the hook
forwarder without pulling in discord.py / aiohttp.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Optional


class SessionState(str, Enum):
    """The supervisor's view of what the wrapped Claude Code session is doing."""

    RUNNING = "RUNNING"
    WAITING_FOR_HUMAN = "WAITING_FOR_HUMAN"
    DISCORD_ALERT_SENT = "DISCORD_ALERT_SENT"
    INJECTING_INPUT = "INJECTING_INPUT"


class WaitReason(str, Enum):
    """Why Claude is waiting. Mirrors the spec's generalized-checkpoint list."""

    PERMISSION = "permission"
    CLARIFICATION = "clarification"
    MILESTONE = "milestone"
    REVIEW = "review"
    RISK = "risk"
    FAILURE = "failure"


# Claude Code Notification.notification_type -> WaitReason
NOTIFICATION_REASON = {
    "permission_prompt": WaitReason.PERMISSION,
    "idle_prompt": WaitReason.CLARIFICATION,
    "agent_needs_input": WaitReason.CLARIFICATION,
    "elicitation_dialog": WaitReason.CLARIFICATION,
    "agent_completed": WaitReason.MILESTONE,
}


def classify_message(text: Optional[str]) -> WaitReason:
    """Best-effort reason classification from free text (Stop's final message).

    Deliberately conservative keyword matching. Claude does not only ask yes/no
    questions, so we spread across the full reason set instead of assuming a
    permission prompt.
    """
    if not text:
        return WaitReason.MILESTONE
    t = text.lower()
    if any(k in t for k in ("build failed", "error", "failed with", "exception", "traceback")):
        return WaitReason.FAILURE
    if any(k in t for k in ("review", "please review", "diff", "look over")):
        return WaitReason.REVIEW
    if any(k in t for k in ("affects", "proceed?", "are you sure", "destructive", "overwrite")):
        return WaitReason.RISK
    if any(k in t for k in ("permission", "allow claude", "grant")):
        return WaitReason.PERMISSION
    if any(k in t for k in ("which", "should i", "two approaches", "clarif", "?")):
        return WaitReason.CLARIFICATION
    return WaitReason.MILESTONE


@dataclass
class SessionMetadata:
    """Tracked per the spec's session-metadata schema."""

    session_id: str = ""
    pid: int = 0
    project_name: str = ""
    git_branch: str = ""
    working_directory: str = ""
    current_state: SessionState = SessionState.RUNNING
    waiting_reason: Optional[WaitReason] = None
    last_prompt_text: str = ""
    wait_start_timestamp: Optional[str] = None  # ISO-8601

    # Non-spec bookkeeping.
    wait_start_monotonic: Optional[float] = field(default=None, repr=False)
    machine: str = ""

    def to_json(self) -> dict:
        d = asdict(self)
        d["current_state"] = self.current_state.value
        d["waiting_reason"] = self.waiting_reason.value if self.waiting_reason else None
        d.pop("wait_start_monotonic", None)
        return d

    def waiting_elapsed_seconds(self) -> float:
        if self.wait_start_monotonic is None:
            return 0.0
        return max(0.0, time.monotonic() - self.wait_start_monotonic)
