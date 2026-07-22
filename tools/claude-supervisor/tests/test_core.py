"""Behavioral test of the supervisor state machine, PTY stubbed.

Runs without discord.py / pywinpty installed -- exercises the RUNNING <->
WAITING_FOR_HUMAN <-> DISCORD_ALERT_SENT logic, reason classification, and
injection dispatch. Run: python tests/test_core.py
"""

from __future__ import annotations

import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from claude_supervisor.config import (
    BehaviorConfig,
    Config,
    DiscordConfig,
    SupervisorConfig,
    TimersConfig,
)
from claude_supervisor.core import Supervisor
from claude_supervisor.state import SessionState, WaitReason


class FakePty:
    def __init__(self):
        self.pid = 4242
        self.calls = []

    def send_text_line(self, t):
        self.calls.append(("text", t))

    def send_keys(self, k):
        self.calls.append(("keys", k))

    def send_ctrl_c(self):
        self.calls.append(("ctrl_c", None))

    def recent_logs(self, n=50):
        return "log-tail"


class FakeAlerter:
    def __init__(self):
        self.alerts = []

    async def send_alert(self, meta, snapshot=None, *, reminder=False):
        self.alerts.append((meta.current_state, meta.waiting_reason, reminder))


def make_cfg(timeout=0.05, reminder=0.05):
    return Config(
        discord=DiscordConfig("tok", "878806618714820638", 123),
        timers=TimersConfig(initial_timeout_seconds=timeout, reminder_interval_seconds=reminder),
        behavior=BehaviorConfig(),
        supervisor=SupervisorConfig(),
        ipc_token="",
    )


PASS = 0
FAIL = 0


def check(cond, label):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  PASS: {label}")
    else:
        FAIL += 1
        print(f"  FAIL: {label}")


async def test_notification_alert_flow():
    print("test_notification_alert_flow")
    loop = asyncio.get_running_loop()
    pty, alerter = FakePty(), FakeAlerter()
    sup = Supervisor(make_cfg(), pty, loop, enable_snapshots=False)
    sup.set_alerter(alerter)

    await sup.handle_event({"hook_event_name": "Notification",
                            "notification_type": "permission_prompt",
                            "message": "Allow rm -rf?", "session_id": "s1"})
    check(sup.meta.current_state == SessionState.WAITING_FOR_HUMAN, "enters WAITING on Notification")
    check(sup.meta.waiting_reason == WaitReason.PERMISSION, "permission_prompt -> PERMISSION reason")

    await asyncio.sleep(0.15)
    check(sup.meta.current_state == SessionState.DISCORD_ALERT_SENT, "fires alert after timeout")
    check(len(alerter.alerts) >= 1 and alerter.alerts[0][2] is False, "first alert is not a reminder")

    await asyncio.sleep(0.12)
    check(any(a[2] is True for a in alerter.alerts), "reminder ping fires while alerted")
    sup._cancel_timers()


async def test_resume_cancels_alert():
    print("test_resume_cancels_alert")
    loop = asyncio.get_running_loop()
    pty, alerter = FakePty(), FakeAlerter()
    sup = Supervisor(make_cfg(timeout=0.2), pty, loop, enable_snapshots=False)
    sup.set_alerter(alerter)

    await sup.handle_event({"hook_event_name": "Stop", "last_assistant_message": "Task 1 done."})
    check(sup.meta.current_state == SessionState.WAITING_FOR_HUMAN, "Stop -> WAITING")
    check(sup.meta.waiting_reason == WaitReason.MILESTONE, "'Task 1 done' -> MILESTONE")

    await sup.handle_event({"hook_event_name": "UserPromptSubmit"})
    check(sup.meta.current_state == SessionState.RUNNING, "UserPromptSubmit -> RUNNING")

    await asyncio.sleep(0.3)
    check(len(alerter.alerts) == 0, "no alert fired after local resume")


async def test_reason_classification():
    print("test_reason_classification")
    loop = asyncio.get_running_loop()
    sup = Supervisor(make_cfg(timeout=10), FakePty(), loop, enable_snapshots=False)
    sup.set_alerter(FakeAlerter())
    cases = [
        ("Build failed with 3 errors.", WaitReason.FAILURE),
        ("Please review the diff before continuing.", WaitReason.REVIEW),
        ("This affects 45 files. Proceed?", WaitReason.RISK),
        ("Which approach should I use?", WaitReason.CLARIFICATION),
    ]
    for msg, expected in cases:
        await sup.handle_event({"hook_event_name": "Stop", "last_assistant_message": msg})
        check(sup.meta.waiting_reason == expected, f"{expected.value}: {msg[:24]!r}")
        sup._to_running()


async def test_injection_dispatch():
    print("test_injection_dispatch")
    loop = asyncio.get_running_loop()
    pty = FakePty()
    sup = Supervisor(make_cfg(timeout=10), pty, loop, enable_snapshots=False)
    sup.set_alerter(FakeAlerter())

    sup.inject_text("go left then shoot")
    check(("text", "go left then shoot") in pty.calls, "reply -> send_text_line")
    check(sup.meta.current_state == SessionState.INJECTING_INPUT, "inject -> INJECTING_INPUT")

    sup.approve()
    check(("keys", "\r") in pty.calls, "approve -> Enter keys")
    sup.deny()
    check(("keys", "\x1b") in pty.calls, "deny -> Esc keys")
    sup.stop_claude()
    check(("ctrl_c", None) in pty.calls, "stop -> Ctrl+C")


async def test_snapshot_refresh():
    print("test_snapshot_refresh")
    loop = asyncio.get_running_loop()
    pty = FakePty()
    sup = Supervisor(make_cfg(timeout=10), pty, loop)  # snapshots enabled
    sup.set_alerter(FakeAlerter())

    snap = await sup.refresh_snapshot()
    check(snap is not None, "refresh returns a snapshot")
    check(sup.last_snapshot is snap, "last_snapshot stored")
    check(bool(snap.project), "project populated from git")
    check(snap.output_tail == "log-tail", "output tail pulled from PTY buffer")


async def main():
    for t in (test_notification_alert_flow, test_resume_cancels_alert,
              test_reason_classification, test_injection_dispatch,
              test_snapshot_refresh):
        await t()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
