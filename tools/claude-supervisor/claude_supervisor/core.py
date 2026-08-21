"""Supervisor core: state machine, inactivity timers, and the local event sink.

Hook events arrive as JSON POSTs on 127.0.0.1 (from hook_forward.py running
inside the Claude child). They drive the RUNNING <-> WAITING_FOR_HUMAN <->
DISCORD_ALERT_SENT transitions. Injection flows the other way, from the Discord
bot into the PTY.
"""

from __future__ import annotations

import asyncio
import datetime as _dt
import logging
import os
import socket
import time
from pathlib import Path
from typing import Optional, Protocol

from aiohttp import web

from .config import Config
from .context_snapshot import ContextSnapshot, collect_snapshot
from .pty_session import PtySession
from .state import (
    NOTIFICATION_REASON,
    SessionMetadata,
    SessionState,
    WaitReason,
    classify_message,
)

log = logging.getLogger("supervisor.core")


class Alerter(Protocol):
    async def send_alert(
        self,
        meta: SessionMetadata,
        snapshot: Optional["ContextSnapshot"] = None,
        *,
        reminder: bool = False,
    ) -> None: ...


def _read_git_branch(cwd: str) -> str:
    try:
        head = Path(cwd, ".git", "HEAD").read_text(encoding="utf-8").strip()
        if head.startswith("ref:"):
            return head.split("/", 2)[-1]
        return head[:12]
    except Exception:
        return ""


class Supervisor:
    def __init__(
        self,
        config: Config,
        pty: PtySession,
        loop: asyncio.AbstractEventLoop,
        *,
        enable_snapshots: bool = True,
    ) -> None:
        self.cfg = config
        self.pty = pty
        self.loop = loop
        self.enable_snapshots = enable_snapshots
        self.last_snapshot: Optional[ContextSnapshot] = None
        self.alerter: Optional[Alerter] = None
        self.meta = SessionMetadata(
            working_directory=os.getcwd(),
            project_name=Path(os.getcwd()).name,
            git_branch=_read_git_branch(os.getcwd()),
            machine=socket.gethostname(),
        )
        self._paused = False
        self._alert_task: Optional[asyncio.Task] = None
        self._reminder_task: Optional[asyncio.Task] = None
        self._runner: Optional[web.AppRunner] = None

    def set_alerter(self, alerter: Alerter) -> None:
        self.alerter = alerter

    # ---- event sink ------------------------------------------------------

    async def start_sink(self) -> None:
        app = web.Application()
        app.router.add_post("/event", self._handle_post)
        app.router.add_get("/health", lambda r: web.Response(text="ok"))
        self._runner = web.AppRunner(app)
        await self._runner.setup()
        site = web.TCPSite(self._runner, self.cfg.supervisor.host, self.cfg.supervisor.port)
        await site.start()
        log.info("event sink listening on %s:%s", self.cfg.supervisor.host, self.cfg.supervisor.port)

    async def stop_sink(self) -> None:
        if self._runner:
            await self._runner.cleanup()

    async def _handle_post(self, request: web.Request) -> web.Response:
        if self.cfg.ipc_token:
            if request.headers.get("X-Supervisor-Token", "") != self.cfg.ipc_token:
                return web.Response(status=403, text="forbidden")
        try:
            payload = await request.json()
        except Exception:
            return web.Response(status=400, text="bad json")
        await self.handle_event(payload)
        return web.Response(text="ok")

    # ---- transitions -----------------------------------------------------

    async def handle_event(self, payload: dict) -> None:
        event = payload.get("hook_event_name", "")
        self._update_session_from_payload(payload)

        if event == "Notification":
            ntype = payload.get("notification_type", "")
            reason = NOTIFICATION_REASON.get(ntype, WaitReason.CLARIFICATION)
            self._to_waiting(reason, payload.get("message", ""))
        elif event == "Stop":
            msg = payload.get("last_assistant_message", "")
            self._to_waiting(classify_message(msg), msg)
        elif event in ("UserPromptSubmit", "PreToolUse", "PostToolUse", "PostToolBatch"):
            self._to_running()
        elif event == "SessionStart":
            self._to_running()
        elif event == "SessionEnd":
            self._cancel_timers()
        log.debug("event=%s -> state=%s", event, self.meta.current_state.value)

    def _update_session_from_payload(self, payload: dict) -> None:
        if payload.get("session_id"):
            self.meta.session_id = payload["session_id"]
        cwd = payload.get("cwd")
        if cwd:
            self.meta.working_directory = cwd
            self.meta.project_name = Path(cwd).name
            self.meta.git_branch = _read_git_branch(cwd) or self.meta.git_branch
        if self.pty.pid:
            self.meta.pid = self.pty.pid

    def _to_running(self) -> None:
        self._cancel_timers()
        self.meta.current_state = SessionState.RUNNING
        self.meta.waiting_reason = None
        self.meta.wait_start_timestamp = None
        self.meta.wait_start_monotonic = None

    def _to_waiting(self, reason: WaitReason, text: str) -> None:
        # Already alerted and still waiting: don't reset the clock.
        if self.meta.current_state == SessionState.DISCORD_ALERT_SENT:
            return
        self.meta.current_state = SessionState.WAITING_FOR_HUMAN
        self.meta.waiting_reason = reason
        self.meta.last_prompt_text = (text or "").strip()
        self.meta.wait_start_timestamp = _dt.datetime.now(_dt.timezone.utc).isoformat()
        self.meta.wait_start_monotonic = time.monotonic()
        self._cancel_timers()
        if self.enable_snapshots:
            # Capture context now (off the event loop); ready well before the alert.
            self.loop.create_task(self.refresh_snapshot())
        self._alert_task = self.loop.create_task(self._wait_then_alert())

    async def _wait_then_alert(self) -> None:
        try:
            await asyncio.sleep(self.cfg.timers.initial_timeout_seconds)
            while self._paused:
                await asyncio.sleep(1)
            if self.meta.current_state != SessionState.WAITING_FOR_HUMAN:
                return
            await self._fire_alert(reminder=False)
            self.meta.current_state = SessionState.DISCORD_ALERT_SENT
            self._reminder_task = self.loop.create_task(self._reminder_loop())
        except asyncio.CancelledError:
            pass

    async def _reminder_loop(self) -> None:
        try:
            while self.meta.current_state == SessionState.DISCORD_ALERT_SENT:
                await asyncio.sleep(self.cfg.timers.reminder_interval_seconds)
                if self._paused:
                    continue
                if self.meta.current_state == SessionState.DISCORD_ALERT_SENT:
                    await self._fire_alert(reminder=True)
        except asyncio.CancelledError:
            pass

    async def _fire_alert(self, *, reminder: bool) -> None:
        if self.alerter:
            try:
                await self.alerter.send_alert(self.meta, self.last_snapshot, reminder=reminder)
            except Exception:
                log.exception("failed to send Discord alert")

    # ---- context snapshot ------------------------------------------------

    async def refresh_snapshot(self) -> Optional[ContextSnapshot]:
        """Collect a fresh context snapshot (git subprocess runs in an executor)."""
        needs = self.meta.waiting_reason.value if self.meta.waiting_reason else None
        question = self.meta.last_prompt_text
        cwd = self.meta.working_directory
        output_tail = self.pty.recent_logs(100)
        try:
            snap = await self.loop.run_in_executor(
                None,
                lambda: collect_snapshot(
                    cwd,
                    output_tail=output_tail,
                    needs=needs,
                    question=question,
                    sidecar_name=self.cfg.supervisor.status_file,
                ),
            )
            self.last_snapshot = snap
            return snap
        except Exception:
            log.exception("snapshot collection failed")
            return self.last_snapshot

    def _cancel_timers(self) -> None:
        for t in (self._alert_task, self._reminder_task):
            if t and not t.done():
                t.cancel()
        self._alert_task = None
        self._reminder_task = None

    # ---- injection (called from the Discord bot) -------------------------

    def _mark_injecting(self) -> None:
        self._cancel_timers()
        self.meta.current_state = SessionState.INJECTING_INPUT

    def inject_text(self, text: str) -> None:
        self._mark_injecting()
        self.pty.send_text_line(text)

    def approve(self) -> None:
        self._mark_injecting()
        self.pty.send_keys(self.cfg.supervisor.approve_keys)

    def deny(self) -> None:
        self._mark_injecting()
        self.pty.send_keys(self.cfg.supervisor.deny_keys)

    def continue_(self) -> None:
        self.inject_text(self.cfg.supervisor.continue_text)

    def stop_claude(self) -> None:
        self._mark_injecting()
        self.pty.send_ctrl_c()

    def pause(self) -> bool:
        self._paused = True
        return self._paused

    def resume(self) -> bool:
        self._paused = False
        return self._paused

    @property
    def paused(self) -> bool:
        return self._paused

    def recent_logs(self, n: int = 50) -> str:
        return self.pty.recent_logs(n)
