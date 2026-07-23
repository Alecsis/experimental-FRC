"""Entry point: launch Claude Code under supervision.

Usage (from a normal PowerShell terminal, in the project you want Claude to work in):

    python C:\\Users\\xdm\\Claude\\experimental-FRC\\tools\\claude-supervisor\\run.py

This spawns `claude` inside a PTY, starts the local hook event-sink, and connects
the Discord bot. Interact with Claude locally as usual; when it goes idle for the
configured timeout, you get a Discord alert and can drive it from your phone.
Exit Claude normally (or its process ends) and the supervisor shuts down.
"""

from __future__ import annotations

import asyncio
import logging
import os
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

from claude_supervisor.config import load_config  # noqa: E402
from claude_supervisor.core import Supervisor  # noqa: E402
from claude_supervisor.pty_session import PtySession  # noqa: E402
from claude_supervisor.bot import SupervisorBot  # noqa: E402

CONFIG_PATH = os.environ.get("CLAUDE_SUPERVISOR_CONFIG", str(_HERE / "config.yaml"))


async def _run_until_stopped(
    pty: PtySession,
    sup: Supervisor,
    bot: SupervisorBot,
    stop_event: asyncio.Event,
    bot_task: "asyncio.Task",
) -> None:
    """Block until ``stop_event`` fires, then tear everything down.

    Task 3.5: wrapped in try/finally so the teardown -- in particular
    ``pty.stop()``, which restores the vt backend's console mode (Tasks
    3.3/3.4) -- still runs on ANY exit from this coroutine's ``await``, not
    just ``stop_event`` firing normally via ``on_child_exit``/``!stop``. This
    is what closes the "process teardown" gap for a KeyboardInterrupt
    delivered while suspended here: ``asyncio.run()`` delivers a Ctrl+C at
    this console as a real ``KeyboardInterrupt`` raised into whichever
    ``await`` is currently active, and Python unwinds through this
    function's own ``finally`` exactly like it would for any other
    exception -- confirmed by test_run_shutdown.py's
    ``test_keyboard_interrupt_while_waiting_still_tears_down``.

    Documented, not testable, residual gap (per this task's explicit
    instruction not to imply coverage that doesn't exist): a *hard* external
    termination of this process -- Windows task-kill, ``os._exit``, a power
    loss -- runs no Python cleanup at all. There is no OS hook available on
    Windows this function (or anything else in-process) could register to
    close that gap; it is accepted as unavoidable, not something this task
    can fix.
    """
    try:
        await stop_event.wait()
    finally:
        pty.stop()
        await bot.close()
        await sup.stop_sink()
        if not bot_task.done():
            bot_task.cancel()


async def _start_and_supervise(
    pty: PtySession,
    sup: Supervisor,
    bot: SupervisorBot,
    stop_event: asyncio.Event,
    loop: "asyncio.AbstractEventLoop",
    log: logging.Logger,
    bot_token: str,
) -> None:
    """Start Claude under the PTY and the Discord bot, then run until stopped.

    Code-review fix (Task 3.5): ``pty.start()`` succeeding, then
    ``sup.meta.pid``/``bot_task`` creation/``log.info(...)`` all used to run
    directly in ``amain()``, BEFORE ``_run_until_stopped``'s own try/finally
    was ever entered -- none of that span was covered by Task 3.4's guard
    (internal to ``pty.start()``, already returned by that point) or by
    ``_run_until_stopped``'s own finally (not entered yet). A
    KeyboardInterrupt landing anywhere in that span would bypass both and
    leave the vt backend's console mode mutated with nothing to restore it.

    Wrapping this whole region -- from ``pty.start()`` through
    ``_run_until_stopped`` -- in ``try/except BaseException: pty.stop(); raise``
    closes that window. A double ``pty.stop()`` call (this guard's, plus
    ``_run_until_stopped``'s own finally, if the interrupt instead lands
    *inside* ``_run_until_stopped``) is safe: ``PtySession.stop()`` and
    ``_restore_stdin_mode()`` are both idempotent (Task 3.3).
    """
    try:
        pty.start()
        sup.meta.pid = pty.pid

        bot_task = loop.create_task(bot.start(bot_token))
        log.info("Supervisor up. Claude pid=%s", pty.pid)

        await _run_until_stopped(pty, sup, bot, stop_event, bot_task)
    except BaseException:
        pty.stop()
        raise


async def amain() -> int:
    cfg = load_config(CONFIG_PATH)
    logging.basicConfig(
        level=getattr(logging, cfg.behavior.log_level.upper(), logging.INFO),
        filename=str(_HERE / "supervisor.log"),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    log = logging.getLogger("supervisor")
    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()

    child_env = dict(os.environ)
    child_env["CLAUDE_SUPERVISOR_HOST"] = cfg.supervisor.host
    child_env["CLAUDE_SUPERVISOR_PORT"] = str(cfg.supervisor.port)
    child_env["CLAUDE_SUPERVISOR_TOKEN"] = cfg.ipc_token

    def on_child_exit(code: int) -> None:
        log.info("Claude child exited with code %s; shutting down.", code)
        loop.call_soon_threadsafe(stop_event.set)

    pty = PtySession(
        cfg.supervisor.claude_command,
        cwd=os.getcwd(),
        env=child_env,
        log_buffer_lines=cfg.supervisor.log_buffer_lines,
        on_exit=on_child_exit,
        input_backend=cfg.behavior.local_input_backend,
    )
    sup = Supervisor(cfg, pty, loop)
    bot = SupervisorBot(cfg, sup)
    sup.set_alerter(bot)

    await sup.start_sink()
    await _start_and_supervise(pty, sup, bot, stop_event, loop, log, cfg.discord.bot_token)
    return 0


def main() -> int:
    try:
        return asyncio.run(amain())
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
