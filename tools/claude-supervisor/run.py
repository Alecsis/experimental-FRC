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
    )
    sup = Supervisor(cfg, pty, loop)
    bot = SupervisorBot(cfg, sup)
    sup.set_alerter(bot)

    await sup.start_sink()
    pty.start()
    sup.meta.pid = pty.pid

    bot_task = loop.create_task(bot.start(cfg.discord.bot_token))
    log.info("Supervisor up. Claude pid=%s", pty.pid)

    await stop_event.wait()

    pty.stop()
    await bot.close()
    await sup.stop_sink()
    if not bot_task.done():
        bot_task.cancel()
    return 0


def main() -> int:
    try:
        return asyncio.run(amain())
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
