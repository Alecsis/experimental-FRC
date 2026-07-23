"""Regression tests for run.py's shutdown sequencing (Plan Phase 3, Task 3.5).

Task 3.3/3.4 made PtySession.stop() restore the vt backend's console mode on
every exit path *internal* to PtySession itself (normal stop(), and a
setup-time exception inside start()). Task 3.5 closes the remaining gap: a
KeyboardInterrupt delivered to run.py's own amain() coroutine (e.g. Ctrl+C at
this console, which asyncio.run() propagates into whichever `await` is
currently suspended) must still reach pty.stop() -- otherwise the console
mode restore Tasks 3.3/3.4 built never actually runs on that exit path, since
nothing else would call pty.stop() for it.

run.py's amain() is not itself unit-testable end-to-end (it loads a real
config.yaml, builds a real Supervisor/aiohttp hook sink, and connects a real
discord.py Bot to Discord's network) -- consistent with this project's
existing testing philosophy (PTY input/mouse loops aren't unit-tested either,
for the same "needs a real environment" reason). What *is* unit-testable, and
is the actual property Task 3.5 cares about, is extracted into
`run._run_until_stopped(pty, sup, bot, stop_event, bot_task)`: block on
stop_event, then always call pty.stop()/bot.close()/sup.stop_sink() in a
finally, regardless of how the block exits. This file exercises that function
directly with fakes standing in for PtySession/Supervisor/SupervisorBot.

Documented, not testable here or anywhere else (per the brief's explicit
instruction not to imply coverage that doesn't exist): a *hard* external
termination of this process -- Windows task-kill, `os._exit`, a power loss --
runs no Python cleanup at all. There is no OS hook available on Windows for
that case; it is an accepted, unavoidable gap, not something this task (or
any test) can close.

Run: python tests/test_run_shutdown.py
"""

from __future__ import annotations

import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from run import _run_until_stopped, _start_and_supervise  # noqa: E402

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


class _FakeMeta:
    def __init__(self):
        self.pid = None


class _FakePty:
    def __init__(self):
        self.start_calls = 0
        self.stop_calls = 0
        self.pid = 4242

    def start(self):
        self.start_calls += 1

    def stop(self):
        self.stop_calls += 1


class _FakeSup:
    def __init__(self):
        self.stop_sink_calls = 0
        self.meta = _FakeMeta()

    async def stop_sink(self):
        self.stop_sink_calls += 1


class _FakeBot:
    def __init__(self):
        self.close_calls = 0

    async def start(self, token):
        await asyncio.sleep(0)

    async def close(self):
        self.close_calls += 1


class _FakeLog:
    """Duck-types logging.Logger's .info() but raises instead -- stands in
    for a KeyboardInterrupt landing somewhere between pty.start() succeeding
    and _run_until_stopped's own try/finally being entered."""

    def __init__(self, raise_on_info=None):
        self._raise_on_info = raise_on_info

    def info(self, *args, **kwargs):
        if self._raise_on_info is not None:
            raise self._raise_on_info


class _RaisingStopEvent:
    """Duck-types asyncio.Event's .wait() but raises instead of returning --
    stands in for a KeyboardInterrupt arriving while suspended on the real
    stop_event.wait() call inside _run_until_stopped."""

    def __init__(self, exc):
        self._exc = exc

    async def wait(self):
        raise self._exc


async def _noop_task():
    return None


def _run(coro):
    return asyncio.run(coro)


def test_normal_stop_event_triggers_full_teardown_exactly_once():
    print("test_normal_stop_event_triggers_full_teardown_exactly_once")

    async def scenario():
        pty = _FakePty()
        sup = _FakeSup()
        bot = _FakeBot()
        stop_event = asyncio.Event()
        stop_event.set()  # already fired -- wait() returns immediately
        bot_task = asyncio.get_event_loop().create_task(_noop_task())
        await bot_task  # make it "done" before teardown so cancel() is skipped
        await _run_until_stopped(pty, sup, bot, stop_event, bot_task)
        return pty, sup, bot

    pty, sup, bot = _run(scenario())
    check(pty.stop_calls == 1, "pty.stop() called exactly once on the normal path")
    check(bot.close_calls == 1, "bot.close() called exactly once on the normal path")
    check(sup.stop_sink_calls == 1, "sup.stop_sink() called exactly once on the normal path")


def test_keyboard_interrupt_while_waiting_still_tears_down():
    """The literal property Task 3.5 asks for: a KeyboardInterrupt raised
    while suspended on stop_event.wait() must still reach pty.stop() (and
    therefore the vt backend's console-mode restore, Tasks 3.3/3.4) before
    propagating -- not be swallowed, and not skip teardown."""
    print("test_keyboard_interrupt_while_waiting_still_tears_down")

    async def scenario():
        pty = _FakePty()
        sup = _FakeSup()
        bot = _FakeBot()
        stop_event = _RaisingStopEvent(KeyboardInterrupt())
        bot_task = asyncio.get_event_loop().create_task(_noop_task())
        await bot_task
        raised = None
        try:
            await _run_until_stopped(pty, sup, bot, stop_event, bot_task)
        except KeyboardInterrupt as exc:
            raised = exc
        return pty, sup, bot, raised

    pty, sup, bot, raised = _run(scenario())
    check(raised is not None, "KeyboardInterrupt propagates out (not swallowed)")
    check(pty.stop_calls == 1, "pty.stop() still called exactly once despite the KeyboardInterrupt")
    check(bot.close_calls == 1, "bot.close() still called exactly once despite the KeyboardInterrupt")
    check(sup.stop_sink_calls == 1, "sup.stop_sink() still called exactly once despite the KeyboardInterrupt")


def test_generic_exception_while_waiting_still_tears_down_and_propagates():
    """Same property, but for an ordinary exception (not just KeyboardInterrupt)
    -- the finally-based teardown should not be special-cased to one exception
    type."""
    print("test_generic_exception_while_waiting_still_tears_down_and_propagates")

    async def scenario():
        pty = _FakePty()
        sup = _FakeSup()
        bot = _FakeBot()
        stop_event = _RaisingStopEvent(RuntimeError("boom"))
        bot_task = asyncio.get_event_loop().create_task(_noop_task())
        await bot_task
        raised = None
        try:
            await _run_until_stopped(pty, sup, bot, stop_event, bot_task)
        except RuntimeError as exc:
            raised = exc
        return pty, sup, bot, raised

    pty, sup, bot, raised = _run(scenario())
    check(raised is not None, "RuntimeError propagates out")
    check(pty.stop_calls == 1, "pty.stop() still called exactly once")
    check(bot.close_calls == 1, "bot.close() still called exactly once")
    check(sup.stop_sink_calls == 1, "sup.stop_sink() still called exactly once")


def test_unfinished_bot_task_is_cancelled_during_teardown():
    print("test_unfinished_bot_task_is_cancelled_during_teardown")

    async def scenario():
        pty = _FakePty()
        sup = _FakeSup()
        bot = _FakeBot()
        stop_event = asyncio.Event()
        stop_event.set()

        async def _never_finishes():
            await asyncio.sleep(30)

        bot_task = asyncio.get_event_loop().create_task(_never_finishes())
        await _run_until_stopped(pty, sup, bot, stop_event, bot_task)
        return bot_task

    bot_task = _run(scenario())
    check(bot_task.cancelled() or bot_task.cancelling() > 0,
          "a still-running bot_task is cancelled during teardown")


# ---- _start_and_supervise -- the code-review-fixed second window ---------
# Covers the window between pty.start() succeeding and _run_until_stopped
# being entered (sup.meta.pid assignment / bot_task creation / log.info).

def test_start_and_supervise_normal_path_starts_pty_and_tears_down_once():
    print("test_start_and_supervise_normal_path_starts_pty_and_tears_down_once")

    async def scenario():
        pty = _FakePty()
        sup = _FakeSup()
        bot = _FakeBot()
        stop_event = asyncio.Event()
        stop_event.set()
        loop = asyncio.get_event_loop()
        log = _FakeLog()
        await _start_and_supervise(pty, sup, bot, stop_event, loop, log, "fake-token")
        return pty, sup, bot

    pty, sup, bot = _run(scenario())
    check(pty.start_calls == 1, "pty.start() called exactly once")
    check(sup.meta.pid == pty.pid, "sup.meta.pid gets set from the started pty")
    check(pty.stop_calls == 1, "pty.stop() still called exactly once via _run_until_stopped's finally")
    check(bot.close_calls == 1, "bot.close() called exactly once")
    check(sup.stop_sink_calls == 1, "sup.stop_sink() called exactly once")


def test_keyboardinterrupt_between_pty_start_and_run_until_stopped_still_restores():
    """The literal window the reviewer identified: pty.start() has already
    succeeded (mode mutated + saved for real, inside PtySession), but the
    KeyboardInterrupt lands in the synchronous span after that -- here,
    log.info(...) -- before _run_until_stopped (and therefore its own
    try/finally) is ever entered. Without the code-review fix, this would
    propagate straight out of _start_and_supervise/amain with pty.stop()
    never called at all."""
    print("test_keyboardinterrupt_between_pty_start_and_run_until_stopped_still_restores")

    async def scenario():
        pty = _FakePty()
        sup = _FakeSup()
        bot = _FakeBot()
        stop_event = asyncio.Event()
        loop = asyncio.get_event_loop()
        log = _FakeLog(raise_on_info=KeyboardInterrupt())
        raised = None
        try:
            await _start_and_supervise(pty, sup, bot, stop_event, loop, log, "fake-token")
        except KeyboardInterrupt as exc:
            raised = exc
        return pty, raised

    pty, raised = _run(scenario())
    check(raised is not None, "the KeyboardInterrupt propagates out")
    check(pty.start_calls == 1, "pty.start() still ran before the interrupt landed")
    check(pty.stop_calls == 1,
          "pty.stop() still called exactly once even though the interrupt landed "
          "before _run_until_stopped was ever entered")


def test_generic_exception_between_pty_start_and_run_until_stopped_still_restores():
    """Same window, ordinary exception -- not special-cased to KeyboardInterrupt."""
    print("test_generic_exception_between_pty_start_and_run_until_stopped_still_restores")

    async def scenario():
        pty = _FakePty()
        sup = _FakeSup()
        bot = _FakeBot()
        stop_event = asyncio.Event()
        loop = asyncio.get_event_loop()
        log = _FakeLog(raise_on_info=RuntimeError("boom"))
        raised = None
        try:
            await _start_and_supervise(pty, sup, bot, stop_event, loop, log, "fake-token")
        except RuntimeError as exc:
            raised = exc
        return pty, raised

    pty, raised = _run(scenario())
    check(raised is not None, "the RuntimeError propagates out")
    check(pty.stop_calls == 1, "pty.stop() still called exactly once")


def test_keyboardinterrupt_inside_run_until_stopped_does_not_double_teardown_badly():
    """When the interrupt instead lands *inside* _run_until_stopped (the
    already-covered first window), _start_and_supervise's own except also
    fires and calls pty.stop() a second time -- confirmed harmless/idempotent
    per Task 3.3, not a new bug introduced by stacking the two guards."""
    print("test_keyboardinterrupt_inside_run_until_stopped_does_not_double_teardown_badly")

    async def scenario():
        pty = _FakePty()
        sup = _FakeSup()
        bot = _FakeBot()
        stop_event = _RaisingStopEvent(KeyboardInterrupt())
        loop = asyncio.get_event_loop()
        log = _FakeLog()
        raised = None
        try:
            await _start_and_supervise(pty, sup, bot, stop_event, loop, log, "fake-token")
        except KeyboardInterrupt as exc:
            raised = exc
        return pty, bot, raised

    pty, bot, raised = _run(scenario())
    check(raised is not None, "the KeyboardInterrupt propagates out")
    check(pty.stop_calls == 2,
          "pty.stop() fires twice (once from _run_until_stopped's finally, "
          "once from _start_and_supervise's own except) -- confirmed harmless, "
          "not asserting this is ideal, just that it doesn't break anything")
    check(bot.close_calls == 1, "bot.close() still only fired once (from "
          "_run_until_stopped's own finally, which _start_and_supervise's "
          "except does not duplicate)")


def main():
    test_normal_stop_event_triggers_full_teardown_exactly_once()
    test_keyboard_interrupt_while_waiting_still_tears_down()
    test_generic_exception_while_waiting_still_tears_down_and_propagates()
    test_unfinished_bot_task_is_cancelled_during_teardown()
    test_start_and_supervise_normal_path_starts_pty_and_tears_down_once()
    test_keyboardinterrupt_between_pty_start_and_run_until_stopped_still_restores()
    test_generic_exception_between_pty_start_and_run_until_stopped_still_restores()
    test_keyboardinterrupt_inside_run_until_stopped_does_not_double_teardown_badly()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
