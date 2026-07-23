"""Regression tests for the vt-input relay's transparent byte relay (Plan
Phase 2, Task 2.2 + 2.3).

Task 1 wired a placeholder ``_vt_relay_loop`` into the start()/stop()
lifecycle (see test_pty_vt_backend.py). Task 2.1 turned on
ENABLE_VIRTUAL_TERMINAL_INPUT on stdin (see test_pty_input.py). This file
covers what's left: the loop itself must read whatever bytes are currently
available and forward them to send_keys() completely unchanged -- no
per-byte/per-sequence interpretation of any kind, since Windows Terminal
(with VT input mode on) has already done all necessary translation before
the bytes ever reach this process.

``_vt_relay_loop`` takes an injectable ``read_available`` callable standing in
for the real blocking console read -- production callers leave it as None
and get the real Win32 reader (_read_vt_input), untested here since it needs
a real console; these tests only need PtySession's plumbing (send_keys/
_write_lock) plus a fake byte source.

Run: python tests/test_pty_vt_relay.py
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from claude_supervisor.pty_session import PtySession

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


class _FakeProc:
    """Stands in for winpty.PtyProcess -- just records what was written."""

    def __init__(self, on_write=None):
        self._on_write = on_write
        self.calls = []

    def isalive(self):
        return True

    def write(self, text):
        if self._on_write:
            self._on_write(text)
        self.calls.append(text)


def _session():
    # forward_local_input doesn't matter here -- we drive _vt_relay_loop
    # directly, never through start().
    return PtySession(["python", "-c", "pass"], forward_local_input=False)


def test_relay_forwards_unknown_escape_sequence_byte_for_byte():
    """Task 2.2's core proof: an arbitrary escape sequence with no special
    meaning to this module (Phase 4 deleted the last of the special-case
    tables that used to interpret sequences like this one) must pass through
    completely unrecognized and unmodified -- proving "no translation"
    behaviorally, not just by inspecting the diff for missing branches."""
    print("test_relay_forwards_unknown_escape_sequence_byte_for_byte")
    session = _session()
    proc = _FakeProc()
    session._proc = proc

    # An arbitrary CSI sequence with no entry anywhere in this module's
    # tables (not an arrow key, not an SGR mouse-wheel report).
    unknown_sequence = "\x1b[<99;5;10M"
    plain_text = "hello world"
    chunks = iter([unknown_sequence, plain_text])

    def fake_read():
        try:
            return next(chunks)
        except StopIteration:
            session._stop.set()  # let the loop terminate on its own
            return ""

    session._vt_relay_loop(read_available=fake_read)

    check(proc.calls == [unknown_sequence, plain_text],
          "both chunks forwarded verbatim, in order, nothing dropped or altered")


def test_relay_skips_empty_reads_without_forwarding():
    """An empty string from read_available (production: 'nothing arrived in
    this poll window') must not turn into a spurious send_keys('') call."""
    print("test_relay_skips_empty_reads_without_forwarding")
    session = _session()
    proc = _FakeProc()
    session._proc = proc

    reads = iter(["", "", "x"])

    def fake_read():
        try:
            value = next(reads)
        except StopIteration:
            session._stop.set()
            return ""
        if value == "x":
            session._stop.set()
        return value

    session._vt_relay_loop(read_available=fake_read)

    check(proc.calls == ["x"], "only the non-empty read was forwarded")


def test_relay_and_injection_share_write_lock_without_interleaving():
    """Task 2.3: the vt relay thread and a concurrent Discord-injection-style
    caller must serialize through the same _write_lock / send_keys() path,
    with no new write path and no interleaved/corrupted writes.

    Uses two *real* threads (not sequential calls standing in for
    concurrency). The fake write() sleeps between characters so that any
    missing/broken locking would visibly interleave two chunks' characters in
    the shared buffer -- a bug the test can actually catch, not just a
    same-thread simulation that could never fail regardless of locking.
    """
    print("test_relay_and_injection_share_write_lock_without_interleaving")
    session = _session()

    char_buffer = []
    write_order = []

    def on_write(text):
        for ch in text:
            char_buffer.append(ch)
            time.sleep(0.0002)
        write_order.append(text)

    session._proc = _FakeProc(on_write=on_write)

    relay_chunks = [f"RELAY{i}\x1b[<99;5;10M" for i in range(15)]
    inject_chunks = [f"INJECT{i}" for i in range(15)]

    relay_iter = iter(relay_chunks)

    def fake_read():
        try:
            return next(relay_iter)
        except StopIteration:
            time.sleep(0.001)  # avoid a hot spin once the relay source is drained
            return ""

    relay_thread = threading.Thread(
        target=session._vt_relay_loop, kwargs={"read_available": fake_read}
    )

    def inject_worker():
        for chunk in inject_chunks:
            session.send_keys(chunk)  # the same call Discord's !reply path uses

    inject_thread = threading.Thread(target=inject_worker)

    relay_thread.start()
    inject_thread.start()
    inject_thread.join(timeout=5)
    time.sleep(0.05)  # let the relay drain whatever's left of relay_chunks
    session._stop.set()
    relay_thread.join(timeout=5)

    check(not relay_thread.is_alive(), "relay thread stopped cleanly after self._stop.set()")
    check(
        sorted(write_order) == sorted(relay_chunks + inject_chunks),
        "every chunk from both producers was written exactly once, none lost or duplicated",
    )
    check(
        "".join(char_buffer) == "".join(write_order),
        "no interleaving: the char-by-char buffer matches the whole-chunk write "
        "order exactly, proving each write() call completed atomically under the lock",
    )


def main():
    test_relay_forwards_unknown_escape_sequence_byte_for_byte()
    test_relay_skips_empty_reads_without_forwarding()
    test_relay_and_injection_share_write_lock_without_interleaving()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
