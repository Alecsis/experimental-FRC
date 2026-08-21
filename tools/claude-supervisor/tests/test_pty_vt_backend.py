"""Regression tests for PtySession's local-input relay thread lifecycle (Plan
Phase 1, Task 1.4; legacy backend removed in Phase 4).

``start()`` launches ``_vt_relay_loop`` as a daemon thread whenever
``forward_local_input`` is set, alongside the reader/resize threads, and
every thread must be wound down cleanly by ``stop()``.

This test spawns a real (trivial) child process via pywinpty, since the
routing lives inside ``start()`` -- there is no pure function to test in
isolation here, same rationale as the resize loop.

Run: python tests/test_pty_vt_backend.py
"""

from __future__ import annotations

import sys
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


# A trivial, quick-to-spawn child that just sits there until killed.
_TRIVIAL_ARGV = ["python", "-c", "import time; time.sleep(30)"]


def test_vt_relay_starts_and_stops_cleanly_with_no_thread_left_alive():
    print("test_vt_relay_starts_and_stops_cleanly_with_no_thread_left_alive")
    session = PtySession(_TRIVIAL_ARGV, forward_local_input=True)
    raised = None
    try:
        session.start()
        # Give the threads a moment to actually spin up before tearing down.
        time.sleep(0.1)
        check(session._vt_relay is not None, "the vt relay thread was launched")
        session.stop()
    except Exception as exc:  # pragma: no cover - failure path
        raised = exc
    check(raised is None, "start()+stop() raises nothing")

    # Every thread start() may have spun up must have wound down by now.
    for attr in ("_reader", "_resizer", "_vt_relay"):
        thread = getattr(session, attr, None)
        if thread is None:
            continue
        thread.join(timeout=2.0)
        check(not thread.is_alive(), f"{attr} thread is not alive after stop()")


def main():
    test_vt_relay_starts_and_stops_cleanly_with_no_thread_left_alive()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
