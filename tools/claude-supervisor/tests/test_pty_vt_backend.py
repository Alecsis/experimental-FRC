"""Regression tests for PtySession's local-input *backend selection* (Plan
Phase 1, Task 1.4).

``start()`` now branches once on ``self._input_backend``: "legacy" launches
today's ``_input``/``_mouser`` threads unchanged; "vt" launches a placeholder
relay thread that starts, waits on the stop event, and exits cleanly -- no
real VT relay logic yet (that's Phase 2). This only proves the new path is
wired into the same lifecycle (``stop()``, thread join) without asserting
anything about relay behavior.

These tests spawn a real (trivial) child process via pywinpty, since the
routing lives inside ``start()`` -- there is no pure function to test in
isolation here, same rationale as the resize/mouse loops.

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


def test_vt_backend_starts_and_stops_cleanly_with_no_thread_left_alive():
    print("test_vt_backend_starts_and_stops_cleanly_with_no_thread_left_alive")
    session = PtySession(_TRIVIAL_ARGV, forward_local_input=True, input_backend="vt")
    raised = None
    try:
        session.start()
        # Give the threads a moment to actually spin up before tearing down.
        time.sleep(0.1)
        session.stop()
    except Exception as exc:  # pragma: no cover - failure path
        raised = exc
    check(raised is None, "start()+stop() on the vt backend raises nothing")

    # Every thread start() may have spun up must have wound down by now.
    for attr in ("_reader", "_input", "_mouser", "_resizer", "_vt_relay"):
        thread = getattr(session, attr, None)
        if thread is None:
            continue
        thread.join(timeout=2.0)
        check(not thread.is_alive(), f"{attr} thread is not alive after stop()")


def test_vt_backend_does_not_start_legacy_input_or_mouse_threads():
    print("test_vt_backend_does_not_start_legacy_input_or_mouse_threads")
    session = PtySession(_TRIVIAL_ARGV, forward_local_input=True, input_backend="vt")
    session.start()
    time.sleep(0.1)
    try:
        check(session._input is None, "legacy _input thread is not launched under vt backend")
        check(session._mouser is None, "legacy _mouser thread is not launched under vt backend")
        check(session._vt_relay is not None, "a vt relay thread placeholder was launched")
    finally:
        session.stop()


def test_legacy_backend_still_starts_input_and_mouse_threads():
    print("test_legacy_backend_still_starts_input_and_mouse_threads")
    session = PtySession(_TRIVIAL_ARGV, forward_local_input=True, input_backend="legacy")
    session.start()
    time.sleep(0.1)
    try:
        check(session._input is not None, "legacy _input thread still launches under legacy backend")
        check(session._mouser is not None, "legacy _mouser thread still launches under legacy backend")
        check(getattr(session, "_vt_relay", None) is None,
              "no vt relay thread is launched under legacy backend")
    finally:
        session.stop()


def main():
    test_vt_backend_starts_and_stops_cleanly_with_no_thread_left_alive()
    test_vt_backend_does_not_start_legacy_input_or_mouse_threads()
    test_legacy_backend_still_starts_input_and_mouse_threads()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
