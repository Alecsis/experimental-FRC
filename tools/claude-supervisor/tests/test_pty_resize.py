"""Regression tests for PTY window-resize propagation.

Guards the fix for the bug where the inner (pywinpty/ConPTY) session hosting
Claude was sized once at spawn and never updated -- resizing the real terminal
never reached Claude, so it couldn't reflow its UI, and its own cursor-redraw
math (based on a stale believed size) could corrupt what looked like scrollback.

``poll_resize`` is the pure piece: given the last-known size and an injected
``get_size`` (mirroring ``os.get_terminal_size``), decide whether the size
changed and what the new size is. The threaded polling loop itself needs a
real console, so it is not unit-tested here -- same rationale as
``test_pty_input.py``'s treatment of ``_input_loop``.

Run: python tests/test_pty_resize.py
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from claude_supervisor.pty_session import poll_resize

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


def _size(cols, lines):
    return lambda: os.terminal_size((cols, lines))


def _raises_oserror():
    def get_size():
        raise OSError("no console attached")
    return get_size


def test_unchanged_size_returns_none():
    print("test_unchanged_size_returns_none")
    check(poll_resize((30, 120), _size(120, 30)) is None, "same (rows, cols) -> None")


def test_changed_size_returns_new_rows_cols():
    print("test_changed_size_returns_new_rows_cols")
    check(poll_resize((30, 120), _size(200, 50)) == (50, 200),
          "wider/taller terminal -> (rows, cols) tuple, not (cols, rows)")


def test_shrunk_size_is_also_reported():
    print("test_shrunk_size_is_also_reported")
    check(poll_resize((50, 200), _size(80, 24)) == (24, 80), "smaller terminal reported too")


def test_oserror_from_get_size_reports_no_change():
    print("test_oserror_from_get_size_reports_no_change")
    check(poll_resize((30, 120), _raises_oserror()) is None,
          "no console attached (redirected output) -> None, not a crash")


def main():
    test_unchanged_size_returns_none()
    test_changed_size_returns_new_rows_cols()
    test_shrunk_size_is_also_reported()
    test_oserror_from_get_size_reports_no_change()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
