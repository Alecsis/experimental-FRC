"""Regression tests for the console-mode plan that makes VT-input relay possible.

Guards the original bug this VT-input-relay migration replaced: local keyboard
input into the proxied Claude TUI was broken under the old classic-console
translation layer (Enter did nothing, Ctrl+C emitted ``;;;;;`` garbage, arrows
dead) any time stdin was accidentally left in ENABLE_VIRTUAL_TERMINAL_INPUT
mode. That whole translation layer is gone now (Plan Phase 4) -- the console
plan below is what actually turns VT-input mode on, deliberately, so
``PtySession._vt_relay_loop`` can forward Windows Terminal's own native VT
byte stream straight through. This file pins that plan's two contractual
guarantees.

Run: python tests/test_pty_input.py
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# pty_session imports cleanly without pywinpty (the native import is lazy, inside
# start()); _console_vt_plan touches neither winpty nor ctypes.
from claude_supervisor.pty_session import (
    ENABLE_VIRTUAL_TERMINAL_INPUT,
    ENABLE_VIRTUAL_TERMINAL_PROCESSING,
    STDIN_HANDLE,
    STDOUT_HANDLE,
    _console_vt_plan,
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


def test_console_plan_enables_vt_input_on_stdin():
    """The whole point of the vt-input relay: Windows Terminal must deliver
    keys/mouse/Shift+Tab/bracketed-paste to Claude as native VT escape
    sequences instead of a hand-rolled classic-console translation, which
    requires ENABLE_VIRTUAL_TERMINAL_INPUT on stdin."""
    print("test_console_plan_enables_vt_input_on_stdin")
    plan = _console_vt_plan()
    check(
        any(handle_id == STDIN_HANDLE and (flag & ENABLE_VIRTUAL_TERMINAL_INPUT)
            for handle_id, flag in plan),
        "plan enables VT-input (0x0200) on stdin",
    )


def test_console_plan_still_includes_stdout_vt_processing():
    """Regression guard: the stdin entry must not drop or alter the
    pre-existing stdout ENABLE_VIRTUAL_TERMINAL_PROCESSING entry -- both live
    in the same plan function."""
    print("test_console_plan_still_includes_stdout_vt_processing")
    plan = _console_vt_plan()
    check(
        any(handle_id == STDOUT_HANDLE and (flag & ENABLE_VIRTUAL_TERMINAL_PROCESSING)
            for handle_id, flag in plan),
        "plan still enables VT-processing (0x0004) on stdout",
    )


def main():
    test_console_plan_enables_vt_input_on_stdin()
    test_console_plan_still_includes_stdout_vt_processing()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
