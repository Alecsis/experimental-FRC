"""Regression tests for stripping xterm mouse-tracking escape sequences.

Guards the fix for the bug where Claude Code's TUI enables xterm mouse
tracking (DECSET ``\x1b[?1000h`` etc.) intended for its own controlling
terminal, but ``PtySession._read_loop``'s raw byte passthrough forwarded it
untouched to the *outer* console too -- Windows Terminal then routed mouse
wheel/click events to the supervisor's own process (which never reads mouse
input) instead of doing native scrollback, so wheel scroll went dead only
when running under the supervisor. Confirmed live via ``mouse_debug.log``
(diagnostic instrumentation added in the prior session) actually capturing
``?1000h``/``?1002h``/``?1003h``/``?1006h``/``?1004h`` during a real session.

``strip_mouse_tracking`` is the pure piece: given a chunk of Claude's raw
output, remove only the mouse-tracking DECSET/DECRST sequences, leaving
everything else -- including bracketed-paste (``?2004``), which is an
unrelated existing mechanism this fix must not disturb -- untouched.

Run: python tests/test_pty_mouse_strip.py
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from claude_supervisor.pty_session import strip_mouse_tracking

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


def test_strips_x10_mouse_mode_enable():
    print("test_strips_x10_mouse_mode_enable")
    check(strip_mouse_tracking("\x1b[?1000h") == "", "?1000h removed entirely")


def test_strips_sgr_mouse_mode_disable():
    print("test_strips_sgr_mouse_mode_disable")
    check(strip_mouse_tracking("\x1b[?1006l") == "", "?1006l removed entirely")


def test_strips_all_mouse_modes_seen_in_mouse_debug_log():
    print("test_strips_all_mouse_modes_seen_in_mouse_debug_log")
    chunk = "\x1b[?1000h\x1b[?1002h\x1b[?1003h\x1b[?1006h\x1b[?1004h"
    check(strip_mouse_tracking(chunk) == "", "every mode captured live strips to empty")


def test_leaves_bracketed_paste_mode_untouched():
    print("test_leaves_bracketed_paste_mode_untouched")
    check(strip_mouse_tracking("\x1b[?2004h") == "\x1b[?2004h",
          "?2004 (bracketed paste) is a different mechanism, must survive")


def test_leaves_surrounding_text_and_other_escapes_intact():
    print("test_leaves_surrounding_text_and_other_escapes_intact")
    chunk = "hello\x1b[31m\x1b[?1000hworld\x1b[0m"
    check(strip_mouse_tracking(chunk) == "hello\x1b[31mworld\x1b[0m",
          "only the mouse-mode sequence is removed, color codes and text stay")


def test_plain_text_with_no_escapes_is_unchanged():
    print("test_plain_text_with_no_escapes_is_unchanged")
    check(strip_mouse_tracking("just some output\n") == "just some output\n",
          "no-op on chunks without any mouse-mode sequence")


def main():
    test_strips_x10_mouse_mode_enable()
    test_strips_sgr_mouse_mode_disable()
    test_strips_all_mouse_modes_seen_in_mouse_debug_log()
    test_leaves_bracketed_paste_mode_untouched()
    test_leaves_surrounding_text_and_other_escapes_intact()
    test_plain_text_with_no_escapes_is_unchanged()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
