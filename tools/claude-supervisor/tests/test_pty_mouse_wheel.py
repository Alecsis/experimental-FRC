"""Regression tests for mouse-wheel forwarding into Claude's PTY.

Guards the fix for the follow-on gap the mouse-tracking-leak fix (previous
session, ``strip_mouse_tracking``) exposed: stripping Claude's DECSET stopped
it leaking to the outer console, but there was never a path for mouse events
to reach Claude at all -- ``_input_loop`` only ever reads keyboard. Wheel
scroll then fell back to Windows Terminal's native QuickEdit scrollback
(raw historical output) instead of scrolling Claude's own chat view.

``extract_wheel_delta``, ``translate_wheel_event``, and ``classify_record``
are the pure pieces: given a raw Win32 ``MOUSE_EVENT_RECORD`` field or a
peeked ``INPUT_RECORD``'s type/flags, decide the signed notch delta, the SGR
sequence to forward, and whether a record should be dequeued at all. The
actual ``PeekConsoleInputW``/``ReadConsoleInputW`` polling loop
(``PtySession._mouse_loop``) needs a real attached console, so it is not
unit-tested here -- same rationale as ``test_pty_resize.py``'s treatment of
the threaded polling loop, and ``test_pty_input.py``'s treatment of
``_input_loop``.

Run: python tests/test_pty_mouse_wheel.py
"""

from __future__ import annotations

import ctypes
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from claude_supervisor.pty_session import (
    INPUT_RECORD,
    KEY_EVENT_TYPE,
    MOUSE_EVENT_TYPE,
    MOUSE_HWHEELED,
    MOUSE_WHEELED,
    classify_record,
    extract_wheel_delta,
    translate_wheel_event,
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


def test_extract_wheel_delta_positive():
    print("test_extract_wheel_delta_positive")
    check(extract_wheel_delta(0x00780000) == 120, "high word 0x0078 -> +120")


def test_extract_wheel_delta_negative():
    print("test_extract_wheel_delta_negative")
    check(extract_wheel_delta(0xFF880000) == -120, "high word 0xFF88 -> -120 (signed)")


def test_extract_wheel_delta_zero():
    print("test_extract_wheel_delta_zero")
    check(extract_wheel_delta(0x00000000) == 0, "zero high word -> 0")


def test_translate_wheel_event_single_notch_up():
    print("test_translate_wheel_event_single_notch_up")
    check(translate_wheel_event(120, 5, 10) == "\x1b[<64;5;10M", "+120 -> one wheel-up SGR event")


def test_translate_wheel_event_single_notch_down():
    print("test_translate_wheel_event_single_notch_down")
    check(translate_wheel_event(-120, 5, 10) == "\x1b[<65;5;10M", "-120 -> one wheel-down SGR event")


def test_translate_wheel_event_multi_notch_fast_scroll():
    print("test_translate_wheel_event_multi_notch_fast_scroll")
    check(translate_wheel_event(360, 1, 1) == "\x1b[<64;1;1M" * 3,
          "360 (3 notches) -> three stacked SGR events")


def test_translate_wheel_event_rounds_to_nearest_notch():
    print("test_translate_wheel_event_rounds_to_nearest_notch")
    check(translate_wheel_event(100, 2, 2) == "\x1b[<64;2;2M", "100 rounds to 1 notch")


def test_translate_wheel_event_zero_delta_is_empty():
    print("test_translate_wheel_event_zero_delta_is_empty")
    check(translate_wheel_event(0, 1, 1) == "", "zero delta forwards nothing")


def test_classify_record_key_event_is_not_mouse():
    print("test_classify_record_key_event_is_not_mouse")
    check(classify_record(KEY_EVENT_TYPE, 0) == "not_mouse",
          "key events left queued for the keyboard loop")


def test_classify_record_wheel_event():
    print("test_classify_record_wheel_event")
    check(classify_record(MOUSE_EVENT_TYPE, MOUSE_WHEELED) == "wheel",
          "vertical wheel -> forward")


def test_classify_record_click_is_other_mouse():
    print("test_classify_record_click_is_other_mouse")
    check(classify_record(MOUSE_EVENT_TYPE, 0) == "other_mouse",
          "click/drag/move -> dequeue and discard, not forwarded")


def test_classify_record_horizontal_wheel_is_other_mouse():
    print("test_classify_record_horizontal_wheel_is_other_mouse")
    check(classify_record(MOUSE_EVENT_TYPE, MOUSE_HWHEELED) == "other_mouse",
          "horizontal wheel is out of scope, treated as discard")


def test_input_record_mouse_event_field_roundtrip():
    print("test_input_record_mouse_event_field_roundtrip")
    rec = INPUT_RECORD()
    rec.EventType = MOUSE_EVENT_TYPE
    rec.Event.MouseEvent.dwMousePosition.X = 12
    rec.Event.MouseEvent.dwMousePosition.Y = 34
    rec.Event.MouseEvent.dwButtonState = 0x00780000
    rec.Event.MouseEvent.dwEventFlags = MOUSE_WHEELED

    check(rec.EventType == MOUSE_EVENT_TYPE, "EventType round-trips")
    check(rec.Event.MouseEvent.dwMousePosition.X == 12, "dwMousePosition.X round-trips")
    check(rec.Event.MouseEvent.dwMousePosition.Y == 34, "dwMousePosition.Y round-trips")
    check(rec.Event.MouseEvent.dwButtonState == 0x00780000, "dwButtonState round-trips")
    check(rec.Event.MouseEvent.dwEventFlags == MOUSE_WHEELED, "dwEventFlags round-trips")


def test_key_and_mouse_event_union_members_are_equal_size():
    print("test_key_and_mouse_event_union_members_are_equal_size")
    # Both real Win32 INPUT_RECORD union members (KEY_EVENT_RECORD,
    # MOUSE_EVENT_RECORD) are 16 bytes -- confirms the union sizes correctly
    # regardless of which variant a given record actually holds.
    check(ctypes.sizeof(ctypes.c_short) * 2 + ctypes.sizeof(ctypes.c_uint32) * 3 == 16,
          "COORD(2 SHORT) + 3 DWORD == 16 bytes, matching KEY_EVENT_RECORD's size")


def main():
    test_extract_wheel_delta_positive()
    test_extract_wheel_delta_negative()
    test_extract_wheel_delta_zero()
    test_translate_wheel_event_single_notch_up()
    test_translate_wheel_event_single_notch_down()
    test_translate_wheel_event_multi_notch_fast_scroll()
    test_translate_wheel_event_rounds_to_nearest_notch()
    test_translate_wheel_event_zero_delta_is_empty()
    test_classify_record_key_event_is_not_mouse()
    test_classify_record_wheel_event()
    test_classify_record_click_is_other_mouse()
    test_classify_record_horizontal_wheel_is_other_mouse()
    test_input_record_mouse_event_field_roundtrip()
    test_key_and_mouse_event_union_members_are_equal_size()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
