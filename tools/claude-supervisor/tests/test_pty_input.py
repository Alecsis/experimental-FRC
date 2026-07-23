"""Regression tests for the Windows PTY *input forwarding* path.

Guards the fix for the bug where local keyboard input into the proxied Claude
TUI was broken (Enter did nothing, Ctrl+C emitted ``;;;;;`` garbage, arrows
dead, only bare printables leaked through). Root cause: the console stdin was
put in ENABLE_VIRTUAL_TERMINAL_INPUT mode, which makes keys arrive as VT escape
sequences -- but ``_input_loop`` reads with ``msvcrt.getwch()``, the classic
0x00/0xe0 scan-code model, which cannot parse those sequences.

These are the *deterministic* parts of that path. The full loop (``msvcrt.kbhit``
/ ``getwch`` against a live console with real key presses) is NOT unit-testable
-- it needs a real interactive console -- so we test the two pure pieces the bug
hinged on: the console-mode plan and the per-key translation.

Run: python tests/test_pty_input.py
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# pty_session imports cleanly without pywinpty (the native import is lazy, inside
# start()); translate_keystroke / _console_vt_plan touch neither winpty nor ctypes.
from claude_supervisor.pty_session import (
    BRACKETED_PASTE_END,
    BRACKETED_PASTE_START,
    ENABLE_VIRTUAL_TERMINAL_INPUT,
    STDIN_HANDLE,
    _console_vt_plan,
    is_paste_burst,
    translate_keystroke,
    wrap_bracketed_paste,
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


def _feed(*chars):
    """A get_next() that yields the given follow-up chars in order."""
    it = iter(chars)
    return lambda: next(it)


def test_console_plan_never_enables_vt_input_on_stdin():
    """THE regression guard: re-enabling VT-input on stdin is the exact bug."""
    print("test_console_plan_never_enables_vt_input_on_stdin")
    plan = _console_vt_plan()
    check(all(handle_id != STDIN_HANDLE for handle_id, _ in plan),
          "plan touches no stdin handle")
    check(all(not (flag & ENABLE_VIRTUAL_TERMINAL_INPUT) for _, flag in plan),
          "VT-input bit (0x0200) appears in no plan entry")


def test_printable_and_control_chars_forward_verbatim():
    print("test_printable_and_control_chars_forward_verbatim")
    never = _feed()  # get_next must NOT be called for these
    check(translate_keystroke("a", never) == "a", "printable 'a' -> 'a'")
    check(translate_keystroke("\r", never) == "\r", "Enter '\\r' -> '\\r'")
    check(translate_keystroke("\x03", never) == "\x03", "Ctrl+C '\\x03' -> '\\x03' (not ';;;;;')")
    check(translate_keystroke(" ", never) == " ", "space -> space")


def test_extended_keys_translate_to_vt_sequences():
    print("test_extended_keys_translate_to_vt_sequences")
    # Both 0x00 and 0xe0 are valid lead bytes for extended keys.
    check(translate_keystroke("\xe0", _feed("H")) == "\x1b[A", "0xe0 + H -> Up (ESC[A)")
    check(translate_keystroke("\x00", _feed("H")) == "\x1b[A", "0x00 + H -> Up (ESC[A)")
    check(translate_keystroke("\xe0", _feed("P")) == "\x1b[B", "0xe0 + P -> Down (ESC[B)")
    check(translate_keystroke("\xe0", _feed("K")) == "\x1b[D", "0xe0 + K -> Left (ESC[D)")
    check(translate_keystroke("\xe0", _feed("S")) == "\x1b[3~", "0xe0 + S -> Delete")


def test_unknown_extended_key_consumes_lead_and_yields_nothing():
    print("test_unknown_extended_key_consumes_lead_and_yields_nothing")
    # An unmapped scan code must still consume its second byte and emit nothing,
    # so a stray follow-up byte is never mistaken for a standalone keystroke.
    check(translate_keystroke("\xe0", _feed("?")) == "", "unknown 0xe0 + '?' -> ''")


def test_typing_and_lone_enter_are_not_pastes():
    print("test_typing_and_lone_enter_are_not_pastes")
    check(not is_paste_burst("a"), "single char is not a paste")
    check(not is_paste_burst("hello"), "typed word (no newline) is not a paste")
    check(not is_paste_burst("\r"), "lone Enter is not a paste (must still submit)")
    check(not is_paste_burst("hello\r"), "single line + trailing Enter is not a paste")
    check(not is_paste_burst("\x1b[A"), "arrow-key VT sequence is not a paste")
    check(not is_paste_burst(""), "empty burst is not a paste")


def test_multiline_burst_is_a_paste():
    print("test_multiline_burst_is_a_paste")
    # The reported failure case: newline(s) with content after them.
    check(is_paste_burst("line1\rline2"), "two lines (CR) is a paste")
    check(is_paste_burst("line1\nline2"), "two lines (LF) is a paste")
    check(is_paste_burst("line1\r\nline2\r\n"), "CRLF multi-line w/ trailing NL is a paste")
    check(is_paste_burst("a\rb\rc"), "three short lines is a paste")


def test_wrap_bracketed_paste_preserves_content_and_newlines():
    print("test_wrap_bracketed_paste_preserves_content_and_newlines")
    body = "Analyze this repo.\r\rDo not modify files.\r\rThen ask me."
    wrapped = wrap_bracketed_paste(body)
    check(wrapped.startswith(BRACKETED_PASTE_START), "starts with ESC[200~")
    check(wrapped.endswith(BRACKETED_PASTE_END), "ends with ESC[201~")
    inner = wrapped[len(BRACKETED_PASTE_START):-len(BRACKETED_PASTE_END)]
    check(inner == body, "inner content is the paste verbatim, newlines preserved")
    check(inner.count("\r") == body.count("\r"), "every newline preserved (none dropped)")


def main():
    test_console_plan_never_enables_vt_input_on_stdin()
    test_printable_and_control_chars_forward_verbatim()
    test_extended_keys_translate_to_vt_sequences()
    test_unknown_extended_key_consumes_lead_and_yields_nothing()
    test_typing_and_lone_enter_are_not_pastes()
    test_multiline_burst_is_a_paste()
    test_wrap_bracketed_paste_preserves_content_and_newlines()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
