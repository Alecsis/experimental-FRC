# Claude Supervisor — Mouse Wheel Forwarding — Design

## Why

The twenty-sixth session fixed a mouse-tracking-leak bug in `tools/claude-supervisor/claude_supervisor/pty_session.py`:
Claude Code's TUI enables xterm mouse-tracking (`DECSET ?1000h`/`?1006h`) for its own controlling
terminal, and the supervisor's raw byte passthrough (`_read_loop`) was forwarding that leak to the
*outer* Windows Terminal console too. `strip_mouse_tracking()` removed the leaking sequences from
the passthrough — which fixed "mouse wheel does nothing," but exposed the real underlying gap:
`_input_loop` only ever reads keyboard (`msvcrt.getwch()`/`kbhit()`). There has never been a path
for mouse events to reach Claude at all. With the DECSET no longer leaking, Windows Terminal falls
back to QuickEdit-style native scrollback on wheel — the mentor's live-test report this session:
"the scroll wheel doesn't go up in chat it scrolls the past messages which I dont want."

This spec adds the missing path: capture real mouse-wheel events on the supervisor's own console
and forward them into Claude as SGR mouse-report sequences, so wheel-scroll behaves like it does
when running `claude` directly (no supervisor) — Claude's own TUI consumes the scroll and moves
within its live chat view, instead of the outer terminal showing raw historical output.

## Scope

**In scope:** vertical mouse-wheel events only, forwarded from the supervisor's outer console into
Claude's PTY stdin.

**Explicitly out of scope (deferred, tracked separately in `CLAUDE.md`'s Next Up):**
- Click / drag mouse-selection and copy (issue 2) — this pass only reads and discards non-wheel
  mouse records, doesn't translate them.
- Ctrl+Tab / other modifier-key combos (issue 3) — a full `ReadConsoleInputW` keyboard rework
  (`KEY_EVENT_RECORD.dwControlKeyState`) was considered and declined for this pass; the existing
  `msvcrt`-based `_input_loop` is left untouched.
- Horizontal wheel (`MOUSE_HWHEELED`) — not requested, not translated.

## Approaches considered

**A — Native Win32 mouse capture via `ReadConsoleInputW` (chosen).** Enable `ENABLE_MOUSE_INPUT`
on the supervisor's own stdin handle and disable `ENABLE_QUICK_EDIT_MODE` (QuickEdit is what
currently claims the wheel for native scrollback — disabling it is required, not incidental). A
new thread peeks the console input queue; when the next record is a wheel event, it dequeues just
that record and leaves everything else for the existing keyboard loop.

**B — Stop stripping Claude's DECSET, put stdin in VT-input mode, relay raw bytes.** Rejected:
`ENABLE_VIRTUAL_TERMINAL_INPUT` on stdin is exactly what `_console_vt_plan()`'s existing comment
says breaks `msvcrt.getwch()` (corrupts Enter/Ctrl+C/arrows). Doing this properly means rewriting
keyboard input too — the wider rework the mentor explicitly declined for this pass.

**C — Global low-level mouse hook (`WH_MOUSE_LL`).** Rejected: system-wide capture regardless of
window focus, needs its own Win32 message loop and window-handle correlation to filter to this
console. More invasive and less standard than the console API every other console app uses.

## Design

### Mechanism

A new `_mouse_loop` daemon thread, started from `PtySession.start()` alongside the existing
`_input_loop`/`_resize_loop` (gated the same way `_forward_local_input` gates `_input_loop`, since
mouse forwarding has no meaning without local input forwarding either).

Startup, once, before the loop begins:
1. Read the current stdin console mode.
2. Clear `ENABLE_QUICK_EDIT_MODE`, set `ENABLE_MOUSE_INPUT` (and keep `ENABLE_EXTENDED_FLAGS`,
   required by Windows whenever `ENABLE_QUICK_EDIT_MODE` is touched programmatically).
3. Write the new mode back.

Loop body, each iteration:
1. `PeekConsoleInputW(stdin_handle, 1)` — look at the next queued record without removing it.
2. If nothing queued, wait briefly (mirrors `_input_loop`'s `0.02s` idle wait) and retry.
3. If the peeked record is a `MOUSE_EVENT` with the `MOUSE_WHEELED` flag set in `dwEventFlags`:
   `ReadConsoleInputW(stdin_handle, 1)` to actually dequeue it, extract the signed wheel delta
   (`dwButtonState`'s high 16 bits) and cursor position (`dwMousePosition`), pass both to
   `translate_wheel_event`, and `send_keys()` the result to the child.
4. If the peeked record is any other `MOUSE_EVENT` (click, drag, move, horizontal wheel):
   dequeue and discard it — must still be consumed so the queue doesn't back up and starve
   real wheel events behind it, but produces no output.
5. If the peeked record is a `KEY_EVENT` (or anything else): **do not dequeue it.** Leave it in
   the queue for the existing `_input_loop`'s `msvcrt` calls to consume through their own,
   separate path. This is the coexistence mechanism — peek-then-conditionally-read means the two
   loops never race for the same record.

### Translation

A new pure function, mirroring the existing `poll_resize`/`translate_keystroke`/`is_paste_burst`
pattern of keeping the untestable Win32 plumbing thin and the logic itself pure and unit-tested:

```python
def translate_wheel_event(delta: int, col: int, row: int) -> str:
    """Translate a raw Win32 wheel delta into SGR xterm mouse-wheel sequence(s).

    ``delta`` is the signed value from MOUSE_EVENT_RECORD.dwButtonState's high word (positive =
    up, negative = down; ~120 per notch on stock Windows wheel settings, though some
    trackpads/mice report other magnitudes). ``col``/``row`` are 1-based console coordinates from
    dwMousePosition, passed straight through as the SGR event's reported position.

    Emits one SGR sequence per full notch (round(delta / 120), minimum 1 if delta is nonzero) --
    a fast scroll produces multiple stacked sequences, matching how real terminals report
    multi-notch scrolls as repeated single-notch events rather than one event with a magnitude
    field (the SGR wheel protocol has no magnitude field).
    """
```

Button codes: `64` = wheel up, `65` = wheel down (xterm SGR mouse protocol, no modifier bits set —
modifier-aware wheel events are out of scope, matching the Ctrl+Tab deferral above). Sequence
shape: `\x1b[<{button};{col};{row}M`.

### Filtering

Only `MOUSE_WHEELED` records produce output. All other mouse record types (button clicks, drags,
plain moves, `MOUSE_HWHEELED`) are peeked, confirmed to be mouse-but-not-vertical-wheel, dequeued,
and dropped — never translated, never forwarded. This is what keeps the change minimal: the
supervisor now *owns* mouse input on its console (required to get wheel events at all, since
QuickEdit has to come off), but only acts on the one event type this spec is about.

### Side effect to flag explicitly

Disabling `ENABLE_QUICK_EDIT_MODE` also turns off Windows Terminal's click-drag text-selection
convenience in the outer console. This is already the mouse-select/copy gap tracked as issue 2 in
`CLAUDE.md`'s Next Up — today it's not working as desired either way, so this doesn't regress a
working feature. Worth the mentor knowing explicitly rather than discovering it as a surprise.

### Testing

- `translate_wheel_event`: unit tests for delta→notch count (single notch, multi-notch fast
  scroll), direction (positive delta → button 64, negative → button 65), coordinate passthrough,
  and the zero-delta edge case (should not happen from real hardware but must not crash/loop).
- The `PeekConsoleInputW`/`ReadConsoleInputW` ctypes plumbing and the console-mode toggle are not
  unit-testable in isolation — same category as the existing, untested `_enable_vt_console()`.
  Verified only by manual live check: mentor runs the supervisor, scrolls the wheel, confirms
  Claude's own chat view scrolls (matching native `claude` behavior) instead of showing outer
  Windows Terminal scrollback, **and** confirms keyboard input/paste still behaves exactly as
  before (regression check on the untouched `_input_loop` coexistence).
- Full existing suite (`test_pty_mouse_strip`, `test_pty_resize`, `test_pty_input`, `test_core`,
  `test_bot_mentions`, `test_snapshot`) must stay green — this change adds a new thread and a new
  pure function, touches no existing function's behavior.

## Non-goals (explicit, for future sessions to check before assuming gaps)

- Not fixing mouse click/drag selection (issue 2).
- Not fixing Ctrl+Tab or any modifier-key combo (issue 3).
- Not changing `strip_mouse_tracking()` or the outer passthrough — it stays exactly as the
  twenty-sixth session left it; this spec adds a parallel capability, not a replacement.
- Not attempting horizontal scroll.
