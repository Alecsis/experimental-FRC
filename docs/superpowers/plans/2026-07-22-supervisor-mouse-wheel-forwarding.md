# Supervisor Mouse Wheel Forwarding Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make mouse-wheel scrolling under the Claude Code Remote Supervisor behave like it does when running `claude` directly — Claude's own TUI consumes the scroll, instead of the outer Windows Terminal showing raw scrollback.

**Architecture:** A new `_mouse_loop` daemon thread in `PtySession` enables `ENABLE_MOUSE_INPUT` (and disables `ENABLE_QUICK_EDIT_MODE`) on the supervisor's own console stdin, then peeks the Win32 console input queue. Wheel events are dequeued, translated to xterm SGR mouse sequences, and forwarded into Claude's PTY via the existing `send_keys()`. Non-wheel mouse events are dequeued and discarded. Key events are left untouched in the queue for the existing `msvcrt`-based `_input_loop` to consume — the two loops never race for the same record because the mouse loop only ever dequeues what it peeked and classified as mouse.

**Tech Stack:** Python 3.14, `ctypes` (Win32 `kernel32` console APIs), `pytest`. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-07-22-supervisor-mouse-wheel-forwarding-design.md`

## Global Constraints

- Windows-only feature (`os.name == "nt"` guard), matching the existing `_enable_vt_console()` pattern.
- Do not modify `strip_mouse_tracking()`, `_read_loop`, `_input_loop`, or any existing function's behavior — this is additive only.
- Every new *pure* function (no Win32 syscalls) must ship with unit tests written first (TDD). Functions that directly call Win32 console APIs are not unit-testable in isolation (same category as the existing, untested `_enable_vt_console()`) and are verified only by manual live check.
- The full existing suite (`test_pty_mouse_strip`, `test_pty_resize`, `test_pty_input`, `test_core`, `test_bot_mentions`, `test_snapshot`) must stay green after every task.
- Out of scope, do not implement: click/drag mouse-selection translation, Ctrl+Tab / modifier-key forwarding, horizontal wheel.

---

### Task 1: Wheel-delta extraction and SGR translation (pure functions)

**Files:**
- Modify: `tools/claude-supervisor/claude_supervisor/pty_session.py`
- Test: `tools/claude-supervisor/tests/test_pty_mouse_wheel.py` (new)

**Interfaces:**
- Produces: `extract_wheel_delta(dw_button_state: int) -> int`, `translate_wheel_event(delta: int, col: int, row: int) -> str`, `classify_record(event_type: int, event_flags: int) -> str` (returns `"wheel"` / `"other_mouse"` / `"not_mouse"`), and constants `MOUSE_EVENT_TYPE = 0x0002`, `KEY_EVENT_TYPE = 0x0001`, `MOUSE_WHEELED = 0x0004`, `MOUSE_HWHEELED = 0x0008`. Task 4 consumes all of these by name.

- [ ] **Step 1: Write the failing tests**

Create `tools/claude-supervisor/tests/test_pty_mouse_wheel.py`:

```python
from claude_supervisor.pty_session import (
    extract_wheel_delta,
    translate_wheel_event,
    classify_record,
    MOUSE_EVENT_TYPE,
    KEY_EVENT_TYPE,
    MOUSE_WHEELED,
    MOUSE_HWHEELED,
)


def test_extract_wheel_delta_positive():
    # dwButtonState with high word 0x0078 (120) -> +120
    assert extract_wheel_delta(0x00780000) == 120


def test_extract_wheel_delta_negative():
    # dwButtonState with high word 0xFF88 (-120 as signed 16-bit) -> -120
    assert extract_wheel_delta(0xFF880000) == -120


def test_extract_wheel_delta_zero():
    assert extract_wheel_delta(0x00000000) == 0


def test_translate_wheel_event_single_notch_up():
    assert translate_wheel_event(120, 5, 10) == "\x1b[<64;5;10M"


def test_translate_wheel_event_single_notch_down():
    assert translate_wheel_event(-120, 5, 10) == "\x1b[<65;5;10M"


def test_translate_wheel_event_multi_notch_fast_scroll():
    assert translate_wheel_event(360, 1, 1) == "\x1b[<64;1;1M" * 3


def test_translate_wheel_event_rounds_to_nearest_notch():
    assert translate_wheel_event(100, 2, 2) == "\x1b[<64;2;2M"


def test_translate_wheel_event_zero_delta_is_empty():
    assert translate_wheel_event(0, 1, 1) == ""


def test_classify_record_key_event_is_not_mouse():
    assert classify_record(KEY_EVENT_TYPE, 0) == "not_mouse"


def test_classify_record_wheel_event():
    assert classify_record(MOUSE_EVENT_TYPE, MOUSE_WHEELED) == "wheel"


def test_classify_record_click_is_other_mouse():
    assert classify_record(MOUSE_EVENT_TYPE, 0) == "other_mouse"


def test_classify_record_horizontal_wheel_is_other_mouse():
    assert classify_record(MOUSE_EVENT_TYPE, MOUSE_HWHEELED) == "other_mouse"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd tools/claude-supervisor && python -m pytest tests/test_pty_mouse_wheel.py -v`
Expected: FAIL with `ImportError: cannot import name 'extract_wheel_delta'` (module doesn't have these yet).

- [ ] **Step 3: Implement the pure functions**

In `tools/claude-supervisor/claude_supervisor/pty_session.py`, add this block immediately after the existing `wrap_bracketed_paste` function (after line 134's `return BRACKETED_PASTE_START + s + BRACKETED_PASTE_END`, before the `# Windows console std-handle ids` comment):

```python
# Win32 console INPUT_RECORD.EventType values (wincon.h).
KEY_EVENT_TYPE = 0x0001
MOUSE_EVENT_TYPE = 0x0002

# Win32 MOUSE_EVENT_RECORD.dwEventFlags bits relevant here (wincon.h).
MOUSE_WHEELED = 0x0004
MOUSE_HWHEELED = 0x0008

_WHEEL_DELTA = 120  # Win32's notch unit; MOUSE_EVENT_RECORD.dwButtonState's high word.

# xterm SGR mouse-report button codes for the vertical wheel.
_SGR_WHEEL_UP = 64
_SGR_WHEEL_DOWN = 65


def extract_wheel_delta(dw_button_state: int) -> int:
    """Extract the signed 16-bit wheel delta from a raw MOUSE_EVENT_RECORD.dwButtonState.

    Win32 packs the wheel delta into the high word of this otherwise-unsigned DWORD;
    positive means rotated away from the user ("up"), negative means toward the user
    ("down"). The low word (button press state) is irrelevant for wheel events and
    ignored here.
    """
    high_word = (dw_button_state >> 16) & 0xFFFF
    return high_word - 0x10000 if high_word >= 0x8000 else high_word


def translate_wheel_event(delta: int, col: int, row: int) -> str:
    """Translate a raw wheel delta into SGR xterm mouse-wheel sequence(s).

    ``delta`` is signed (positive = up, negative = down), typically a multiple of 120
    (one Win32 "notch"). ``col``/``row`` are passed straight through into the SGR
    event's reported position. Emits one SGR sequence per notch -- a fast scroll
    producing a larger delta emits multiple stacked sequences, since the SGR wheel
    protocol has no magnitude field of its own.
    """
    if delta == 0:
        return ""
    notches = max(1, round(abs(delta) / _WHEEL_DELTA))
    button = _SGR_WHEEL_UP if delta > 0 else _SGR_WHEEL_DOWN
    return f"\x1b[<{button};{col};{row}M" * notches


def classify_record(event_type: int, event_flags: int) -> str:
    """Classify a peeked console INPUT_RECORD for the mouse loop's dequeue decision.

    Returns ``"not_mouse"`` (leave queued for the keyboard loop), ``"wheel"``
    (dequeue and forward), or ``"other_mouse"`` (dequeue and discard -- clicks,
    drags, moves, horizontal wheel; mouse-selection support is a separate,
    deferred item).
    """
    if event_type != MOUSE_EVENT_TYPE:
        return "not_mouse"
    return "wheel" if event_flags & MOUSE_WHEELED else "other_mouse"
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd tools/claude-supervisor && python -m pytest tests/test_pty_mouse_wheel.py -v`
Expected: PASS, 10/10.

- [ ] **Step 5: Run full existing suite to confirm no regression**

Run: `cd tools/claude-supervisor && python -m pytest tests/ -v`
Expected: all prior suites still pass, plus the 10 new tests (86 + 10 = 96 total, matching the count carried over from the twenty-sixth session's `4b45a21`).

- [ ] **Step 6: Commit**

```bash
git add tools/claude-supervisor/claude_supervisor/pty_session.py tools/claude-supervisor/tests/test_pty_mouse_wheel.py
git commit -m "supervisor: add wheel-delta extraction and SGR translation"
```

---

### Task 2: Win32 INPUT_RECORD ctypes structures

**Files:**
- Modify: `tools/claude-supervisor/claude_supervisor/pty_session.py`
- Test: `tools/claude-supervisor/tests/test_pty_mouse_wheel.py`

**Interfaces:**
- Consumes: nothing from Task 1.
- Produces: `COORD`, `MOUSE_EVENT_RECORD`, `KEY_EVENT_RECORD`, `INPUT_RECORD_EVENT` (union), `INPUT_RECORD` ctypes classes. Task 4 consumes `INPUT_RECORD` (constructs one, passes it to the Win32 console read calls, reads `.EventType` and `.Event.MouseEvent.*` off it).

These are exact-layout mirrors of `wincon.h`'s real structs -- getting the byte layout wrong risks the OS writing a real console event into a mis-sized/misaligned Python buffer. `KEY_EVENT_RECORD` and `MOUSE_EVENT_RECORD` are both included in the union (even though only `MouseEvent` is ever read) because both are real members of the true `INPUT_RECORD` union and determine its total size -- both happen to be 16 bytes, so the union sizes correctly either way, but only including one variant would be a coincidence, not a guarantee.

- [ ] **Step 1: Write the failing test**

Add to `tools/claude-supervisor/tests/test_pty_mouse_wheel.py`:

```python
import ctypes

from claude_supervisor.pty_session import INPUT_RECORD, MOUSE_EVENT_TYPE, MOUSE_WHEELED


def test_input_record_mouse_event_field_roundtrip():
    rec = INPUT_RECORD()
    rec.EventType = MOUSE_EVENT_TYPE
    rec.Event.MouseEvent.dwMousePosition.X = 12
    rec.Event.MouseEvent.dwMousePosition.Y = 34
    rec.Event.MouseEvent.dwButtonState = 0x00780000
    rec.Event.MouseEvent.dwEventFlags = MOUSE_WHEELED

    assert rec.EventType == MOUSE_EVENT_TYPE
    assert rec.Event.MouseEvent.dwMousePosition.X == 12
    assert rec.Event.MouseEvent.dwMousePosition.Y == 34
    assert rec.Event.MouseEvent.dwButtonState == 0x00780000
    assert rec.Event.MouseEvent.dwEventFlags == MOUSE_WHEELED


def test_input_record_mouse_event_size_matches_key_event():
    # Both real Win32 union members are 16 bytes -- the union (and therefore
    # INPUT_RECORD as a whole) must size identically regardless of which
    # member ctypes computed sizeof from, or the layout is wrong.
    assert ctypes.sizeof(ctypes.c_short) * 2 + ctypes.sizeof(ctypes.c_uint32) * 3 == 16
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd tools/claude-supervisor && python -m pytest tests/test_pty_mouse_wheel.py -v`
Expected: FAIL with `ImportError: cannot import name 'INPUT_RECORD'`.

- [ ] **Step 3: Implement the ctypes structures**

In `pty_session.py`, add this block right after the `classify_record` function from Task 1, still before the `# Windows console std-handle ids` section:

```python
from ctypes import wintypes


class _COORD(ctypes.Structure):
    _fields_ = [("X", wintypes.SHORT), ("Y", wintypes.SHORT)]


class MOUSE_EVENT_RECORD(ctypes.Structure):
    _fields_ = [
        ("dwMousePosition", _COORD),
        ("dwButtonState", wintypes.DWORD),
        ("dwControlKeyState", wintypes.DWORD),
        ("dwEventFlags", wintypes.DWORD),
    ]


class _CHAR_UNION(ctypes.Union):
    _fields_ = [("UnicodeChar", wintypes.WCHAR), ("AsciiChar", wintypes.CHAR)]


class KEY_EVENT_RECORD(ctypes.Structure):
    _fields_ = [
        ("bKeyDown", wintypes.BOOL),
        ("wRepeatCount", wintypes.WORD),
        ("wVirtualKeyCode", wintypes.WORD),
        ("wVirtualScanCode", wintypes.WORD),
        ("uChar", _CHAR_UNION),
        ("dwControlKeyState", wintypes.DWORD),
    ]


class INPUT_RECORD_EVENT(ctypes.Union):
    _fields_ = [
        ("KeyEvent", KEY_EVENT_RECORD),
        ("MouseEvent", MOUSE_EVENT_RECORD),
    ]


class INPUT_RECORD(ctypes.Structure):
    _fields_ = [
        ("EventType", wintypes.WORD),
        ("Event", INPUT_RECORD_EVENT),
    ]
```

Add the `import ctypes` module already exists at the top of the file (line 15) -- do not duplicate it; only add the `from ctypes import wintypes` line shown above, placed with the other imports at the top of the file (after the existing `import ctypes` on line 15) rather than inline before the class block.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd tools/claude-supervisor && python -m pytest tests/test_pty_mouse_wheel.py -v`
Expected: PASS, 12/12.

- [ ] **Step 5: Run full existing suite to confirm no regression**

Run: `cd tools/claude-supervisor && python -m pytest tests/ -v`
Expected: all pass, 98 total.

- [ ] **Step 6: Commit**

```bash
git add tools/claude-supervisor/claude_supervisor/pty_session.py tools/claude-supervisor/tests/test_pty_mouse_wheel.py
git commit -m "supervisor: add Win32 INPUT_RECORD ctypes structures for mouse events"
```

---

### Task 3: Console mode toggle (enable mouse input, disable QuickEdit)

**Files:**
- Modify: `tools/claude-supervisor/claude_supervisor/pty_session.py`

**Interfaces:**
- Consumes: `STDIN_HANDLE` (already exists at line 139), the existing `kernel32`-via-`ctypes.windll` pattern from `_enable_vt_console()`.
- Produces: `_enable_mouse_console()` function, `ENABLE_MOUSE_INPUT`, `ENABLE_QUICK_EDIT_MODE`, `ENABLE_EXTENDED_FLAGS` constants. Task 4's `start()` wiring calls `_enable_mouse_console()`.

No unit test for this task: it is a real Win32 console-mode side effect with no meaningful way to fake `GetConsoleMode`/`SetConsoleMode` short of mocking the whole `ctypes.windll` surface, which would test the mock, not the behavior. This matches the existing, deliberately-untested `_enable_vt_console()` a few lines away. Verified only by the manual live check in Task 5.

- [ ] **Step 1: Add the mode constants**

In `pty_session.py`, find the existing block (currently around line 140):

```python
STDOUT_HANDLE = -11
STDIN_HANDLE = -10
ENABLE_VIRTUAL_TERMINAL_PROCESSING = 0x0004
ENABLE_VIRTUAL_TERMINAL_INPUT = 0x0200
```

Replace it with:

```python
STDOUT_HANDLE = -11
STDIN_HANDLE = -10
ENABLE_VIRTUAL_TERMINAL_PROCESSING = 0x0004
ENABLE_VIRTUAL_TERMINAL_INPUT = 0x0200
ENABLE_MOUSE_INPUT = 0x0010
ENABLE_QUICK_EDIT_MODE = 0x0040
ENABLE_EXTENDED_FLAGS = 0x0080
```

- [ ] **Step 2: Implement `_enable_mouse_console()`**

Add this function immediately after the existing `_enable_vt_console()` function (after its closing lines, currently ending around line 165 with `kernel32.SetConsoleMode(handle, mode.value | flag)`):

```python
def _enable_mouse_console() -> None:
    """Enable mouse-event reporting on this process's own stdin console handle.

    Windows Terminal's default QuickEdit mode claims wheel/click events for its own
    text-selection and scrollback UI, which is why the wheel showed the outer
    console's raw scrollback instead of reaching Claude (see strip_mouse_tracking's
    docstring for the other half of that story). Turning ENABLE_QUICK_EDIT_MODE off
    hands mouse events to this process instead, via ReadConsoleInputW/
    PeekConsoleInputW (see PtySession._mouse_loop) -- ENABLE_EXTENDED_FLAGS must be
    set in the same call for ENABLE_QUICK_EDIT_MODE to take effect at all (an
    undocumented-but-well-known SetConsoleMode quirk).
    """
    if os.name != "nt":
        return
    kernel32 = ctypes.windll.kernel32
    handle = kernel32.GetStdHandle(STDIN_HANDLE)
    mode = ctypes.c_uint32()
    if not kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
        return
    new_mode = (mode.value & ~ENABLE_QUICK_EDIT_MODE) | ENABLE_EXTENDED_FLAGS | ENABLE_MOUSE_INPUT
    kernel32.SetConsoleMode(handle, new_mode)
```

- [ ] **Step 3: Run full existing suite to confirm no regression**

Run: `cd tools/claude-supervisor && python -m pytest tests/ -v`
Expected: all pass, 98 total (this task adds no new tests -- see rationale above).

- [ ] **Step 4: Commit**

```bash
git add tools/claude-supervisor/claude_supervisor/pty_session.py
git commit -m "supervisor: add console mode toggle for mouse-wheel capture"
```

---

### Task 4: `_mouse_loop` thread and `start()` wiring

**Files:**
- Modify: `tools/claude-supervisor/claude_supervisor/pty_session.py`

**Interfaces:**
- Consumes: `classify_record`, `extract_wheel_delta`, `translate_wheel_event` (Task 1), `INPUT_RECORD`, `MOUSE_EVENT_TYPE` (Task 2), `_enable_mouse_console()` (Task 3), `self.send_keys()` (existing), `self._forward_local_input` (existing, line 183).
- Produces: `PtySession._mouse_loop()` method, `self._mouser` thread attribute, wired into `start()`.

No unit test for the thread body itself -- `PeekConsoleInputW`/`ReadConsoleInputW` require a real attached console; this is the same category as `_input_loop`/`_resize_loop`, neither of which has a unit test today either (their pure helpers -- `translate_keystroke`, `poll_resize` -- are what's tested, exactly as Task 1 already did for this feature). Verified only by the manual live check in Task 5.

- [ ] **Step 1: Add the `_mouser` attribute**

In `PtySession.__init__` (around line 168-194), find:

```python
        self._reader: Optional[threading.Thread] = None
        self._input: Optional[threading.Thread] = None
        self._resizer: Optional[threading.Thread] = None
        self._last_size: Optional[tuple[int, int]] = None
```

Replace with:

```python
        self._reader: Optional[threading.Thread] = None
        self._input: Optional[threading.Thread] = None
        self._resizer: Optional[threading.Thread] = None
        self._mouser: Optional[threading.Thread] = None
        self._last_size: Optional[tuple[int, int]] = None
```

- [ ] **Step 2: Wire `_mouse_loop` into `start()`**

Find, in `start()` (around lines 219-223):

```python
        if self._forward_local_input:
            self._input = threading.Thread(target=self._input_loop, name="pty-input", daemon=True)
            self._input.start()
        self._resizer = threading.Thread(target=self._resize_loop, name="pty-resize", daemon=True)
        self._resizer.start()
```

Replace with:

```python
        if self._forward_local_input:
            self._input = threading.Thread(target=self._input_loop, name="pty-input", daemon=True)
            self._input.start()
            _enable_mouse_console()
            self._mouser = threading.Thread(target=self._mouse_loop, name="pty-mouse", daemon=True)
            self._mouser.start()
        self._resizer = threading.Thread(target=self._resize_loop, name="pty-resize", daemon=True)
        self._resizer.start()
```

(Mouse forwarding is gated on `_forward_local_input` -- same as keyboard -- since it has no meaning when local input forwarding is disabled.)

- [ ] **Step 3: Implement `_mouse_loop`**

Add this method to the `PtySession` class, immediately after `_input_loop` (after its closing line, currently ending around line 343 with `self.send_keys(burst)`), before `_resize_loop`:

```python
    def _mouse_loop(self) -> None:
        """Forward local mouse-wheel events to the child as SGR mouse sequences.

        Peeks the console input queue rather than blind-reading it: a key event left
        in place by classify_record's "not_mouse" branch must still be there for
        _input_loop's msvcrt calls to pick up. Only mouse-typed records are ever
        actually dequeued here -- wheel events are translated and forwarded, other
        mouse events (click/drag/move/horizontal-wheel) are dequeued and dropped.
        """
        if os.name != "nt":
            return
        kernel32 = ctypes.windll.kernel32
        handle = kernel32.GetStdHandle(STDIN_HANDLE)
        record = INPUT_RECORD()
        num_events = ctypes.c_uint32()
        while not self._stop.is_set():
            if not kernel32.PeekConsoleInputW(handle, ctypes.byref(record), 1, ctypes.byref(num_events)):
                self._stop.wait(0.02)
                continue
            if num_events.value == 0:
                self._stop.wait(0.02)
                continue
            kind = classify_record(record.EventType, record.Event.MouseEvent.dwEventFlags)
            if kind == "not_mouse":
                self._stop.wait(0.02)
                continue
            if not kernel32.ReadConsoleInputW(handle, ctypes.byref(record), 1, ctypes.byref(num_events)):
                continue
            if kind != "wheel":
                continue
            delta = extract_wheel_delta(record.Event.MouseEvent.dwButtonState)
            col = record.Event.MouseEvent.dwMousePosition.X + 1
            row = record.Event.MouseEvent.dwMousePosition.Y + 1
            sequence = translate_wheel_event(delta, col, row)
            if sequence:
                self.send_keys(sequence)
```

Note: `classify_record` reads `record.Event.MouseEvent.dwEventFlags` even before confirming `EventType == MOUSE_EVENT_TYPE` is checked by the caller -- this is safe because `classify_record` itself checks `event_type != MOUSE_EVENT_TYPE` first and returns `"not_mouse"` without the caller needing to branch; reading `Event.MouseEvent` on a record that's actually a `KeyEvent` just reads the union's other interpretation of the same bytes, which is never used since `classify_record` already discarded it via the type check.

**Known residual risk, not fully closeable within "wheel only" scope:** `msvcrt.getwch()`'s C-runtime implementation also reads from this same Win32 console input queue internally (it's how `conio`-style raw input has always worked on Windows) -- peek-then-conditionally-read in `_mouse_loop` prevents *this* code from stealing key events, but can't guarantee the CRT's own internal queue scan never interacts with a mouse record sitting ahead of a key event. Eliminating this fully would mean a single unified `ReadConsoleInputW`-based reader for both keyboard and mouse -- the wider rework already declined for this pass. If Task 5's live check shows occasional dropped or delayed keystrokes that weren't there before, this shared queue is the first place to look, not a reason to add another patch blind.

- [ ] **Step 4: Run full existing suite to confirm no regression**

Run: `cd tools/claude-supervisor && python -m pytest tests/ -v`
Expected: all pass, 98 total.

- [ ] **Step 5: Module-import sanity check**

Run: `cd tools/claude-supervisor && python -c "import claude_supervisor.pty_session"`
Expected: no output, exit code 0 (confirms the new ctypes structures and threading wiring don't break import on this machine).

- [ ] **Step 6: Commit**

```bash
git add tools/claude-supervisor/claude_supervisor/pty_session.py
git commit -m "supervisor: forward mouse-wheel events into Claude's PTY"
```

---

### Task 5: Full regression run and manual live verification handoff

**Files:** none (verification only, no code changes).

- [ ] **Step 1: Run the full test suite one more time**

Run: `cd tools/claude-supervisor && python -m pytest tests/ -v`
Expected: PASS, 98/98 (`test_pty_mouse_wheel` 12, `test_pty_mouse_strip` 6, `test_pty_resize` 4, `test_pty_input` 26, `test_core` 22, `test_snapshot` 19, `test_bot_mentions` 9).

- [ ] **Step 2: Hand off the manual live check**

This cannot be verified by an automated test -- it requires a real Windows Terminal session and a human confirming subjective scroll behavior. Report to the mentor:

> Implementation complete, full suite green. Please run the supervisor (`python run.py` from `tools/claude-supervisor/`), launch Claude, and:
> 1. Scroll the mouse wheel up/down in the Claude pane -- confirm it now scrolls **Claude's own chat view** (like running `claude` directly), not the outer Windows Terminal's raw scrollback.
> 2. Confirm keyboard input and multi-line paste still work exactly as before (regression check on `_input_loop`, which this change runs alongside but does not modify).
>
> If scroll still shows raw terminal history instead of Claude's chat view, or if keyboard input misbehaves (dropped/delayed keystrokes -- see Task 4's "known residual risk" note on the shared console input queue), report exactly what you see -- next step would be re-opening root-cause investigation per systematic-debugging, not another blind patch.

No commit for this task (verification only).
