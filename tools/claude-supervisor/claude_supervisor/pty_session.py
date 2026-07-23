"""PTY ownership of the Claude Code process (Windows / ConPTY via pywinpty).

The supervisor launches Claude *inside* a pseudo-terminal so it can:
  * forward the child's output to the real console (the user still sees the TUI),
  * keep a rolling ANSI-stripped buffer for the Discord !logs command,
  * write into the child's stdin  -- the one capability hooks cannot provide,
    which is what makes remote injection (!reply / Approve / Ctrl+C) possible.

State *detection* does NOT come from parsing this stream; it comes from Claude
Code hooks (see event_sink.py). The PTY is here purely for I/O ownership.
"""

from __future__ import annotations

import ctypes
import os
import re
import sys
import threading
from collections import deque
from typing import Any, Callable, Optional

# pywinpty is imported lazily in start() so this module (and core.py) can be
# imported / unit-tested without the native dependency present.

# Matches CSI / OSC / other ANSI escape sequences for stripping before logging.
_ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]|\x1b\][^\x07]*(?:\x07|\x1b\\)|\x1b[@-Z\\-_]")

# Matches DECSET/DECRST xterm mouse-tracking mode sequences (X10/normal/
# button-event/any-event/focus/SGR). Claude Code's TUI enables these for its
# own controlling terminal; confirmed live (mouse_debug.log) that the raw byte
# passthrough was forwarding them to the *outer* console too, which made
# Windows Terminal route mouse-wheel/click events to the supervisor process
# (which never reads mouse input) instead of doing native scrollback. Stripped
# out of the passthrough in _read_loop -- see strip_mouse_tracking.
_MOUSE_MODE_RE = re.compile(r"\x1b\[\?(100[0-6]|1015)[hl]")

CTRL_C = "\x03"

# msvcrt extended-key (prefix 0x00 / 0xe0) second-byte -> ANSI escape sequence.
_SPECIAL_KEYS = {
    "H": "\x1b[A",  # Up
    "P": "\x1b[B",  # Down
    "M": "\x1b[C",  # Right
    "K": "\x1b[D",  # Left
    "G": "\x1b[H",  # Home
    "O": "\x1b[F",  # End
    "I": "\x1b[5~", # PageUp
    "Q": "\x1b[6~", # PageDown
    "S": "\x1b[3~", # Delete
}


def strip_ansi(text: str) -> str:
    return _ANSI_RE.sub("", text)


def strip_mouse_tracking(text: str) -> str:
    """Remove xterm mouse-tracking DECSET/DECRST sequences before passthrough.

    Leaves everything else -- including bracketed-paste (?2004), an unrelated
    mechanism -- untouched.
    """
    return _MOUSE_MODE_RE.sub("", text)


# How often to poll the local console for a size change and forward it into the
# inner ConPTY. Windows has no SIGWINCH, so polling is the standard approach;
# this interval is short enough to feel live but cheap enough to not matter.
_RESIZE_POLL_SECONDS = 0.3


def poll_resize(last: "Optional[tuple[int, int]]", get_size: Callable[[], Any]) -> "Optional[tuple[int, int]]":
    """Return the new ``(rows, cols)`` if the terminal size changed since ``last``.

    ``get_size`` mirrors ``os.get_terminal_size`` -- anything exposing
    ``.columns``/``.lines``. Returns ``None`` (no change to report) both when the
    size is unchanged and when ``get_size`` raises ``OSError`` (no real console
    attached, e.g. redirected output) -- resize forwarding just goes quiet in
    that case rather than crashing the poll loop.
    """
    try:
        size = get_size()
    except OSError:
        return None
    current = (size.lines, size.columns)
    return None if current == last else current


def translate_keystroke(ch: str, get_next: Callable[[], str]) -> str:
    """Map one ``msvcrt.getwch()`` character to the bytes to forward to the child.

    This is the *classic* Win32 console-input model (which is why the console
    stdin must NOT be put in ENABLE_VIRTUAL_TERMINAL_INPUT mode -- see
    ``_console_vt_plan``): a 0x00 / 0xe0 lead byte introduces a two-byte extended
    key whose scan-code second byte we translate to a VT sequence via
    ``get_next``. Everything else -- printables, ``\\r`` (Enter), ``\\x03``
    (Ctrl+C, which msvcrt surfaces as raw input) -- forwards verbatim.
    """
    if ch in ("\x00", "\xe0"):
        return _SPECIAL_KEYS.get(get_next(), "")
    return ch


# Bracketed-paste markers. Claude Code (like any modern TUI) enables bracketed
# paste, so wrapping pasted text in these tells it "this is one paste" -- newlines
# are inserted literally instead of each one firing an Enter/submit.
BRACKETED_PASTE_START = "\x1b[200~"
BRACKETED_PASTE_END = "\x1b[201~"
# How long to keep draining after a newline-bearing burst, to reassemble a paste
# that the console delivered in several back-to-back chunks. Short enough that a
# separately-typed keystroke (>50ms human gap) never merges into it.
_PASTE_COALESCE_SECONDS = 0.01


def is_paste_burst(s: str) -> bool:
    """True if a drained input burst is a multi-line *paste*.

    The distinguishing mark of a paste (vs. typing or a lone trailing Enter) is a
    newline with at least one more character after it -- a human cannot type
    Enter-then-more-text inside a single console drain, but a paste delivers it
    atomically. A bare ``"\\r"`` (typed Enter) or a newline-free burst is not a
    paste.
    """
    for i, c in enumerate(s):
        if c in ("\r", "\n") and i != len(s) - 1:
            return True
    return False


def wrap_bracketed_paste(s: str) -> str:
    """Wrap ``s`` so the child treats it as one paste (newlines preserved, no
    per-line submit) instead of a sequence of typed Enters."""
    return BRACKETED_PASTE_START + s + BRACKETED_PASTE_END


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


# ctypes.wintypes raises at import time on non-Windows systems, so it -- and the
# Win32 console-record structures built from it -- are only defined when actually
# on Windows. This preserves the property (see the module docstring) that the pure
# functions in this file stay importable/testable without the native dependency
# present; INPUT_RECORD and friends simply don't exist off-Windows, same as
# PtyProcess itself not existing until start()'s lazy pywinpty import.
if os.name == "nt":
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


# Windows console std-handle ids (GetStdHandle).
STDOUT_HANDLE = -11
STDIN_HANDLE = -10
ENABLE_PROCESSED_INPUT = 0x0001
ENABLE_LINE_INPUT = 0x0002
ENABLE_ECHO_INPUT = 0x0004  # input-mode bit; numerically coincides with
                            # ENABLE_VIRTUAL_TERMINAL_PROCESSING below (an
                            # unrelated *output*-mode bit on a different
                            # handle) -- not a conflict, just a coincidence of
                            # Win32's separate input/output bit namespaces.
ENABLE_VIRTUAL_TERMINAL_PROCESSING = 0x0004
ENABLE_VIRTUAL_TERMINAL_INPUT = 0x0200
ENABLE_MOUSE_INPUT = 0x0010
ENABLE_QUICK_EDIT_MODE = 0x0040
ENABLE_EXTENDED_FLAGS = 0x0080

# WaitForSingleObject return value meaning "the handle is signaled" (wait
# succeeded because input is ready, as opposed to WAIT_TIMEOUT).
WAIT_OBJECT_0 = 0x00000000

# How long PtySession._read_vt_input waits on stdin per poll before giving the
# relay loop a chance to recheck self._stop -- the vt-backend equivalent of
# the legacy loops' self._stop.wait(0.02)/_RESIZE_POLL_SECONDS poll cadence.
_VT_RELAY_WAIT_MS = 50
_VT_RELAY_BUFFER_CHARS = 4096

# Bound on how long PtySession.stop() waits for the vt relay thread to notice
# self._stop and return, before restoring the console mode anyway (Task 3's
# race mitigation -- see stop()'s own docstring for the full reasoning). A
# generous multiple of _VT_RELAY_WAIT_MS: the real reader rechecks self._stop
# roughly every 50ms, so this join succeeds well within one second in the
# overwhelming common case.
_VT_RELAY_STOP_JOIN_SECONDS = 0.5


def resolve_saved_mode(existing: "Optional[int]", candidate: int) -> int:
    """Decide what to persist as the "saved original console mode" (Task 3.1).

    Save-once semantics: if nothing has been saved yet (``existing`` is
    ``None``), persist ``candidate`` -- the mode read immediately before the
    vt backend's mutation. If something is already saved (e.g. a second
    ``start()`` call on the same PtySession without an intervening restore),
    keep the existing value instead of overwriting it with ``candidate``,
    which by that point would itself already be a *mutated* mode -- clobbering
    the true original and permanently losing what needs to be restored later.
    """
    return existing if existing is not None else candidate


def diff_console_mode_bits(before: int, after: int) -> "dict[str, int]":
    """Return which console-mode bits changed between two raw mode integers
    (Task 3.2), purely by bitwise comparison -- no console handle involved.

    ``{"gained": bits set in after but not before, "lost": bits set in before
    but not after}``. Exists so the vt backend's mutation contract (stdin
    gains ENABLE_VIRTUAL_TERMINAL_INPUT; loses ENABLE_PROCESSED_INPUT,
    ENABLE_LINE_INPUT, ENABLE_ECHO_INPUT; touches nothing else -- see
    _enable_vt_console's docstring) is stated and tested independently of ever
    touching a real handle: if a future edit changes what gets cleared, the
    test built on this function fails immediately instead of only surfacing
    as a live-console regression.
    """
    return {"gained": after & ~before, "lost": before & ~after}


def _console_vt_plan(backend: str = "legacy") -> "list[tuple[int, int]]":
    """(handle_id, mode_flag) pairs to OR into the console mode for ``backend``.

    Output is always enabled, on both backends.

    Under the "legacy" backend, deliberately does not enable
    ENABLE_VIRTUAL_TERMINAL_INPUT on stdin: that makes keys arrive as VT escape
    sequences, which the ``msvcrt.getwch()``-based ``_input_loop`` (classic
    0x00/0xe0 model, see ``translate_keystroke``) cannot parse -- it corrupts
    Enter, Ctrl+C and the arrow keys while letting bare printables leak through.

    The "vt" backend (Plan Phase 2) also enables ENABLE_VIRTUAL_TERMINAL_INPUT
    on stdin, so Windows Terminal delivers keys/mouse/Shift+Tab/bracketed-paste
    as native VT escape sequences that ``PtySession._vt_relay_loop`` forwards
    straight through instead of hand-translating. ``_enable_vt_console``
    additionally clears a few conflicting input bits (ENABLE_LINE_INPUT,
    ENABLE_ECHO_INPUT, ENABLE_PROCESSED_INPUT) on stdin, specifically for the
    "vt" backend, in the same SetConsoleMode call -- see that function's
    docstring for why each is required, not just this OR-only set of bits;
    this plan format only expresses bits to set, not bits to clear.
    """
    plan = [(STDOUT_HANDLE, ENABLE_VIRTUAL_TERMINAL_PROCESSING)]
    if backend == "vt":
        plan.append((STDIN_HANDLE, ENABLE_VIRTUAL_TERMINAL_INPUT))
    return plan


def _enable_vt_console(backend: str = "legacy") -> "Optional[int]":
    """Turn on virtual-terminal console mode per ``_console_vt_plan(backend)``.

    Returns stdin's console mode exactly as read *before* any mutation in
    this call, for the "vt" backend only (``None`` for "legacy" -- nothing is
    cleared there -- and ``None`` off-Windows or if ``GetConsoleMode`` itself
    fails). This is Task 3.1's save step: the caller (``PtySession.start()``)
    is the single place that decides what to persist (via
    ``resolve_saved_mode``), so this function has exactly one job --
    mutate, and report what it overwrote.

    For the "vt" backend's stdin entry specifically, also clears three input
    bits in that same SetConsoleMode call -- these are *clears*, not *sets*,
    so they don't fit ``_console_vt_plan``'s OR-only (handle_id, flag) tuple
    format and are applied here instead. All three are verified requirements
    (Microsoft's SetConsoleMode docs), not speculative additions:

      * ENABLE_LINE_INPUT -- with this set, ReadFile/ReadConsole "returns only
        when a carriage return character is read"; a raw byte relay needs
        individual keystrokes/escape sequences the instant they're available,
        not buffered behind Enter. Disabling it means those functions instead
        "return when one or more characters are available" (docs, verbatim).
      * ENABLE_ECHO_INPUT -- documented as usable "only if ENABLE_LINE_INPUT
        is also enabled"; cleared alongside it. Also avoids the outer console
        double-echoing what Claude's own PTY already echoes back through
        _read_loop's passthrough.
      * ENABLE_PROCESSED_INPUT -- with this set, "CTRL+C is processed by the
        system and is not placed in the input buffer" at all (docs, verbatim)
        -- it would never reach the relay as a byte, and would instead raise
        this *process's own* control-handler signal. Clearing it lets \\x03
        flow through the input stream like any other byte, for the relay to
        forward to Claude.

        **Task 3.6 -- deliberate, user-facing behavior change, not a bug:**
        clearing this bit means local Ctrl+C stops killing the *supervisor*
        process the way it does today under the "legacy" backend -- under
        "vt" it becomes a literal 0x03 byte forwarded to Claude instead (the
        same as typing it inside a normal, unsupervised `claude` session).
        There is no code-level bug to fix here; this is carried forward as a
        named item in the plan's Phase 5 mandatory live-validation list, not
        something a unit test can or should wave through on its own.

    Task 3 (Phase 3) now saves the pre-mutation stdin mode (see
    resolve_saved_mode) and returns it to the caller, so PtySession.start()
    can persist it in self._saved_stdin_mode for stop()/exception-path
    restoration (PtySession._restore_stdin_mode) -- these bits are still set
    unconditionally here, since Task 2's relay needs them to function at all,
    but the mode they overwrite is no longer lost.
    """
    if os.name != "nt":
        return None
    kernel32 = ctypes.windll.kernel32
    original_stdin_mode = None
    for handle_id, flag in _console_vt_plan(backend):
        handle = kernel32.GetStdHandle(handle_id)
        mode = ctypes.c_uint32()
        if not kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
            continue
        if backend == "vt" and handle_id == STDIN_HANDLE:
            original_stdin_mode = mode.value
        new_mode = mode.value | flag
        if backend == "vt" and handle_id == STDIN_HANDLE:
            new_mode &= ~(ENABLE_LINE_INPUT | ENABLE_ECHO_INPUT | ENABLE_PROCESSED_INPUT)
        kernel32.SetConsoleMode(handle, new_mode)
    return original_stdin_mode


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


class PtySession:
    def __init__(
        self,
        argv: list[str],
        *,
        cwd: Optional[str] = None,
        env: Optional[dict] = None,
        log_buffer_lines: int = 500,
        on_exit: Optional[Callable[[int], None]] = None,
        forward_local_input: bool = True,
        input_backend: str = "legacy",
    ) -> None:
        self._argv = argv
        self._cwd = cwd
        self._env = env
        self._on_exit = on_exit
        self._forward_local_input = forward_local_input
        # "legacy" = today's msvcrt/console-API translation path; "vt" = the new
        # VT-input relay backend (see start()'s thread routing). Selected via
        # BehaviorConfig.local_input_backend, plumbed through by run.py.
        self._input_backend = input_backend

        self._proc: Optional[Any] = None  # winpty.PtyProcess once started
        self._write_lock = threading.Lock()
        self._lines: deque[str] = deque(maxlen=log_buffer_lines)
        self._cur_line = ""
        self._buf_lock = threading.Lock()
        self._stop = threading.Event()
        self._reader: Optional[threading.Thread] = None
        self._input: Optional[threading.Thread] = None
        self._resizer: Optional[threading.Thread] = None
        self._mouser: Optional[threading.Thread] = None
        self._vt_relay: Optional[threading.Thread] = None
        self._last_size: Optional[tuple[int, int]] = None
        # Task 3.1: the stdin console mode as it was *before* the vt backend's
        # mutation, so it can be written back later (see _restore_stdin_mode).
        # Stays None for the legacy backend (which never mutates stdin's mode
        # at all) and, transiently, for "vt" before start() has run.
        self._saved_stdin_mode: Optional[int] = None

    # ---- lifecycle -------------------------------------------------------

    def start(self, *, spawn: Optional[Callable[..., Any]] = None) -> None:
        """Launch Claude in a PTY and start this session's background threads.

        ``spawn`` is an injectable stand-in for ``winpty.PtyProcess.spawn``
        (mirrors this file's other injectable-callable pattern -- e.g.
        ``_vt_relay_loop``'s ``read_available``); production leaves it as
        ``None`` and gets the real pywinpty import + spawn. Tests use it to
        simulate a setup failure without needing pywinpty to actually fail
        (Task 3.4).

        Task 3.1/3.4: the vt backend's console-mode mutation (via
        ``_enable_vt_console``) and everything that can fail afterward --
        spawning the child, starting the relay/reader/resize threads -- are
        wrapped in try/except so a failure anywhere in that region still
        restores the mode (``_restore_stdin_mode``) before the exception
        propagates, instead of leaving stdin's mode mutated with the session
        never actually starting.
        """
        if spawn is None:
            try:
                from winpty import PtyProcess  # pywinpty (native, Windows only)
            except Exception as exc:
                raise RuntimeError(
                    "pywinpty is required to launch Claude (pip install pywinpty). "
                    "Import failed: %r" % (exc,)
                ) from exc
            spawn = PtyProcess.spawn
        cols, rows = 120, 30
        try:
            size = os.get_terminal_size()
            cols, rows = size.columns, size.lines
        except OSError:
            pass
        try:
            # Code-review fix: the mutation call and the save-assignment used
            # to sit BEFORE this try block, so a KeyboardInterrupt landing
            # anywhere in either -- including inside _enable_vt_console's own
            # GetConsoleMode/SetConsoleMode calls -- would either leave the
            # mode mutated with nothing recorded to restore, or leave the
            # save recorded with nothing catching the interrupt to act on it.
            # Moving both inside the guarded region closes that window
            # entirely: any exception raised by either line now propagates
            # straight into the except block below, same as a spawn/thread
            # failure already did.
            original_stdin_mode = _enable_vt_console(self._input_backend)
            if original_stdin_mode is not None:
                self._saved_stdin_mode = resolve_saved_mode(self._saved_stdin_mode, original_stdin_mode)
            self._proc = spawn(self._argv, cwd=self._cwd, env=self._env, dimensions=(rows, cols))
            self._last_size = (rows, cols)
            self._reader = threading.Thread(target=self._read_loop, name="pty-reader", daemon=True)
            self._reader.start()
            if self._forward_local_input:
                if self._input_backend == "vt":
                    self._vt_relay = threading.Thread(
                        target=self._vt_relay_loop, name="pty-vt-relay", daemon=True
                    )
                    self._vt_relay.start()
                else:
                    self._input = threading.Thread(target=self._input_loop, name="pty-input", daemon=True)
                    self._input.start()
                    _enable_mouse_console()
                    self._mouser = threading.Thread(target=self._mouse_loop, name="pty-mouse", daemon=True)
                    self._mouser.start()
            self._resizer = threading.Thread(target=self._resize_loop, name="pty-resize", daemon=True)
            self._resizer.start()
        except BaseException:
            # BaseException, not Exception: a KeyboardInterrupt landing in
            # this window (Ctrl+C during setup, before _run_until_stopped in
            # run.py is even entered -- Task 3.5) is a real possibility this
            # region must also restore on. KeyboardInterrupt/SystemExit are
            # BaseException, not Exception, so `except Exception` would have
            # silently skipped the restore for exactly the interrupt case
            # Task 3 exists to handle.
            self._restore_stdin_mode()
            raise

    @property
    def pid(self) -> int:
        return self._proc.pid if self._proc else 0

    def is_alive(self) -> bool:
        return bool(self._proc and self._proc.isalive())

    def stop(self) -> None:
        """Signal shutdown, terminate the child, and restore stdin's console
        mode (Task 3.3) for the vt backend.

        Race note (Task 3, corrected per Task 2's review): ``_vt_relay`` is a
        daemon thread never ``.join()``-ed elsewhere in this file, so
        ``self._stop.set()`` alone does not guarantee ``_vt_relay_loop``'s
        in-flight ``_read_vt_input`` call has actually returned by the time
        this method would otherwise restore the console mode -- that read
        could still be mid-flight against the mode that's about to change.
        This is a real race in the relay thread's teardown, not "stop() isn't
        interruptible" (``stop()`` itself never blocks on anything by
        default -- nothing in this file calls ``.join()`` without an explicit
        timeout, including the one added immediately below).

        Chosen mitigation: signal-then-best-effort-wait. A short, *bounded*
        join on the relay thread (``_VT_RELAY_STOP_JOIN_SECONDS``) before
        restoring -- this eliminates the race in the overwhelming common case
        (the real reader rechecks ``self._stop`` roughly every
        ``_VT_RELAY_WAIT_MS``), without turning ``stop()`` into a call that
        can hang indefinitely: the join has a timeout, and restoring anyway
        after it expires is judged better than never restoring while waiting
        on a read that, per ``_read_vt_input``'s own documented residual gap,
        is not provably bounded in the worst case. The relay thread is a
        daemon, so one still running past the timeout is harmless to leave
        behind at process exit.
        """
        self._stop.set()
        if self._proc and self._proc.isalive():
            try:
                self._proc.terminate(force=True)
            except Exception:
                pass
        if self._vt_relay is not None:
            self._vt_relay.join(timeout=_VT_RELAY_STOP_JOIN_SECONDS)
        self._restore_stdin_mode()

    # ---- injection (the reason the PTY exists) ---------------------------

    def send_keys(self, keys: str) -> None:
        with self._write_lock:
            if self._proc and self._proc.isalive():
                self._proc.write(keys)

    def send_text_line(self, text: str) -> None:
        """Inject text as if typed, followed by Enter (CR)."""
        self.send_keys(text + "\r")

    def send_ctrl_c(self) -> None:
        self.send_keys(CTRL_C)

    # ---- log buffer for !logs -------------------------------------------

    def recent_logs(self, n: int = 50) -> str:
        with self._buf_lock:
            lines = list(self._lines)
            if self._cur_line:
                lines.append(self._cur_line)
        return "\n".join(lines[-n:])

    # ---- internals -------------------------------------------------------

    def _restore_stdin_mode(self) -> None:
        """Write the console mode Task 3.1 saved (``self._saved_stdin_mode``)
        back onto stdin, exactly once.

        No-op when there's nothing to restore: the legacy backend never
        mutates stdin's mode at all (``self._saved_stdin_mode`` stays
        ``None``), and a second call after a successful restore is also a
        no-op -- the saved value is cleared the moment it's used, which is
        what makes it safe to call this from both ``stop()`` (Task 3.3,
        normal exit) and ``start()``'s own ``except`` block (Task 3.4, setup
        failure) without risking a double ``SetConsoleMode`` call or
        restoring a stale value after a later ``start()`` re-mutated it.
        """
        if self._saved_stdin_mode is None:
            return
        mode = self._saved_stdin_mode
        self._saved_stdin_mode = None
        if os.name != "nt":
            return
        kernel32 = ctypes.windll.kernel32
        handle = kernel32.GetStdHandle(STDIN_HANDLE)
        kernel32.SetConsoleMode(handle, mode)

    def _record(self, chunk: str) -> None:
        clean = strip_ansi(chunk).replace("\r", "")
        with self._buf_lock:
            parts = clean.split("\n")
            self._cur_line += parts[0]
            for extra in parts[1:]:
                self._lines.append(self._cur_line)
                self._cur_line = extra

    def _read_loop(self) -> None:
        assert self._proc is not None
        exit_code = 0
        while not self._stop.is_set():
            try:
                data = self._proc.read(4096)
            except EOFError:
                break
            except Exception:
                break
            if not data:
                if not self._proc.isalive():
                    break
                continue
            # Passthrough to the real console -- minus mouse-tracking mode
            # sequences, which are meant for Claude's own controlling terminal
            # and must not leak to the outer console (see strip_mouse_tracking).
            try:
                sys.stdout.write(strip_mouse_tracking(data))
                sys.stdout.flush()
            except Exception:
                pass
            self._record(data)
        try:
            exit_code = self._proc.exitstatus or 0
        except Exception:
            exit_code = 0
        if self._on_exit and not self._stop.is_set():
            self._on_exit(exit_code)

    def _drain_input(self, getwch: Callable[[], str], kbhit: Callable[[], bool]) -> str:
        """Read every character currently buffered, translating extended keys."""
        out = []
        while kbhit():
            out.append(translate_keystroke(getwch(), getwch))
        return "".join(out)

    def _input_loop(self) -> None:
        """Forward local keystrokes to the child so the user can still type here.

        Reads are coalesced into bursts. A multi-line *paste* -- which the console
        delivers as one flood of characters -- is forwarded as a single unit
        wrapped in bracketed-paste markers, instead of letting each embedded
        newline fire a separate Enter/submit. A typed Enter still submits, because
        it arrives alone in its own burst (see ``is_paste_burst``).
        """
        try:
            import msvcrt
        except ImportError:
            return
        while not self._stop.is_set():
            if not msvcrt.kbhit():
                self._stop.wait(0.02)
                continue
            burst = self._drain_input(msvcrt.getwch, msvcrt.kbhit)
            # A paste can arrive in several back-to-back OS chunks; briefly keep
            # draining so a chunk boundary can't split it and mis-fire a submit.
            if "\r" in burst or "\n" in burst:
                while not self._stop.is_set():
                    self._stop.wait(_PASTE_COALESCE_SECONDS)
                    more = self._drain_input(msvcrt.getwch, msvcrt.kbhit)
                    if not more:
                        break
                    burst += more
            if not burst:
                continue
            if is_paste_burst(burst):
                self.send_keys(wrap_bracketed_paste(burst))
            else:
                self.send_keys(burst)

    def _mouse_loop(self) -> None:
        """Forward local mouse-wheel events to the child as SGR mouse sequences.

        Peeks the console input queue rather than blind-reading it: a key event left
        in place by classify_record's "not_mouse" branch must still be there for
        _input_loop's msvcrt calls to pick up. Only mouse-typed records are ever
        actually dequeued here -- wheel events are translated and forwarded, other
        mouse events (click/drag/move/horizontal-wheel) are dequeued and dropped.

        Known residual risk: msvcrt.getwch()'s C-runtime implementation also reads
        from this same Win32 console input queue internally, so peek-then-
        conditionally-read prevents this loop from stealing key events, but can't
        guarantee the CRT's own internal queue scan never interacts with a mouse
        record sitting ahead of a key event. Fully eliminating that would mean a
        single unified ReadConsoleInputW-based reader for both keyboard and mouse --
        out of scope for this pass (see the design spec's Ctrl+Tab deferral).
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

    def _read_vt_input(self) -> str:
        """Block on stdin's console handle for up to _VT_RELAY_WAIT_MS, then
        read whatever VT-translated characters are currently available.

        This is the "real" read _vt_relay_loop uses in production (tests
        inject a fake standing in for it -- see test_pty_vt_relay.py).

        Design choice (Task 2's interruptible-blocking-read question):
        WaitForSingleObject on a console *input* handle is documented to
        signal exactly when the input buffer holds unread records, and reset
        to non-signaled once it's drained -- so waiting on it with a short
        timeout is the real "pollable, interruptible blocking read" primitive
        for a console handle, the same role self._stop.wait(...) plays
        elsewhere in this file (_resize_loop, legacy _input_loop). An
        unconditional blocking ReadConsoleW/ReadFile call, with no timeout
        parameter at all, would have no way to unblock on stop() short of a
        much bigger lift (overlapped I/O + CancelSynchronousIo) -- rejected
        for that reason.

        Residual gap, flagged rather than silently shipped: once
        WaitForSingleObject reports the handle signaled, the follow-up
        ReadConsoleW call can still internally keep waiting a little longer
        if the record that triggered the signal turned out to be a bare
        modifier keypress (or similar) that produces no character output --
        ReadConsole (per Microsoft's docs) only returns once "one or more
        characters are available", not merely once *a* record exists. With
        ENABLE_LINE_INPUT off (see _enable_vt_console) this is bounded to
        "until the very next real keystroke", not a full line, so in practice
        it's a sub-second edge case, not a real hang -- but it means this
        read is "interruptible within one poll window in the common case",
        not unconditionally so in a provable worst case.
        """
        if os.name != "nt":
            return ""
        kernel32 = ctypes.windll.kernel32
        handle = kernel32.GetStdHandle(STDIN_HANDLE)
        if kernel32.WaitForSingleObject(handle, _VT_RELAY_WAIT_MS) != WAIT_OBJECT_0:
            return ""
        buffer = ctypes.create_unicode_buffer(_VT_RELAY_BUFFER_CHARS)
        chars_read = wintypes.DWORD()
        if not kernel32.ReadConsoleW(
            handle, buffer, _VT_RELAY_BUFFER_CHARS, ctypes.byref(chars_read), None
        ):
            return ""
        if not chars_read.value:
            return ""
        return ctypes.wstring_at(buffer, chars_read.value)

    def _vt_relay_loop(self, read_available: Optional[Callable[[], str]] = None) -> None:
        """Transparent VT-input relay for the "vt" backend (Plan Phase 2).

        With ENABLE_VIRTUAL_TERMINAL_INPUT on stdin (see _console_vt_plan /
        _enable_vt_console), Windows Terminal already delivers keyboard,
        mouse, Shift+Tab and bracketed-paste as one plain VT byte/escape-
        sequence stream -- there is nothing left here to interpret. Every
        read is forwarded to the child unchanged via send_keys(), the exact
        same write path (and _write_lock) Discord injection already uses --
        no new write path, no per-byte/per-sequence branching of any kind.

        ``read_available`` is an injectable stand-in for the real blocking
        read (mirrors how _drain_input takes injectable getwch/kbhit for the
        legacy backend); production callers (start()) leave it as None and
        get _read_vt_input -- see that method's docstring for the bounded-
        wait design and its one documented residual gap.
        """
        read = read_available if read_available is not None else self._read_vt_input
        while not self._stop.is_set():
            data = read()
            if data:
                self.send_keys(data)

    def _resize_loop(self) -> None:
        """Forward the local console's size into the inner ConPTY on change.

        Windows has no SIGWINCH, so this polls. Without it the inner ConPTY stays
        pinned at the size measured once at spawn, forever -- Claude never learns
        the real terminal was resized, so it can't reflow, and its own
        cursor-redraw math (computed against a stale size) can visibly corrupt
        already-printed transcript lines.
        """
        while not self._stop.is_set():
            if self._stop.wait(_RESIZE_POLL_SECONDS):
                break
            new_size = poll_resize(self._last_size, os.get_terminal_size)
            if new_size is None:
                continue
            self._last_size = new_size
            if self._proc and self._proc.isalive():
                try:
                    self._proc.setwinsize(*new_size)
                except Exception:
                    pass
