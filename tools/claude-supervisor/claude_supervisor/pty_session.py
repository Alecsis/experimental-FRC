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


# Windows console std-handle ids (GetStdHandle).
STDOUT_HANDLE = -11
STDIN_HANDLE = -10
ENABLE_VIRTUAL_TERMINAL_PROCESSING = 0x0004
ENABLE_VIRTUAL_TERMINAL_INPUT = 0x0200


def _console_vt_plan() -> "list[tuple[int, int]]":
    """(handle_id, mode_flag) pairs to OR into the console mode.

    Output ONLY. Deliberately does not enable ENABLE_VIRTUAL_TERMINAL_INPUT on
    stdin: that makes keys arrive as VT escape sequences, which the
    ``msvcrt.getwch()``-based ``_input_loop`` (classic 0x00/0xe0 model, see
    ``translate_keystroke``) cannot parse -- it corrupts Enter, Ctrl+C and the
    arrow keys while letting bare printables leak through.
    """
    return [(STDOUT_HANDLE, ENABLE_VIRTUAL_TERMINAL_PROCESSING)]


def _enable_vt_console() -> None:
    """Turn on virtual-terminal *output* processing so the proxied TUI renders."""
    if os.name != "nt":
        return
    kernel32 = ctypes.windll.kernel32
    for handle_id, flag in _console_vt_plan():
        handle = kernel32.GetStdHandle(handle_id)
        mode = ctypes.c_uint32()
        if kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
            kernel32.SetConsoleMode(handle, mode.value | flag)


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
    ) -> None:
        self._argv = argv
        self._cwd = cwd
        self._env = env
        self._on_exit = on_exit
        self._forward_local_input = forward_local_input

        self._proc: Optional[Any] = None  # winpty.PtyProcess once started
        self._write_lock = threading.Lock()
        self._lines: deque[str] = deque(maxlen=log_buffer_lines)
        self._cur_line = ""
        self._buf_lock = threading.Lock()
        self._stop = threading.Event()
        self._reader: Optional[threading.Thread] = None
        self._input: Optional[threading.Thread] = None

    # ---- lifecycle -------------------------------------------------------

    def start(self) -> None:
        try:
            from winpty import PtyProcess  # pywinpty (native, Windows only)
        except Exception as exc:
            raise RuntimeError(
                "pywinpty is required to launch Claude (pip install pywinpty). "
                "Import failed: %r" % (exc,)
            ) from exc
        cols, rows = 120, 30
        try:
            size = os.get_terminal_size()
            cols, rows = size.columns, size.lines
        except OSError:
            pass
        _enable_vt_console()
        self._proc = PtyProcess.spawn(
            self._argv, cwd=self._cwd, env=self._env, dimensions=(rows, cols)
        )
        self._reader = threading.Thread(target=self._read_loop, name="pty-reader", daemon=True)
        self._reader.start()
        if self._forward_local_input:
            self._input = threading.Thread(target=self._input_loop, name="pty-input", daemon=True)
            self._input.start()

    @property
    def pid(self) -> int:
        return self._proc.pid if self._proc else 0

    def is_alive(self) -> bool:
        return bool(self._proc and self._proc.isalive())

    def stop(self) -> None:
        self._stop.set()
        if self._proc and self._proc.isalive():
            try:
                self._proc.terminate(force=True)
            except Exception:
                pass

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
            # Passthrough to the real console.
            try:
                sys.stdout.write(data)
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
