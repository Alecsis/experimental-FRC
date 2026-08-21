# Supervisor VT-Input Migration — Design

**Goal:** Replace the hand-rolled legacy-console-API input translation in `pty_session.py` with a raw VT-input relay on the supervisor's own console, restoring native-quality mouse wheel / Shift+Tab / paste behavior while keeping the supervisor as Claude's ConPTY parent (required for `!reply`/`!approve`/button-driven Discord injection — see the architecture comparison in this session's prior turn, which ruled out moving PTY ownership to Windows Terminal).

**Non-goals:** Ctrl+Tab (owned entirely by Windows Terminal's own keybinding, unreachable from any process-side change) and click/drag mouse selection (already native-working today, untouched by this migration either way).

---

## 1. Current input pipeline

- `_enable_vt_console()` sets `ENABLE_VIRTUAL_TERMINAL_PROCESSING` on **stdout only** — stdin is deliberately excluded per `_console_vt_plan()`'s comment, because VT input is incompatible with `msvcrt`.
- `_enable_mouse_console()` toggles the **stdin** console mode: QuickEdit off, `ENABLE_MOUSE_INPUT` on (readback-verified).
- `_input_loop` (thread): `msvcrt.getwch()`/`kbhit()`, classic scan-code model. `translate_keystroke()` + `_SPECIAL_KEYS` map 0x00/0xe0-prefixed extended keys to VT sequences. `_drain_input` coalesces bursts; `is_paste_burst()`/`wrap_bracketed_paste()` hand-detect multi-line pastes and wrap them in bracketed-paste markers before forwarding via `send_keys()`.
- `_mouse_loop` (thread): `PeekConsoleInputW`/`ReadConsoleInputW` polls the same stdin console input queue for raw `INPUT_RECORD`s. `classify_record()` filters wheel vs. other-mouse vs. not-mouse. `extract_wheel_delta()` + `translate_wheel_event()` build SGR sequences, forwarded via `send_keys()`. **Confirmed this session (`mouse_debug.log`, 5,341 samples, 0 mouse events) that Windows Terminal never delivers `MOUSE_EVENT_RECORD`s into this queue at all — this whole thread is dead code in practice.**
- `_read_loop` (thread): reads Claude's ConPTY output, strips mouse-tracking DECSET/DECRST via `strip_mouse_tracking()` before echoing to the outer console, because the legacy input model above has no way to act on it.
- `_resize_loop`: unaffected by this migration, kept as-is.

## 2. New VT input pipeline

- `_enable_vt_console()` also sets `ENABLE_VIRTUAL_TERMINAL_INPUT` on **stdin** (plus `ENABLE_VIRTUAL_TERMINAL_PROCESSING` on stdout, as today). No more QuickEdit/`ENABLE_MOUSE_INPUT` toggling.
- `_read_loop` **stops stripping** mouse-tracking sequences — Claude's own DECSET/DECRST must reach Windows Terminal untouched, since that's what tells WT to start delivering VT mouse input back to this process at all.
- `_input_loop` becomes a raw-byte relay: blocking read on the real stdin handle, forward bytes to `send_keys()` essentially unchanged. Escape sequences (arrows, Shift+Tab, SGR wheel events, bracketed-paste markers) arrive pre-formed from WT and need no per-key decoding.
- `_mouse_loop` and its thread are deleted outright — mouse now flows through the same single raw-byte stdin relay as keyboard, since VT mode delivers both as one byte stream.

## 3. Functions deleted

`translate_keystroke`, `_SPECIAL_KEYS`, `classify_record`, `extract_wheel_delta`, `translate_wheel_event`, `_enable_mouse_console`, `_mouse_loop` (+ `self._mouser` thread handle), `strip_mouse_tracking` + `_MOUSE_MODE_RE`, `_debug_log`/`_MOUSE_DEBUG_LOG` (diagnostic-only, already flagged temporary), the `INPUT_RECORD`/`MOUSE_EVENT_RECORD`/`KEY_EVENT_RECORD`/`_COORD`/`_CHAR_UNION` ctypes structures, `ENABLE_MOUSE_INPUT`/`ENABLE_QUICK_EDIT_MODE` constants, `MOUSE_WHEELED`/`MOUSE_HWHEELED`/`_WHEEL_DELTA`/`_SGR_WHEEL_UP`/`_SGR_WHEEL_DOWN`/`KEY_EVENT_TYPE`/`MOUSE_EVENT_TYPE`.

`_drain_input`, `is_paste_burst`, `wrap_bracketed_paste`, `_PASTE_COALESCE_SECONDS`, `BRACKETED_PASTE_START`/`_END` — **conditionally deleted**. Delete only after Task 2's live check confirms WT's native bracketed-paste already arrives pre-wrapped; otherwise keep as a fallback (see Risk 3).

## 4. Functions that replace them

- `_enable_vt_console()` — extended in place, not replaced, to also flip stdin's `ENABLE_VIRTUAL_TERMINAL_INPUT` bit.
- `_console_vt_plan()` — extended in place: add `(STDIN_HANDLE, ENABLE_VIRTUAL_TERMINAL_INPUT)`, delete its now-inaccurate docstring about why stdin is excluded.
- New `_input_loop` (VT relay) — replaces the entire `msvcrt`-based loop. Reads via `os.read(sys.stdin.fileno(), 4096)` (Windows text-mode stdin in VT mode behaves as a normal readable stream once `ENABLE_LINE_INPUT`/`ENABLE_ECHO_INPUT` are also cleared — see Task list in the follow-up plan for the exact flag set) and calls `send_keys()` directly.
- `send_keys`/`send_text_line`/`send_ctrl_c` (Discord injection path) — **unchanged**, not part of this migration at all; they write straight to `PtyProcess.write()` regardless of how local input is captured.
- `_resize_loop`/`poll_resize()` — unchanged.

## 5. Data flow after migration

- **Keyboard:** key press → WT (VT input mode) → raw bytes on supervisor stdin → `_input_loop` reads → `send_keys()` → `PtyProcess.write()` → Claude's ConPTY input pipe → Claude parses VT input itself (same as it already does when run natively, no supervisor).
- **Mouse wheel:** Claude enables DECSET 1000/1006 on its own output → `_read_loop` passes it through unstripped → WT sees the request and starts translating OS wheel messages into SGR sequences on *this process's* stdin → same `_input_loop` relay path as keyboard → Claude.
- **Resize:** unchanged — `_resize_loop` polls `os.get_terminal_size()`, calls `PtyProcess.setwinsize()` directly; never touches stdin at all.
- **Paste:** WT's native bracketed-paste (if VT input mode surfaces it with markers already attached) flows through the same raw relay untouched; if not, the existing `is_paste_burst`/`wrap_bracketed_paste` pair stays as a safety net wrapping the raw burst before `send_keys()`.
- **Discord `!reply`/`!approve`/buttons:** entirely bypasses all of the above — `Supervisor.inject_text()`/`.approve()`/`.deny()`/`.stop_claude()` call `pty.send_keys()`/`send_text_line()`/`send_ctrl_c()` directly. Zero exposure to this migration.

## 6. Regression risks

1. **Local Ctrl+C behavior change (highest risk).** VT input mode typically requires clearing `ENABLE_PROCESSED_INPUT` for Ctrl+C to arrive as a literal `\x03` byte instead of raising a console break. Today, `run.py`'s `main()` catches `KeyboardInterrupt` to shut the supervisor down cleanly (exit 130). If `ENABLE_PROCESSED_INPUT` is cleared, local Ctrl+C stops killing the supervisor and instead forwards `\x03` to Claude — arguably *more* correct (Ctrl+C should go to the foreground app), but it removes the user's current local-exit mechanism and must be called out, tested, and decided deliberately, not discovered live.
2. **Double-wrapped paste.** If WT's VT-mode paste already arrives bracketed and the old `is_paste_burst`/`wrap_bracketed_paste` logic is left in unconditionally, markers could nest and corrupt Claude's paste parsing. Resolve via the conditional deletion in §3, verified live before deciding.
3. **Loss of extended-key special-casing.** Arrows/Home/End/PageUp/PageDown/Delete currently go through `_SPECIAL_KEYS`; in VT mode these should arrive as native sequences requiring no translation, but this is a full swap of the mechanism, not an additive change — must be individually re-verified, not assumed.
4. **Discord path is a false-safety blind spot.** Because `send_keys()` is untouched, it's tempting to assume Discord injection needs no re-testing — but the *supervisor's own* console mode changes are process-global, so a full round-trip re-test (Discord `!reply` while local input is idle) is still required to rule out any Windows-side interaction.
5. **`forward_local_input=False` path.** Tests already construct `PtySession` with local input disabled; the new `_enable_vt_console()` change must stay correctly gated behind the same flag so headless/test construction doesn't touch console mode at all (mirrors today's `_enable_mouse_console()` gating).

## 7. Test plan

- **Retire, don't "fix," the tests targeting deleted code:** `tests/test_pty_mouse_wheel.py` (18 tests) and `tests/test_pty_mouse_strip.py` (6 tests) delete along with the functions they test — their existence after this migration would itself be a signal something wasn't actually removed.
- **New unit coverage** is necessarily thin — a raw byte relay has little pure logic to unit-test. Cover what remains pure: the extended `_console_vt_plan()` mode-flag composition (a pure function returning `(handle, flag)` pairs, already the pattern used today) and, if kept, `is_paste_burst`/`wrap_bracketed_paste` (already covered by existing `test_pty_input.py`, keep those cases).
- **Full existing suite must stay green** everywhere not targeting deleted code: `test_pty_input.py`, `test_pty_resize.py`, `test_core.py`, `test_snapshot.py`, `test_bot_mentions.py`.
- **Mandatory live-check protocol** (this feature has already failed two live checks after passing unit tests — do not claim done on green tests alone): mouse wheel scroll, Shift+Tab, Ctrl+Tab (expected still broken — confirms it's WT-level, not a regression), right-click paste, Ctrl+Shift+C copy-of-selection, multi-line paste, **local Ctrl+C exit behavior**, and a Discord `!reply` round-trip while the local session sits idle.

## 8. Rollback strategy

- Land the migration as a small number of atomic commits (mode-flag change / input-loop rewrite / mouse-loop deletion / read-loop strip removal, roughly one per §3–4 bullet group) so a partial revert is possible if, say, wheel works but Ctrl+C exit doesn't.
- Keep the existing `forward_local_input` constructor flag as the kill switch — a config-level toggle between "old input loop" and "new VT relay" is cheap to add on top of it (branch inside `start()` on a new `Config` field) and lets a failed live check be undone without a code revert, which matters given this exact feature's two prior live-check failures.
- If a live check fails and the cause isn't quickly isolated, `git revert` the input-loop commit(s) specifically — `_resize_loop`/`send_keys`/Discord injection are on separate commits and untouched, so a revert here can't regress those.
