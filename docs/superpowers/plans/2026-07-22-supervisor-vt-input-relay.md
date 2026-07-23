# Supervisor VT Input Relay — TDD Implementation Plan

> Companion documents: `docs/superpowers/specs/2026-07-22-supervisor-mouse-wheel-forwarding-design.md` (original, now superseded for input), `docs/superpowers/specs/2026-07-22-supervisor-vt-input-migration-design.md` (design), and this session's architecture investigation (chat, cites microsoft/terminal#15296, #18094, #4949, and the official ConPTY samples).

**Goal:** replace the Win32 console-event translation layer in `pty_session.py` with a transparent VT byte relay, without changing PTY ownership, Discord injection, hooks, the state machine, snapshots, or IPC.

**Execution-order note:** the phases below are numbered to match the requested structure, but the real gating order is **0 → 1 → 2 → 3 → 5 (live validation) → 4 (deletion)**. Phase 4's own header text ("After VT backend passes validation") already says this — flagging it explicitly because this feature has twice shipped green-tests-but-failing-live-check, and deleting the legacy fallback (Phase 4) before Phase 5's live checks pass would remove the safety net at exactly the moment it's most needed.

**No code or pseudocode in this document** — steps below name the test, state what it asserts, and state what production change satisfies it, in prose.

**Guiding principle / success criterion:** the supervisor must get thinner, not smarter. For every input-path behavior, the question is always **"can Windows Terminal already do this natively?"** — if yes, the supervisor's only job is to relay the bytes: no interpreting them, no rewriting them, no synthesizing them. This plan is judged not just on "does mouse/Shift+Tab/paste work" but on "did we delete more custom logic than we added." Applied to the phases below:
- Phase 2's byte relay is the principle's positive case: mouse, Shift+Tab, arrows, and plain typing all become pure passthrough because WT already does the translation natively once VT input mode is on — nothing here should grow new branching.
- Phase 3's mode save/restore and the `ENABLE_PROCESSED_INPUT` clear are the principle's necessary exception: WT has no "natively correct" opinion on whether Ctrl+C should signal the supervisor or forward to Claude — that's a real product decision this process has to make, not a translation to avoid.
- The conditional retention of `is_paste_burst`/`wrap_bracketed_paste` (Phase 0/4) is exactly this question applied to paste: keep them **only if** Phase 5's live check proves WT's native bracketed-paste does *not* already arrive pre-wrapped. Default assumption going in is that it does, and the code should come out — not stay "just in case."
- Phase 4's deletion list should be read as the primary deliverable, not cleanup — a plan that adds a vt backend without shrinking net code volume has not satisfied this criterion, regardless of whether the live checks pass.

---

## Phase 0 — Baseline

**Functions to be removed (end state, after Phase 4 only):**
`translate_keystroke`, `_SPECIAL_KEYS`, `classify_record`, `extract_wheel_delta`, `translate_wheel_event`, `_enable_mouse_console`, `_mouse_loop`, `strip_mouse_tracking`, `_MOUSE_MODE_RE`, `_debug_log`, `_MOUSE_DEBUG_LOG`, the ctypes structures `_COORD`/`MOUSE_EVENT_RECORD`/`_CHAR_UNION`/`KEY_EVENT_RECORD`/`INPUT_RECORD_EVENT`/`INPUT_RECORD`, and constants `KEY_EVENT_TYPE`, `MOUSE_EVENT_TYPE`, `MOUSE_WHEELED`, `MOUSE_HWHEELED`, `_WHEEL_DELTA`, `_SGR_WHEEL_UP`, `_SGR_WHEEL_DOWN`, `ENABLE_MOUSE_INPUT`, `ENABLE_QUICK_EDIT_MODE`.
Conditionally removed (decision made in Phase 5, not before): `_drain_input`, `is_paste_burst`, `wrap_bracketed_paste`, `_PASTE_COALESCE_SECONDS`, `BRACKETED_PASTE_START`, `BRACKETED_PASTE_END` — kept only if the live paste check shows Windows Terminal's native bracketed-paste does *not* arrive pre-wrapped under the vt backend.

**Functions that remain, unchanged:**
`strip_ansi`, `poll_resize`, `_resize_loop`, `send_keys`, `send_text_line`, `send_ctrl_c`, `recent_logs`, `_record`, `start`/`stop`/`pid`/`is_alive` lifecycle methods, the `PtySession` class shape.

**Functions that remain but are extended:**
`_enable_vt_console` (gains the stdin branch), `_console_vt_plan` (gains a backend-aware stdin entry).

**Callers affected:**
- `PtySession.start()` — must branch on the new backend selection when launching local-input threads; currently unconditionally starts `_input`/`_mouser`.
- `run.py` — constructs `PtySession(...)`; must read and pass through the new backend selection from config, mirroring how it already passes `forward_local_input` implicitly via the default.
- `config.py` — `BehaviorConfig` gains one new field; `_validate()` gains one new check. Nothing in `DiscordConfig`, `TimersConfig`, or `SupervisorConfig` changes.
- `core.py`, `bot.py` — **not touched.** Both only call `pty.send_keys` / `send_text_line` / `send_ctrl_c` / `recent_logs`, none of whose signatures change. This is the concrete basis for the "Discord injection exactly as it works today" constraint — it's true by construction, not by inspection, because nothing in this plan edits those two files.

**Existing tests requiring updates:**
- `tests/test_pty_input.py` — the single most important test here is `test_console_plan_never_enables_vt_input_on_stdin`. Today it asserts the console plan **never** enables VT input on stdin, under any circumstance — that assertion becomes categorically false once a vt backend legitimately does exactly that. It must be reframed in Phase 1 to assert the *legacy* backend's plan never touches stdin/VT-input, while a new counterpart test asserts the *vt* backend's plan does. Getting this reframing wrong (e.g. deleting the guard instead of splitting it) would silently remove the regression protection that caught the original bug this file documents.
- `tests/test_pty_mouse_strip.py` — every test in this file targets `strip_mouse_tracking`, which Phase 4 deletes. The whole file is retired, not edited.
- `tests/test_pty_mouse_wheel.py` — every test targets `extract_wheel_delta`/`translate_wheel_event`/`classify_record`/the `INPUT_RECORD` structures, all deleted in Phase 4. The whole file is retired, not edited.
- `tests/test_pty_resize.py`, `tests/test_core.py`, `tests/test_bot_mentions.py`, `tests/test_snapshot.py` — no edits expected; they exist as regression gates that must stay green through every phase below, and a red result in any of them at any phase is a stop signal, not something to patch around.

---

## Phase 1 — Introduce a VT backend

Add the selection mechanism and thread-routing only. The vt path exists and is reachable, but Phase 2 fills in its actual relay behavior — Phase 1's own deliverable is "the flag exists, is validated, and correctly routes," independently verifiable before any relay logic exists.

- [ ] **Task 1.1 — Config field.** Add `local_input_backend: str = "legacy"` to `BehaviorConfig`. Test: a config-loading test asserts a `config.yaml` with no `local_input_backend` key defaults to `"legacy"`, and one with `local_input_backend: vt` loads as `"vt"`.
- [ ] **Task 1.2 — Config validation.** Extend `_validate()` to reject any value other than `"legacy"`/`"vt"`, in the same style as the existing `discord.channel_id` check. Test: a config with `local_input_backend: bogus` raises the same `ValueError` family the other invalid-config tests already assert against.
- [ ] **Task 1.3 — Constructor plumbing.** `PtySession.__init__` gains `input_backend: str = "legacy"`, stored alongside the existing `forward_local_input`. `run.py` passes `cfg.behavior.local_input_backend` through. Test: constructing a `PtySession` with no `input_backend` argument still behaves exactly as every existing `test_pty_*` construction call does today (no existing test's expectations change).
- [ ] **Task 1.4 — Thread routing.** `start()` branches once on `self._input_backend`: `"legacy"` launches today's `_input`/`_mouser` threads unchanged; `"vt"` launches a placeholder relay thread that starts, waits on the stop event, and exits cleanly, doing no real relay yet. Test: with `forward_local_input=True` and `input_backend="vt"`, starting and immediately stopping a `PtySession` against a trivial spawned process leaves no thread alive and raises nothing — proves the new path is wired into the same lifecycle (`stop()`, `is_alive()`) without asserting anything about relay behavior yet.
- [ ] **Task 1.5 — Split the stdin regression guard.** Reframe `test_console_plan_never_enables_vt_input_on_stdin` into two: one asserting the *legacy* backend's console plan still never touches stdin/VT-input (protects the original bug fix), one asserting the *vt* backend's plan does touch stdin with the VT-input bit (drives Phase 2). The vt-side assertion is expected to fail until Phase 2 — that failure is the correct TDD red state to carry into Phase 2, not a Phase 1 bug.

---

## Phase 2 — Raw input relay

Design constraint, restated as a hard rule for every task in this phase: **no event translation, no key translation, no mouse translation — only relay bytes.** Anything that looks like per-key or per-record branching in this phase is a scope violation, not a feature.

- [ ] **Task 2.1 — Enable VT input on stdin (vt backend only).** Extend the vt backend's console-mode plan to include stdin with `ENABLE_VIRTUAL_TERMINAL_INPUT` set. Test: the vt-side assertion from Task 1.5 now passes. A second test asserts the vt backend's plan still includes stdout's `ENABLE_VIRTUAL_TERMINAL_PROCESSING` unchanged — a regression guard against accidentally dropping output VT processing while adding the input bit, since both live in the same extended plan function.
- [ ] **Task 2.2 — Raw byte read.** Replace the placeholder vt relay thread from Task 1.4 with one that performs a blocking read of whatever bytes are currently available from the real stdin stream, in one thread, with no `msvcrt` involvement at all. Test: given a fake byte source (not a real console — the same style of injectable dependency `_drain_input` already uses for `getwch`/`kbhit` in the legacy tests) that yields a known byte sequence including an escape sequence unknown to any of today's special-case tables (e.g. an arbitrary unmapped CSI sequence), the relay forwards it byte-for-byte, unchanged — proving the "only relay bytes" rule behaviorally, not just by code inspection.
- [ ] **Task 2.3 — Forward into the existing write path.** The vt relay calls `send_keys()` directly — the same method Discord injection already calls, sharing its existing `_write_lock`. No new write path is introduced. Test: a test constructs a `PtySession`, drives the vt relay's byte source and a simulated `inject_text`-style call concurrently, and asserts both land in the recorded writes without interleaving corruption — proving the two producers (relay thread, Discord coroutine) safely share one consumer, which is the concrete meaning of "preserve Discord injection behavior" at the code level.
- [ ] **Task 2.4 — Regression gate.** Re-run `test_core.py` and `test_bot_mentions.py` unmodified. Both must stay green — they exercise `core.py`/`bot.py`'s injection call sites directly and neither should notice this phase happened. This is the automated half of "preserve Discord injection behavior"; the live half (`!reply` actually reaching Claude under the vt backend) is a Phase 5 obligation, not assumed safe by symmetry with the legacy backend.

---

## Phase 3 — Console mode lifecycle

- [ ] **Task 3.1 — Save original mode before mutation.** Before the vt backend changes anything, read and store stdin's current console mode. Test: a pure function taking "no prior save" vs. "already saved" state returns the correct value to persist, without touching a real console handle (mirrors the existing pattern of keeping ctypes-touching code thin and the decision logic pure and testable, as `poll_resize` already does for resize).
- [ ] **Task 3.2 — Mode-bit diff, documented and testable.** Add a pure function that, given a "before" and "after" mode integer, returns which bits changed. Test: feeding the exact before/after values this phase intends to use returns exactly: stdin gains `ENABLE_VIRTUAL_TERMINAL_INPUT`; stdin loses `ENABLE_PROCESSED_INPUT` (required for Ctrl+C to arrive as a literal `\x03` byte instead of an OS-level signal — see Ctrl+C risk below); stdin's QuickEdit/`ENABLE_MOUSE_INPUT` bits are **not present in this diff at all** — the vt backend never touches them, unlike the legacy backend's `_enable_mouse_console`. A test asserts this diff contains no `ENABLE_QUICK_EDIT_MODE`/`ENABLE_MOUSE_INPUT` entries, as an explicit boundary between the two backends' mode footprints.
- [ ] **Task 3.3 — Restore on normal exit.** Wrap the vt backend's mode mutation and relay lifecycle so the saved mode from Task 3.1 is written back when `stop()` completes normally. Test: start and stop a `PtySession` on the vt backend against a trivial process, assert the mode-restore call fires exactly once with the value saved in Task 3.1.
- [ ] **Task 3.4 — Restore on exception.** Wrap the same region in a construct that guarantees restoration even if the relay thread's setup raises. Test: inject a failure into the mode-mutation step and assert restoration still fires before the exception propagates.
- [ ] **Task 3.5 — Restore on process teardown, with a documented gap.** Register the same restoration in `run.py`'s shutdown path (in addition to, not instead of, Task 3.3/3.4) so a `KeyboardInterrupt`-driven exit still restores the mode. Explicitly document — in the code and in this plan — that a hard external termination (task-kill, power loss) cannot run any cleanup at all on Windows; this is an accepted, unavoidable gap, not something Phase 3 can close. No test can cover this case; document it instead of implying coverage that doesn't exist.
- [ ] **Task 3.6 — Ctrl+C behavior change, explicit sign-off.** Because `ENABLE_PROCESSED_INPUT` is cleared under the vt backend, local Ctrl+C stops killing the supervisor process the way it does today under the legacy backend — it becomes a literal byte forwarded to Claude instead. This is a deliberate, user-facing behavior change, not a bug. No unit test can validate the *felt* behavior of this; it is carried forward as a named item in Phase 5's mandatory live validation list, and must not be waved through on unit-test coverage alone.

---

## Phase 4 — Remove translation layer

**Gate: do not start this phase until every Phase 5 live check has passed on the vt backend.** Deletion converts rollback from "change one config value" to "git revert" (see Rollback below) — sequence accordingly.

Delete, in `claude_supervisor/pty_session.py`:
- `translate_keystroke`, `_SPECIAL_KEYS` — superseded by Phase 2's raw byte read.
- `classify_record`, `extract_wheel_delta`, `translate_wheel_event` — superseded by Phase 2; mouse is no longer a distinct code path from keyboard.
- `_enable_mouse_console`, `_mouse_loop` (and the `self._mouser` thread attribute) — superseded by Phase 2/3.
- `strip_mouse_tracking`, `_MOUSE_MODE_RE` — no longer needed; `_read_loop` stops calling `strip_mouse_tracking` and passes Claude's output through unmodified, letting mouse-mode DECSET reach Windows Terminal as intended.
- `_debug_log`, `_MOUSE_DEBUG_LOG` — diagnostic-only, already flagged temporary in the codebase's own comments.
- Ctypes structures: `_COORD`, `MOUSE_EVENT_RECORD`, `_CHAR_UNION`, `KEY_EVENT_RECORD`, `INPUT_RECORD_EVENT`, `INPUT_RECORD`.
- Constants: `KEY_EVENT_TYPE`, `MOUSE_EVENT_TYPE`, `MOUSE_WHEELED`, `MOUSE_HWHEELED`, `_WHEEL_DELTA`, `_SGR_WHEEL_UP`, `_SGR_WHEEL_DOWN`, `ENABLE_MOUSE_INPUT`, `ENABLE_QUICK_EDIT_MODE`.
- Conditionally: `_drain_input`, `is_paste_burst`, `wrap_bracketed_paste`, `_PASTE_COALESCE_SECONDS`, `BRACKETED_PASTE_START`, `BRACKETED_PASTE_END` — only if Phase 5's paste live-check showed these are no longer needed (§ Phase 0).
- The now-dead `"legacy"` branch in `start()`'s thread routing, `_enable_mouse_console` call site, and the `input_backend` config option collapses to vt-only — or, if the mentor wants the legacy backend kept indefinitely as a documented fallback rather than deleted, this last bullet is skipped and the config default flips from `"legacy"` to `"vt"` instead. **This is a decision to make explicitly at Phase 4 kickoff, not a default to assume.**

- [ ] **Task 4.x — Thinner-not-smarter audit.** Before closing this phase, walk every function that survives in `pty_session.py` and ask the guiding-principle question of each: is this relaying bytes WT already handles natively, or is it interpreting/rewriting/synthesizing something? Anything in the latter category (expected survivors: mode save/restore, the resize poll loop, the conditional paste safety net if Phase 5 proved it necessary, `send_keys`/injection) must have a one-line justification for why WT has no native opinion here. Anything without a justification is a sign Phase 2/3 accidentally reintroduced translation and should be cut, not documented. Report the net line-count delta (deleted vs. added) in this file's Phase 4 section as the concrete artifact of "thinner, not smarter" — a net increase is a failed exit criterion even if every live check in Phase 5 passed.

Delete, in `tools/claude-supervisor/tests/`:
- `test_pty_mouse_strip.py` — entire file.
- `test_pty_mouse_wheel.py` — entire file.

Edit, not delete:
- `test_pty_input.py` — remove the now-dead `translate_keystroke`/`_SPECIAL_KEYS`-targeting tests and the now-single (legacy-only, or removed entirely per the decision above) half of the Task 1.5 split; keep whatever paste tests Phase 5 determined are still needed.

**Executed 2026-07-23, mentor sign-off on both decision points explicit (not defaulted):** Ctrl+C (Task 3.6) confirmed tested live and acceptable; legacy backend deleted entirely (not kept as fallback) — `local_input_backend` is gone from `BehaviorConfig`/`_validate()`/`PtySession.__init__`/`run.py`, `_console_vt_plan()`/`_enable_vt_console()` lost their now-constant backend parameter, `start()` unconditionally launches the vt relay thread. Paste's conditional deletion (§ Phase 0) resolved to "delete" — the live check confirmed multi-line/bracketed paste already works correctly through the relay's plain passthrough, and `_vt_relay_loop` never called `is_paste_burst`/`wrap_bracketed_paste` in the first place, so they were dead even before removal. `test_pty_mouse_strip.py`/`test_pty_mouse_wheel.py` retired whole; `test_config.py` also retired whole (it existed solely to cover `local_input_backend`, added in Task 1.1/1.2 — confirmed via `git log --follow` before deleting). `test_pty_input.py`/`test_pty_vt_backend.py`/`test_pty_console_restore.py` trimmed to drop every legacy-vs-vt comparison test.

**Task 4.x thinner-not-smarter audit result:** `pty_session.py` 848 → 539 lines (-309, -36%). Repo-wide (Phase 4 commit alone): +175/-1013 lines across 11 files, net **-838 lines**. A net decrease, satisfying the exit criterion. Every surviving function was walked: mode save/restore (`resolve_saved_mode`/`diff_console_mode_bits`/`_restore_stdin_mode`), the resize poll loop, and `send_keys`/injection all still have a real "WT has no native opinion here" justification (documented in their own docstrings); nothing survived that should have been cut.

---

## Phase 5 — Validation

**Unit tests (automated, run as part of every task above, restated here as the full gate before Phase 4):**
- Keyboard input — Task 2.2/2.3's byte-relay tests.
- Multi-line paste — whichever of `is_paste_burst`/`wrap_bracketed_paste`'s existing tests remain relevant, plus a new test confirming the vt backend doesn't double-wrap an already-bracketed native paste (Phase 0's conditional-deletion risk).
- Shift+Tab — covered indirectly by Task 2.2's arbitrary-escape-sequence relay test (Shift+Tab is just another unmapped-by-us CSI sequence to this code); no dedicated Shift+Tab unit test is possible without a real terminal, same limitation the file's own header comment already states for the legacy loop.
- Resize — `test_pty_resize.py`, unmodified, must stay green (this phase never touches `_resize_loop`).
- Discord reply — `test_core.py`/`test_bot_mentions.py`, unmodified, must stay green (Task 2.4).
- Shutdown cleanup — Task 3.3/3.4's restore-on-exit/exception tests.

**Mandatory live validation — do not mark this plan complete until every item below passes, in a real Windows Terminal session, on the vt backend:**
- Typing (plain characters, arrows, Enter, Backspace)
- Mouse wheel scrolling
- Shift+Tab mode switching
- Copy (selection)
- Paste (short, single-line)
- Bracketed paste (multi-line)
- Terminal resize
- Ctrl+C behavior (confirm the Task 3.6 behavior change is acceptable as experienced, not just as documented)
- Discord `!reply`
- Discord checkpoint flow (alert → button/`!approve`/`!deny` → resumes correctly)

This list is deliberately the same shape as the mandatory live-check list already established for this feature area after two prior green-tests/failed-live-check rounds — treat a skipped item the same as a failed one.

---

## Rollback

**While Phase 4 has not run (backends coexist):** change `behavior.local_input_backend` from `vt` back to `legacy` in `config.yaml` (or delete the key, which defaults to `legacy` per Task 1.1) and restart the supervisor. No code changes, no revert, no redeploy of anything beyond the config file — this is the intended cheap rollback path for the entire time both backends exist side by side.

**After Phase 4 has run (legacy code deleted):** the cheap config rollback no longer exists by construction. Revert to the commit(s) immediately preceding Phase 4's deletion via `git revert` — Phase 4 should land as its own commit(s), separate from Phases 1–3, specifically so this revert is clean and doesn't also undo the (validated, working) vt backend itself.
