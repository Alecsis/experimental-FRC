"""Regression tests for config loading (claude_supervisor/config.py).

Covers the `local_input_backend` behavior switch added for the VT-input-relay
migration (Plan Phase 1): defaults to "legacy" when absent, loads "vt" when
set, and is rejected by `_validate` when set to anything else.

Run: python tests/test_config.py
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from claude_supervisor.config import load_config

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


# Minimal valid config.yaml body -- just enough to pass `_validate`.
_BASE_YAML = """
discord:
  bot_token: "real-token"
  authorized_user_id: "12345"
  channel_id: 999
"""


def _write_config(tmp_path: Path, extra_yaml: str = "") -> Path:
    p = tmp_path / "config.yaml"
    p.write_text(_BASE_YAML + extra_yaml, encoding="utf-8")
    return p


def test_local_input_backend_defaults_to_legacy():
    print("test_local_input_backend_defaults_to_legacy")
    with tempfile.TemporaryDirectory() as td:
        p = _write_config(Path(td))
        cfg = load_config(p)
        check(cfg.behavior.local_input_backend == "legacy",
              "no local_input_backend key -> defaults to 'legacy'")


def test_local_input_backend_vt_loads():
    print("test_local_input_backend_vt_loads")
    with tempfile.TemporaryDirectory() as td:
        p = _write_config(Path(td), "behavior:\n  local_input_backend: vt\n")
        cfg = load_config(p)
        check(cfg.behavior.local_input_backend == "vt",
              "local_input_backend: vt -> loads as 'vt'")


def test_bogus_local_input_backend_rejected():
    print("test_bogus_local_input_backend_rejected")
    with tempfile.TemporaryDirectory() as td:
        p = _write_config(Path(td), "behavior:\n  local_input_backend: bogus\n")
        try:
            load_config(p)
            check(False, "load_config raises ValueError for bogus local_input_backend")
        except ValueError as exc:
            check("local_input_backend" in str(exc),
                  "ValueError mentions local_input_backend")


def main():
    test_local_input_backend_defaults_to_legacy()
    test_local_input_backend_vt_loads()
    test_bogus_local_input_backend_rejected()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
