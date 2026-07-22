"""Tests for the context snapshot collector (dep-free). Run: python tests/test_snapshot.py"""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from claude_supervisor.context_snapshot import (
    ContextSnapshot,
    _infer_tests,
    collect_snapshot,
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


def test_infer_tests():
    print("test_infer_tests")
    check(_infer_tests("... BUILD FAILED in 3s").startswith("FAIL"), "BUILD FAILED -> FAIL")
    check(_infer_tests("BUILD SUCCESSFUL\n42 tests").startswith("PASS"), "BUILD SUCCESSFUL -> PASS")
    check(_infer_tests("just some logs") == "unknown", "no markers -> unknown")


def test_sidecar_overrides_and_git_safe():
    print("test_sidecar_overrides_and_git_safe")
    # A temp dir that is NOT a git repo -> git calls fail gracefully, sidecar wins.
    with tempfile.TemporaryDirectory() as d:
        (Path(d) / ".claude-supervisor-status.json").write_text(
            json.dumps({
                "current_task": "Implementing autonomous speed clamp",
                "completed": ["Fixed sim spawn ownership", "Added trajectory telemetry"],
                "tests": "PASS",
            }),
            encoding="utf-8",
        )
        snap = collect_snapshot(
            d,
            output_tail="BUILD FAILED somewhere",  # should be ignored: sidecar sets tests
            needs="clarification",
            question="Clamp before or after PathPlanner output?",
        )
    check(snap.current_task == "Implementing autonomous speed clamp", "current_task from sidecar")
    check("Added trajectory telemetry" in snap.completed, "completed from sidecar")
    check(snap.completed_from_commits is False, "sidecar completed not marked as commits")
    check(snap.tests == "PASS", "sidecar tests wins over output heuristic")
    check(snap.needs == "clarification" and snap.question, "needs/question passed through")
    check(bool(snap.project), "project falls back to dir name when not a git repo")


def test_to_text_renders_example_shape():
    print("test_to_text_renders_example_shape")
    snap = ContextSnapshot(
        captured_at="now",
        project="Cheesy-Citrus",
        branch="auto-refactor",
        needs="architecture decision",
        question="Should this clamp happen before or after PathPlanner controller output?",
        current_task="Implementing autonomous speed clamp",
        completed=["Fixed sim spawn ownership", "Added trajectory telemetry"],
        changed_files=["CommandSwerveDrivetrain.java"],
        tests="PASS",
    )
    text = snap.to_text()
    for token in ("Claude Checkpoint", "Cheesy-Citrus", "auto-refactor",
                  "Completed", "Fixed sim spawn ownership", "Implementing autonomous speed clamp",
                  "CommandSwerveDrivetrain.java", "Tests: PASS"):
        check(token in text, f"to_text contains {token!r}")


def test_real_repo_snapshot():
    print("test_real_repo_snapshot")
    # Run against this repo (we're inside experimental-FRC).
    snap = collect_snapshot(str(Path(__file__).resolve().parent), output_tail="x")
    check(bool(snap.branch) and snap.branch != "?", "branch read from real git repo")
    check(bool(snap.project), "project read from real git repo")


def main():
    test_infer_tests()
    test_sidecar_overrides_and_git_safe()
    test_to_text_renders_example_shape()
    test_real_repo_snapshot()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
