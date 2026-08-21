"""Context snapshot: everything a human needs to make a call from their phone.

Collected whenever Claude enters WAITING_FOR_HUMAN and on demand via !context.
Pure stdlib (subprocess for git) so it is independently testable. Every field is
best-effort and failure-safe -- collection must never raise into the event loop.

Sources, in order of trust:
  * git          -- repo name, branch, status, diff summary, changed files
  * PTY buffer   -- last N lines of Claude's own output
  * sidecar file -- optional JSON the workflow (or Claude itself) maintains for
                    current_task / completed / tests, since hooks cannot supply them
  * heuristics   -- fallback: recent commit subjects for "completed", output scan
                    for test pass/fail. Clearly labelled as derived, never invented.
"""

from __future__ import annotations

import datetime as _dt
import json
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

DEFAULT_SIDECAR = ".claude-supervisor-status.json"


def _git(cwd: str, args: list[str], timeout: float = 5.0) -> str:
    try:
        out = subprocess.run(
            ["git", "-C", cwd, *args],
            capture_output=True, text=True, timeout=timeout,
        )
        return out.stdout.strip() if out.returncode == 0 else ""
    except Exception:
        return ""


def _clip_lines(text: str, n: int) -> str:
    lines = [ln for ln in text.splitlines() if ln.strip()]
    return "\n".join(lines[:n])


# Test-result heuristics over the tail of Claude's output.
_FAIL_MARKERS = ("BUILD FAILED", "FAILED", "tests failed", "FAIL:", " FAILED ",
                 "AssertionError", "compilation failed")
_PASS_MARKERS = ("BUILD SUCCESSFUL", "0 failures", "all tests passed",
                 "passed,", "PASSED", "0 failed")


def _infer_tests(output_tail: str) -> str:
    low = output_tail.lower()
    fails = [m for m in _FAIL_MARKERS if m.lower() in low]
    if fails:
        return "FAIL (detected in output)"
    if any(m.lower() in low for m in _PASS_MARKERS):
        return "PASS (detected in output)"
    return "unknown"


@dataclass
class ContextSnapshot:
    captured_at: str
    project: str
    branch: str
    needs: Optional[str] = None        # waiting reason
    question: Optional[str] = None     # last prompt / assistant message
    current_task: Optional[str] = None
    completed: list[str] = field(default_factory=list)
    completed_from_commits: bool = False
    changed_files: list[str] = field(default_factory=list)
    diff_summary: str = ""
    git_status: str = ""
    tests: str = "unknown"
    output_tail: str = ""

    def to_json(self) -> dict:
        d = self.__dict__.copy()
        return d

    def to_text(self, max_len: int = 1800) -> str:
        """Compact plain-text rendering for the !context command."""
        completed_label = "Recent commits" if self.completed_from_commits else "Completed"
        lines = [
            "🚨 Claude Checkpoint",
            "",
            f"Project: {self.project}",
            f"Branch:  {self.branch}",
        ]
        if self.completed:
            lines.append(f"\n{completed_label}:")
            lines += [f"  - {c}" for c in self.completed[:8]]
        lines.append(f"\nCurrent: {self.current_task or '—'}")
        lines.append(f"Needs:   {self.needs or '—'}")
        if self.question:
            lines.append(f"Question: {self.question}")
        if self.changed_files:
            lines.append(f"\nFiles:\n  " + "\n  ".join(self.changed_files[:12]))
        lines.append(f"\nTests: {self.tests}")
        if self.diff_summary:
            lines.append(f"\nDiff summary:\n{self.diff_summary}")
        if self.output_tail:
            lines.append(f"\n--- output tail ---\n{self.output_tail[-800:]}")
        text = "\n".join(lines)
        return text[:max_len]


def _load_sidecar(cwd: str, sidecar_name: str) -> dict:
    try:
        p = Path(cwd) / sidecar_name
        if p.exists():
            return json.loads(p.read_text(encoding="utf-8")) or {}
    except Exception:
        pass
    return {}


def collect_snapshot(
    cwd: str,
    *,
    output_tail: str = "",
    needs: Optional[str] = None,
    question: Optional[str] = None,
    sidecar_name: str = DEFAULT_SIDECAR,
) -> ContextSnapshot:
    """Assemble a snapshot. Never raises."""
    toplevel = _git(cwd, ["rev-parse", "--show-toplevel"])
    project = Path(toplevel).name if toplevel else Path(cwd).name
    branch = _git(cwd, ["rev-parse", "--abbrev-ref", "HEAD"]) or "?"
    status = _clip_lines(_git(cwd, ["status", "--short"]), 30)
    diff_summary = _clip_lines(_git(cwd, ["diff", "--stat", "HEAD"]), 20)

    changed_files: list[str] = []
    for ln in _git(cwd, ["status", "--porcelain"]).splitlines():
        name = ln[3:].strip() if len(ln) > 3 else ln.strip()
        if "->" in name:  # renamed: "old -> new"
            name = name.split("->")[-1].strip()
        if name:
            changed_files.append(name)

    sidecar = _load_sidecar(cwd, sidecar_name)

    completed = sidecar.get("completed") or []
    completed_from_commits = False
    if not completed:
        commits = _git(cwd, ["log", "--oneline", "-6", "--pretty=%s"])
        completed = [c for c in commits.splitlines() if c.strip()]
        completed_from_commits = bool(completed)

    tests = sidecar.get("tests") or _infer_tests(output_tail)
    current_task = sidecar.get("current_task")

    return ContextSnapshot(
        captured_at=_dt.datetime.now(_dt.timezone.utc).isoformat(),
        project=project,
        branch=branch,
        needs=needs,
        question=(question or "").strip() or None,
        current_task=current_task,
        completed=list(completed),
        completed_from_commits=completed_from_commits,
        changed_files=changed_files,
        diff_summary=diff_summary,
        git_status=status,
        tests=str(tests),
        output_tail=output_tail,
    )
