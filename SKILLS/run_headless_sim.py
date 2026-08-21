#!/usr/bin/env python3
"""Headless launch-check for the robot sim.

Runs `gradlew simulateJava -Pheadless=true` (build.gradle disables the sim GUI
when that property is set), waits for the robot program to reach init, lets it
run a settle window, then kills the whole process tree.

What a PASS means: the build compiled, every subsystem singleton constructed,
and the disabled-mode periodic loop ran for --run-seconds without an exception.
What a PASS does NOT mean: no command or state-machine behavior was exercised —
the sim boots disabled with no driver station attached, so the scheduler never
runs commands. Behavior claims require a JUnit/SimHooks test or an interactive
sim session.

Exit codes: 0 = launch clean, 1 = runtime exception or init never reached,
2 = build/environment failure.
"""

import argparse
import os
import queue
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

# Windows consoles/pipes often default to cp1252, which can't encode emoji.
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

PROJECT_ROOT = Path(__file__).resolve().parent.parent
INIT_MARKER = "********** Robot program starting **********"
# WPILib JDK 17 — the machine PATH java is JDK 25, which Gradle cannot run.
WPILIB_JDK_WIN = Path("C:/Users/Public/wpilib/2026/jdk")

ERROR_RE = re.compile(r"\bException\b|\bFAILURE:|BUILD FAILED|Unhandled exception")
BENIGN_RE = re.compile(r"BUILD SUCCESSFUL|0 errors")


def build_env():
    env = os.environ.copy()
    override = env.get("WPILIB_JDK")
    jdk = Path(override) if override else WPILIB_JDK_WIN
    if jdk.is_dir():
        env["JAVA_HOME"] = str(jdk)
        env["PATH"] = str(jdk / "bin") + os.pathsep + env.get("PATH", "")
    else:
        print(f"⚠️  WPILib JDK not found at {jdk}; using existing JAVA_HOME "
              f"({env.get('JAVA_HOME', 'unset')}). Gradle may fail under JDK 25.")
    return env


def gradle_cmd():
    wrapper = "gradlew.bat" if os.name == "nt" else "./gradlew"
    return [str(PROJECT_ROOT / wrapper), "simulateJava",
            "-Pheadless=true", "--console=plain", "--no-daemon"]


def kill_tree(process):
    if process.poll() is not None:
        return
    if os.name == "nt":
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(process.pid)],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        import signal
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        process.kill()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--launch-timeout", type=float, default=420.0,
                    help="seconds allowed for build + robot init (default 420)")
    ap.add_argument("--run-seconds", type=float, default=12.0,
                    help="seconds to let the sim run after init (default 12)")
    ap.add_argument("--echo", action="store_true",
                    help="echo every sim/gradle output line")
    ap.add_argument("--parse", action="store_true",
                    help="on PASS, chain straight into parse_akit_log.py on the newest logs/*.wpilog")
    args = ap.parse_args()

    print(f"🤖 Launching headless sim: {' '.join(gradle_cmd())}")
    popen_kwargs = dict(cwd=str(PROJECT_ROOT), env=build_env(),
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                        text=True, errors="replace")
    if os.name != "nt":
        popen_kwargs["start_new_session"] = True
    process = subprocess.Popen(gradle_cmd(), **popen_kwargs)

    lines = queue.Queue()

    def pump():
        for line in process.stdout:
            lines.put(line.rstrip())
        lines.put(None)

    threading.Thread(target=pump, daemon=True).start()

    tail = []          # last lines seen, for failure diagnostics
    errors = []
    initialized = False
    init_time = None
    eof = False
    start = time.time()

    try:
        while True:
            now = time.time()
            if not initialized and now - start > args.launch_timeout:
                print(f"⏰ Init marker not seen within {args.launch_timeout:.0f}s.")
                break
            if initialized and now - init_time > args.run_seconds:
                break
            try:
                line = lines.get(timeout=0.5)
            except queue.Empty:
                if process.poll() is not None and eof:
                    break
                continue
            if line is None:
                eof = True
                continue
            tail.append(line)
            del tail[:-40]
            if args.echo:
                print(f"  [SIM] {line}")
            if ERROR_RE.search(line) and not BENIGN_RE.search(line):
                errors.append(line)
            if INIT_MARKER in line and not initialized:
                initialized = True
                init_time = time.time()
                print(f"🏁 Robot program started after {init_time - start:.1f}s; "
                      f"running {args.run_seconds:.0f}s settle window...")
    finally:
        kill_tree(process)

    exited_early = process.poll() is not None and not initialized
    print("\n--- Sim launch-check report ---")
    if errors:
        print(f"❌ FAIL: {len(errors)} error line(s) during build/run:")
        for err in errors[:15]:
            print(f"   -> {err}")
        sys.exit(2 if not initialized else 1)
    if exited_early or not initialized:
        print("❌ FAIL: robot program never reached init. Last output:")
        for line in tail[-20:]:
            print(f"   | {line}")
        sys.exit(2)
    print(f"✅ PASS: robot init reached; {args.run_seconds:.0f}s of disabled-mode "
          "periodic loops with no exceptions.")
    print("   Scope: launch/construction only — no commands or state-machine "
          "behavior were exercised (sim boots disabled, no DS attached).")

    log_dir = PROJECT_ROOT / "logs"
    wpilogs = sorted(log_dir.glob("*.wpilog"), key=lambda p: p.stat().st_mtime) if log_dir.is_dir() else []
    if wpilogs:
        newest = wpilogs[-1]
        print(f"📄 Newest log: {newest}")
        if args.parse:
            parser_script = PROJECT_ROOT / "SKILLS" / "parse_akit_log.py"
            print(f"--- Parsing {newest.name} ---")
            subprocess.run([sys.executable, str(parser_script), str(newest)], cwd=str(PROJECT_ROOT))
    else:
        print("⚠️  No logs/*.wpilog found — SIM mode should have written one this run.")
        if args.parse:
            print("   --parse requested but there is nothing to parse.")

    sys.exit(0)


if __name__ == "__main__":
    main()
