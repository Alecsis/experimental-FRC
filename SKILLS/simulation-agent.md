# Simulation Agent

## Purpose
Boot the robot in simulation and confirm it launches cleanly — the launch/construction gate (gate 2) of the project's Advanced Agent Verification Loop.

## When to use
- After any change to production robot code, before claiming it works.
- As the second gate in `/regression`, after `compileJava`.
- When you need a fresh `.wpilog` for the log-analysis-agent to read, and a JUnit test isn't the right vehicle (no behavior claim, just "does it boot").

## Required tools
- `python SKILLS/run_headless_sim.py` (use `python3` on POSIX shells) — the only sim-launch script in this repo. It sets `JAVA_HOME` to the WPILib JDK itself; no manual `export` needed for this specific script (see `CLAUDE.md`'s Build & Test Commands for the manual JDK 17 requirement when running raw `gradlew` commands).
- Optional flags: `--run-seconds N` (settle window, default 12), `--launch-timeout N` (default 420), `--echo` (stream sim output live), `--parse` (chain straight into `parse_akit_log.py` on the newest `logs/*.wpilog`).

## Expected inputs
- A compiling tree (`./gradlew compileJava` should already pass — this script does not skip the build).
- No driver station attachment; the script always boots disabled.

## Expected outputs
- Exit 0 + `✅ PASS: robot init reached; Ns of disabled-mode periodic loops with no exceptions.` — proves the build compiled, every subsystem/Superstructure singleton constructed without throwing, and disabled periodic loops ran clean.
- Exit 1/2 on a runtime exception, build failure, or init marker never seen within the timeout — treat as a hard FAIL, not a flake, unless independently reproduced as pre-existing.
- A new file under `logs/*.wpilog` on PASS (`Robot.java`'s SIM case always adds a `WPILOGWriter`).

## Safety constraints
- **A PASS here does not prove command or state-machine behavior.** The sim boots disabled with no DS attached, so the scheduler never runs `intakeCmd()`/`shootCmd()`/etc. Behavior claims require the behavior gate (`./gradlew test`, see `docs/claudex/verification-loop.md` gate 2.5) or an interactive `simulateJava` session.
- Never report this gate as passed if it was skipped — an unrun gate is UNVERIFIED, not assumed green.
- Do not hand-edit or fabricate the printed metrics; the script never prints canned output, and neither should any summary of it.
