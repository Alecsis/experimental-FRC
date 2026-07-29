# Log Analysis Agent

## Purpose
Extract and inspect real data from `.wpilog` files — the evidence gate (gate 3) of the Advanced Agent Verification Loop. Every number this agent reports is computed from log bytes, never asserted from memory or a script's canned output.

## When to use
- After any sim/real/replay run that produced a `.wpilog`, to confirm a specific telemetry claim (a transition happened, a value ramped, a pose didn't jump).
- As part of `/audit` when a claim needs log-backed evidence rather than code-reading alone.
- As the third step of `/regression` whenever a `.wpilog` exists for the run being verified.

## Required tools
- `python SKILLS/parse_akit_log.py <log.wpilog>` — a pure-stdlib WPILOG v1.0 parser (no bindings). Flags:
  - (none) — entry summary (names, types, record counts).
  - `--grep GREP` — filter entries by substring.
  - `--dump ENTRY_NAME` — print decoded values for an exact entry name (`--limit N` caps how many).
  - `--pose-entry ENTRY_NAME [--jump-threshold METERS]` — frame-to-frame pose-jump analysis for `struct:Pose2d`/`double[3]` entries.
- Entry names must match exactly as printed in the summary table, including the leading slash (e.g. `/RealOutputs/Vision/IMUMode`).

## Expected inputs
- Any `.wpilog` under `logs/` — produced by `run_headless_sim.py`, `./gradlew test` (behavior-gate runs write real logs via `Robot.java`'s SIM-mode `WPILOGWriter`), or a REPLAY run's `_sim`-suffixed output.

## Expected outputs
- An entry summary or `--dump`/`--pose-entry` printout, computed live from the file. Prior sessions have used this to confirm things like the `Vision/IMUMode` 1→4 disabled→autonomous transition, `Drive/AppliedVoltsPerModule` ramps during SysId windows, and pose-jump magnitude during vision-rejection scenarios.
- If the requested entry is missing or of an unsupported type, the script says so — that is a correct outcome, not a bug to work around by guessing.

## Safety constraints
- **Never print or accept a canned/hardcoded metric as if it came from the log.** The pre-2026-07-17 version of this script fabricated output ("IMU Fusion Mode 4 detected", "Pose Stability: STABLE") unconditionally — any historical output resembling that pattern is fiction and must not be trusted or reproduced.
- A gate that can't run (missing log, missing entry) is reported UNVERIFIED, never assumed to have passed.
- Quote the actual command and its literal output when reporting a finding — not a paraphrase.
- This script only reads `.wpilog` files; it never modifies robot code, logs, or Gradle state.
