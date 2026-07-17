# PLAN.md — Closing the Evidence Gap in the Agent Verification Loop

*Drafted 2026-07-17. Goal: make the MegaTag2 `IMUMode(4)` auto fix and Superstructure
state behavior provable from real log bytes, so gate 3 of the Advanced Agent
Verification Loop (CLAUDE.md) stops being blocked. Plan only — no robot code has
been changed yet.*

## Current blockers (verified against the tree)
1. `Robot.java:60-63` — SIM mode adds only `NT4Publisher`; no `WPILOGWriter`, so a
   headless sim run produces no `.wpilog` for `SKILLS/parse_akit_log.py`.
2. Nothing logs the Limelight IMU mode. `Vision.setIMUMode(int)` forwards to the IO
   layer and forgets; no `Vision/IMUMode` entry exists in any log.
3. The headless sim boots **disabled** with no driver station attached —
   `autonomousInit()` never fires and no command ever runs, so even with 1+2 fixed,
   a plain headless run cannot exercise the IMU-mode switch or `EJECTING`.

## Phase 1 — Sim-mode WPILOGWriter (unblocks gate 3 mechanically)
- `Robot.java` SIM case: add `Logger.addDataReceiver(new WPILOGWriter("logs"))`
  alongside the `NT4Publisher`. Mirror the REAL case's folder-path usage
  (`new WPILOGWriter("media/sda1/logs")`) — same constructor shape, but confirm
  against AdvantageKit docs via context7 at implementation time per Karpathy rules.
- **Recommendation: always-on in SIM** (simplicity first) rather than env-gated —
  deterministic, no env plumbing through Gradle to the sim JVM; cost is a small
  file per run. Add `logs/` to `.gitignore`.
- `SKILLS/run_headless_sim.py`: after a PASS, print the newest `logs/*.wpilog`
  path; optional `--parse` flag chains straight into `parse_akit_log.py`.
- **Verify:** headless run → `.wpilog` appears → parser lists real entries
  (`Vision/...`, `Intake/...`, `RealOutputs/...`).

## Phase 2 — IMU-mode telemetry (makes the fix observable)
- `Vision.java`: keep a `private int currentIMUMode` field updated in
  `setIMUMode(int)`; each `periodic()`, `Logger.recordOutput("Vision/IMUMode",
  currentIMUMode)`. AdvantageKit-only imports — Architecture Rule 1 safe, no
  vendor/LimelightHelpers leak.
- Also worth logging `Vision/IMUAssistAlpha` while in there (same pattern, one line).
- **Verify:** parser `--dump "RealOutputs/Vision/IMUMode"` on any log shows mode 1
  while disabled (from `disabledInit`/`disabledPeriodic`).

## Phase 3 — Enabled-mode evidence (the actual payoff)
- Add a JUnit sim test (`src/test/java`) driving the robot lifecycle with
  `DriverStationSim.setAutonomous(true)` + `setEnabled(true)` + `notifyNewData()`,
  stepping time via `SimHooks.stepTiming()`. JUnit 5 is already wired in
  `build.gradle` (`useJUnitPlatform`, `wpi.java.configureTestTasks`).
  - **Test A (IMU fix):** assert `Vision/IMUMode` transitions 1 → 4 across the
    disabled → autonomous boundary, then parse the produced log and run the
    pose-jump analysis against the 0.15 m threshold.
  - **Test B (EJECTING):** schedule `superstructure.ejectCmd()`, step several
    ticks, assert intake pivot down + roller reversed + indexer ejecting; on
    cancel, assert `STOWED` commands `Roller.STOP`.
- Wire `./gradlew test` into the Verification Loop as gate 2.5 once these exist.
- Interim manual check (until Phase 3 lands): interactive `./gradlew simulateJava`
  with the GUI, sim controller port 3, per the "Next up" items in CLAUDE.md.

## Sequencing & risk
- Phases are independent and individually revertible (each touches 1–2 files).
- Phase 1 before Phase 2 (no point logging a mode no file captures).
- Karpathy rules apply throughout: surgical diffs, verify AdvantageKit/WPILib
  signatures via context7 before writing, `./gradlew compileJava` after each phase,
  and no success claims without gate output.
- **User decision needed before Phase 1:** always-on sim logging (recommended) vs
  gated. Nothing here executes until approved.
