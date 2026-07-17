# Advanced Agent Verification Loop

Full gate definitions, mechanics, and honesty rules behind `CLAUDE.md`'s Verification Loop mandate. *Adopted 2026-07-17 after the SKILLS/ audit. "Caveman debugging" — eyeballing code, or trusting a script's printed "SUCCESS" without knowing what it measured — is banned outright.*

**Mandate: every agent workflow that modifies robot code MUST pass gate 1 and gate 2 (`python SKILLS/run_headless_sim.py`; use `python3` on POSIX shells) before claiming completion.** Gate 3 runs whenever a `.wpilog` exists for the claim being verified. A skipped gate means the claim is reported UNVERIFIED — no exceptions.

## The lifecycle (run in order; stop at the first failing gate)

1. **Static gate:** `./gradlew compileJava` exits 0 (with `JAVA_HOME` → WPILib JDK 17, see `CLAUDE.md` Build & Test Commands). Never claim a change works before this passes.
2. **Runtime launch gate:** `python SKILLS/run_headless_sim.py`. Launches `gradlew simulateJava -Pheadless=true --no-daemon` (build.gradle disables the sim GUI when `-Pheadless` is set), sets `JAVA_HOME` itself, waits up to 420 s for `********** Robot program starting **********`, then lets the sim run a 12 s settle window and kills the full process tree (`taskkill /T` on Windows).
   - **A PASS proves:** the build compiled, every subsystem/Superstructure singleton constructed without throwing, and disabled-mode periodic loops ran clean.
   - **A PASS does NOT prove:** any command or state-machine *behavior*. The headless sim boots **disabled with no driver station attached**, so the scheduler never runs `intakeCmd()`/`ejectCmd()`/`shootCmd()`. Behavior claims require a JUnit test driving `DriverStationSim`/`SimHooks`, or an interactive `simulateJava` session with the GUI.
2.5. **Behavior gate:** `./gradlew test`. Runs the JUnit 5 suite under `src/test/java` — currently
   two classes, each a real `Robot.startCompetition()` loop test via `DriverStationSim`/`SimHooks`
   (never direct `disabledInit()`/`autonomousInit()` calls, see `build.gradle`'s `test{}` comment):
   `RobotLifecycleTest` asserts the `Vision/IMUMode` 1→4 disabled→autonomous transition (see
   `docs/superpowers/plans/2026-07-17-phase3-imu-mode-junit-test.md`), and `SuperstructureEjectingTest`
   asserts `superstructure.ejectCmd()` drives `EJECTING` (roller/indexer/agitator reversed) and that
   cancelling it stows the roller (see
   `docs/superpowers/plans/2026-07-17-phase3-test-b-ejecting-junit-test.md`). Unlike gate 2, this
   exercises actual state-machine behavior, not just launch/construction. A PASS here plus a parsed
   `.wpilog` (gate 3, where applicable) is what "verified" means for these fixes.
3. **Evidence gate:** `python SKILLS/parse_akit_log.py <file.wpilog>` — a real WPILOG v1.0 binary parser (entry table, record counts, `--dump`, and `--pose-entry`/`--jump-threshold` frame-to-frame pose-jump analysis for `struct:Pose2d`/`double[3]` entries). Every number it prints is computed from the log bytes. Hardened 2026-07-17 against three spec edge cases verified by synthetic-log self-test: records are sorted by timestamp before analysis (the spec does not guarantee order), entry-ID reuse after a Finish record no longer drops the earlier entry's data, and all standard array types (`boolean[]`/`int64[]`/`float[]`/`string[]`/`double[]`) decode.
   - **Evidence path (closed 2026-07-17):** `Robot.java`'s SIM case now adds a `WPILOGWriter("logs")`
     alongside `NT4Publisher`, and `Vision.periodic()` now logs `Vision/IMUMode`/`Vision/IMUAssistAlpha`
     every tick. A `./gradlew test` run (gate 2.5) produces a real `.wpilog` under `logs/` that
     `--dump "/RealOutputs/Vision/IMUMode"` confirms transitions 1→4 at the disabled→autonomous
     boundary — verified from log bytes, not memory. See
     `docs/superpowers/plans/2026-07-17-phase3-imu-mode-junit-test.md` Task 3 for the full evidence run.

## Honesty rules (non-negotiable)

- Verification scripts must **never print canned or hardcoded metrics**. The pre-2026-07-17 `parse_akit_log.py` fabricated `"IMU Fusion Mode 4 detected"` and `"Pose Stability: STABLE"` unconditionally — treat any historical output from it as fiction.
- A gate that cannot run is reported as **UNVERIFIED**, never as passed. Partial verification states exactly what was and wasn't exercised.
- Success claims quote the actual command exit code / output, not a paraphrase.
