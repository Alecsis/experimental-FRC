---
description: Run the project's regression gates and report pass/fail, distinguishing known flakes from new regressions.
---

# /regression

Run the Advanced Agent Verification Loop's gates (full definitions: `docs/claudex/verification-loop.md`) and summarize the result. Read-only against source — this command runs builds/tests, it does not fix failures itself.

Extra context from the user (may be empty — e.g. "just gates 1-2", "include the recovery/disturbance package"): $ARGUMENTS

## Gates, in order (stop at the first hard failure that isn't a known flake)

1. **`./gradlew compileJava`** — static gate. Requires `JAVA_HOME` pointed at the WPILib JDK 17 (`CLAUDE.md` Build & Test Commands) or Gradle configuration itself fails under the machine's default JDK 25.
2. **`python SKILLS/run_headless_sim.py`** — launch gate (see `SKILLS/simulation-agent.md`). Proves boot/construction only, not behavior.
3. **`./gradlew test`** — behavior gate. Runs the full JUnit suite, including `AutoRegressionTestBase` subclasses (`src/test/java/frc/robot/auto/`) and, if present, the disposable `PathDisturbanceSimTestBase` package (`src/test/java/frc/robot/recovery/`) and `AutonomousHealthMonitorTest`.
4. **Full regression suite** — when the change is broad enough that a targeted subset isn't sufficient, re-run gate 3 in full and, if a `.wpilog` was produced for the specific claim being verified, follow with `SKILLS/parse_akit_log.py` (gate 3 of the verification loop / `SKILLS/log-analysis-agent.md`).

## Reporting

- Quote actual exit codes and output — never paraphrase a PASS/FAIL.
- **Known documented flakes are not regressions.** Before reporting a failure as new, check whether it matches an already-documented flaky signature in `CLAUDE.md`'s Current Task State / `docs/claudex/history.md` — e.g. `LtNeutralAutoRegressionTest`'s razor-thin stall-check margin, or the `SimHooks.stepTiming()` JNI-hang class affecting tests like `IntakeJamRecoveryTest`. If a failure matches, say so explicitly and cite the prior occurrence instead of flagging it as a regression.
- If a failure does **not** match a documented flake, treat it as a real regression and say so plainly — do not default to "probably flaky" without evidence (an A/B stash-and-rerun, per `CLAUDE.md`'s established practice, is the way to confirm either way).
- A gate that couldn't run (e.g. no `.wpilog` produced) is UNVERIFIED for whatever it would have proven — never reported as passed.

## Rules

- Never skip a gate and report it as passed.
- Never fix a failure as a side effect of running `/regression` — report it and let the user decide the next step, per this project's surgical-changes-only rule.
