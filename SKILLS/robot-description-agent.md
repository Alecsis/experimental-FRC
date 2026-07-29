# Robot Description Agent

## Purpose
Answer "what is this robot, physically and architecturally" from the checked-in source of truth, instead of guessing or reusing numbers from `temp_reference/`.

## When to use
- Before any change that depends on a physical constant (mass, wheel radius, gear ratio, bumper footprint, wheel COF) or an architectural rule (hardware isolation, singleton pattern, Superstructure state machine).
- When an audit or plan needs to cite the robot's actual configuration rather than assume it.

## Required tools
- File reads only — no script exists for this; it is a matter of reading the right files.
- `Read`/`Grep`/`Glob` against:
  - `src/main/java/frc/robot/Constants.java` — mass, bumper footprint, wheel COF, and other physical/robot-wide constants.
  - `src/main/java/frc/robot/FieldConstants.java` — field-relative poses, including `simPracticeSpawn()`.
  - `src/main/java/frc/robot/generated/TunerConstants.java` — CTRE Tuner X-generated drivetrain constants (gear ratios, `kSpeedAt12Volts`, `Slot0Configs` drive/steer gains). Treat this file as generated output per `CLAUDE.md`'s sanctioned-exception rule for `CommandSwerveDrivetrain.java`/`MapleSimSwerveDrivetrain.java` — read it, don't hand-tune it without going through the Tuner X workflow or an explicit, sourced characterization task.
  - `src/main/deploy/pathplanner/settings.json` — PathPlanner's own copy of mass/bumper/wheel-COF/robotMOI; must stay consistent with `Constants.java` (see `CLAUDE.md` Backlog — this was reconciled 2026-07-18, not independently measured).
  - `CLAUDE.md`'s `## Architecture Rules` section and `docs/claudex/architecture.md` for the hardware-isolation boundary, singleton requirement, and Superstructure state-machine rule.

## Expected inputs
- A specific question ("what is the drive gear ratio", "what motors drive the shooter", "is the chassis rectangular").

## Expected outputs
- A direct answer with a file:line citation, not a paraphrase from memory.
- An explicit "not yet measured" flag for anything `CLAUDE.md`'s Backlog already marks as unmeasured (e.g. real mass/MOI/wheel-COF against the physical robot) — internal consistency between `Constants.java` and `settings.json` is not the same as ground-truth measurement.

## Safety constraints
- **Never copy a number out of `temp_reference/` or `C:\Users\xdm\frc-steal-from-the-best\`.** Those describe other teams' robots and years — per `CLAUDE.md`, read them only for structural patterns, never for values.
- Never edit files under `temp_reference/`.
- This agent is read-only — it answers questions, it does not change `Constants.java`, `TunerConstants.java`, or `settings.json`. Any actual constant change is a production-code task outside this agent's scope and needs its own verification-loop gates.
- If a referenced class or file doesn't exist (per `CLAUDE.md`'s "Don't Invent Files" rule), say so and stop rather than inventing or copying one in from a reference directory.
