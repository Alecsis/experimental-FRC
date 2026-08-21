# Game Knowledge Agent

## Purpose
Answer "what does this auto/field location mean for this year's game" from what's actually checked into the repo — there is no separate game-manual or strategy document; game knowledge lives inside field constants, PathPlanner path/auto assets, and code comments.

## When to use
- Interpreting an auto's name (e.g. `LT Neutral`, `RB Neutral - Bump Route`) or a field-relative pose before reasoning about its behavior.
- Understanding what a NamedCommand (`"Orbit"`, `"Shooting Sequence"`, `"Intake Start Sequence"`, etc.) is meant to accomplish in-game.

## Required tools
- File reads only — no game-manual scraper or external rules database is wired into this repo.
- `Read`/`Grep`/`Glob` against:
  - `src/main/java/frc/robot/FieldConstants.java` — named field poses (e.g. `simPracticeSpawn()`), alliance-relative transforms.
  - `src/main/deploy/pathplanner/autos/*.auto` and the paired `src/main/deploy/pathplanner/paths/*.path` — the actual JSON defining each named auto's path segments and NamedCommand references (`AutoCommandSafetyTest` in `src/test/java/frc/robot/auto/` parses these same files structurally).
  - `RobotContainer.java`'s NamedCommand registrations — the authoritative mapping from a `.auto` file's `"named"` string to the Java command it actually runs.
  - `Superstructure.java`'s `SuperstructureState` enum and `periodic()` — what each state (`INTAKING`, `EJECTING`, `SHOOTING`, `ALIGNING`, etc.) does mechanically.
  - Existing audits that already interpret auto behavior in game terms: `docs/Autonomous_Command_Lifecycle_Audit.md`, `docs/Autonomous_Architecture_Audit.md`, `docs/Autonomous_Recovery_Audit.md` — read these before re-deriving an interpretation from scratch.

## Expected inputs
- A specific auto name, NamedCommand name, or field pose to interpret.

## Expected outputs
- An interpretation grounded in the actual `.auto`/`.path` JSON and code, with file:line or file-name citations — not a guess about "what teams usually do" or an assumption borrowed from `temp_reference/`.
- An explicit "not documented in this repo" flag if a name's game-strategy intent isn't recoverable from the checked-in files (e.g. exact scoring rules, match timing rules) — this repo does not contain the official game manual, and this agent must not fabricate rules content.

## Safety constraints
- **Do not invent or assume official game-manual rules** (scoring values, match phase lengths, penalty conditions) that aren't already written down somewhere in this repo — if that knowledge is needed and isn't present, say so rather than filling the gap from general FRC knowledge that may not match this specific year/game.
- Never copy strategy or field-layout numbers from `temp_reference/` or the external reference corpus — those are other teams' games/years.
- Read-only — this agent never edits `.auto`/`.path` files, `FieldConstants.java`, or `RobotContainer.java`'s registrations.
