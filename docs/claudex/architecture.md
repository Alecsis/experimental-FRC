# Architecture Rules — Rationale & History

Rationale layer behind `CLAUDE.md`'s `## 📐 Architecture Rules`. The rule statements in `CLAUDE.md` are the enforceable contract; this file is *why* each rule exists, what's been decided about its edges, and what's still an open, accepted gap. Update this file (not `CLAUDE.md`) when a session establishes or revises an architecture decision.

## Rule 1 — Strict Hardware Isolation

No vendor hardware APIs (CTRE Phoenix, REVLib, etc.) are allowed in standard subsystem files. They must live strictly inside `*IOReal.java` implementations.

- **Sanctioned vendor exception (decided 2026-07-16):** `subsystems/CommandSwerveDrivetrain.java` and `utility/simulation/MapleSimSwerveDrivetrain.java` are the **singular** sanctioned exception to this rule. They are CTRE Tuner X / swerve-generator output and stay vendor-inline rather than wrapped in a `DriveIO` — regenerating them from Tuner X would otherwise conflict with a hand-maintained IO wrapper. Every other file must stay vendor-free.
- **Utility layer is NOT exempt.** `utility/HubActiveState.java` used to import `com.ctre.phoenix6` (`HootAutoReplay`, `Utils`) directly for Hoot auto-log/auto-replay. That hookup is now isolated in `utility/HootReplayBridge.java` (package-private, the only file in `utility/` that touches Phoenix 6); `HubActiveState.java` itself is vendor-free.
- **Blast radius, accepted (2026-07-16 verification scan):** `RobotContainer.java` and `Telemetry.java` import `com.ctre.*` (`SwerveRequest`, `SwerveDriveState`, `SignalLogger`). This is the *blast radius* of the sanctioned `CommandSwerveDrivetrain` exception — its public API is vendor-typed — not a new Rule 1 leak. Accepted; wrapping would mean wrapping the drivetrain itself, which we decided against.

## Rule 2 — Singleton Subsystems

Every subsystem and the `Superstructure` must implement the Singleton pattern (`private` constructor and `public static Subsystem getInstance()`). No history beyond the rule itself — this has held unchanged since the Intake/Shooter/Vision refactors (see `docs/claudex/history.md`).

## Rule 3 — Centralized States

The `Superstructure` coordinates complex multi-subsystem states via a `SuperstructureState` enum (`OFF`, `INTAKING`, `EJECTING`, `STOWED`, `ALIGNING`, `SHOOTING`) and a `periodic()` switch-case that commands Shooter/Intake/Vision directly.

- `shootCmd()`/`shootingSequence(double)` are thin Command bridges kept only because `RobotContainer`'s `whileTrue` binding and PathPlanner's NamedCommands both require real Command objects — the actual arbitration logic lives in `periodic()`, not in composed Commands.
- Subsystems themselves still only handle their own immediate mechanism control (e.g. `Intake.agitatePivot()` stays owned by `Intake`; `Superstructure` schedules/cancels it as a unit rather than reimplementing its oscillation).
- **Known, accepted gap (2026-07-16 verification scan):** `Superstructure.shootCmd()` declares its requirement (`this`); `shootingSequence()` and `Intake.agitatePivot()` declare **no scheduler requirements**, and `Superstructure.periodic()` writes mechanism setpoints without requirements. Safe today by binding discipline, not enforcement. **Trigger to revisit:** first observed "two writers, same tick" conflict. Full analysis in the vault Deep Dive, Part II §6.1.

## Reference-pattern adoption queue

A priority-ordered queue of patterns from 254/1678/6328 sized for adoption in this codebase lives in the vault: `Projects/Elite Reference Architecture Deep Dive.md` §5 + §8. Top items as of 2026-07-17: loop phase split + LoggedTracer, config-retry wrapper, cached-signal audit of `IntakeIOReal`/`ShooterIOReal`. (The CANcoder boot barrier item from that queue is done — see `docs/claudex/history.md`.)
