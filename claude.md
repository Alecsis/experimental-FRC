# CLAUDE.md - FRC Hybrid Refactor Guidelines

## 🤖 System Context
- **Project Type:** FRC Robotics Code (2026/2027 Transition)
- **Team:** 4935 (Trex4935). Both this repo and `C:\Users\xdm\frc-2026\2026` declare `"teamNumber": 4935`. The Obsidian vault folder is named `6767 frc bible` — that's a naming meme, not a team number.
- **User Role:** Lead Programmer/Mentor overseeing the system architecture.
- **Goal:** Build a modular, simulation-first codebase combining 1678's IO-isolation and 254's state machine.

## 🧠 Permanent Brain Sync (Obsidian Workflow)
- **Vault Location:** `C:\Users\xdm\6767 frc bible`
- **Sync Rule:** Claude reads this file at the start of every session to establish state. At the end of every session, the user invokes `/tldr` to update both this file and append new codebase memories (critical math, calibrations, or fixes) to the central Obsidian Vault.
- **Vault entry point:** `00 Index.md`. Key notes: `Best Practices & Upgrades.md` (peer benchmarking), `Control Loops & Math/PID Gains Registry.md`, `Control Loops & Math/Swerve Offsets & Kinematics.md`, `WPILib & Simulation/Field & Alliance Coordinates.md`.
- The vault is plain markdown — write to it with normal file tools. The Obsidian CLI is **not** installed and is not required.

## 🛠️ Superpowers (Available MCP Tools & Plugins)
- **Context7 (online):** Up-to-date WPILib & JavaDocs API reference layer.
- **WebSearch (built-in search):** Real-time web-search validation engine.
- **WebFetch (built-in scraping):** Remote markdown/URL scraping capability.

## 📂 Reference Directories (`temp_reference/`)
You have read access to elite architectural templates inside these subdirectories. Use them for structural examples, but **never merge or import them directly**:
- `temp_reference/Lynk 2026/`
- `temp_reference/Phenoix 6 API Examples/` *(yes, misspelled on disk)*
- `temp_reference/Team 254 Code/`
- `temp_reference/Team 1678 Code/`
- `temp_reference/Team 6328 Code/`

A larger corpus also lives at `C:\Users\xdm\frc-steal-from-the-best\` (254, 1678, 6328/MechAdv, Lynk, YAMS, Phoenix 6 examples). Same rule: read, never merge.

> **Never copy numbers out of reference code.** Their masses, gains and field constants describe their robot and their year. Copy patterns, rewrite against our `Constants` / `FieldConstants`.

## 🛠️ Build & Test Commands
- **Compile Java:** `./gradlew compileJava`
- **Build Robot:** `./gradlew build`
- **Run Simulator:** `./gradlew simulateJava`

**JDK requirement:** the machine's PATH `java` is JDK 25, which Gradle 8.11 cannot run — every task dies during configuration with `Could not create task ':test'` / `Type T not present`. This is an environment fault, not a code fault. Export the WPILib JDK 17 first:

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"   # Temurin 17.0.16
```

## 📐 Architecture Rules
1. **Strict Hardware Isolation:** No vendor hardware APIs (CTRE Phoenix, REVLib, etc.) are allowed in standard subsystem files. They must live strictly inside `*IOReal.java` implementations.
   - **Sanctioned vendor exception (decided 2026-07-16):** `subsystems/CommandSwerveDrivetrain.java` and `utility/simulation/MapleSimSwerveDrivetrain.java` are the **singular** sanctioned exception to this rule. They are CTRE Tuner X / swerve-generator output and stay vendor-inline rather than wrapped in a `DriveIO` — regenerating them from Tuner X would otherwise conflict with a hand-maintained IO wrapper. Every other file must stay vendor-free.
   - **Utility layer is NOT exempt.** `utility/HubActiveState.java` used to import `com.ctre.phoenix6` (`HootAutoReplay`, `Utils`) directly for Hoot auto-log/auto-replay. That hookup is now isolated in `utility/HootReplayBridge.java` (package-private, the only file in `utility/` that touches Phoenix 6); `HubActiveState.java` itself is vendor-free.
2. **Singleton Subsystems:** Every subsystem and the `Superstructure` must implement the Singleton pattern (`private` constructor and `public static Subsystem getInstance()`).
3. **Centralized States:** The `Superstructure` coordinates complex multi-subsystem states via a `SuperstructureState` enum (`OFF`, `INTAKING`, `STOWED`, `ALIGNING`, `SHOOTING`) and a `periodic()` switch-case that commands Shooter/Intake/Vision directly. `shootCmd()`/`shootingSequence(double)` are thin Command bridges kept only because RobotContainer's `whileTrue` binding and PathPlanner's NamedCommands both require real Command objects — the actual arbitration logic lives in `periodic()`, not in composed Commands. Subsystems themselves still only handle their own immediate mechanism control (e.g. `Intake.agitatePivot()` stays owned by Intake; Superstructure schedules/cancels it as a unit rather than reimplementing its oscillation).

## 🧠 Karpathy Guidelines (Anti-Failure Mode Protocol)
To ensure elite execution, strictly adhere to these behaviors:
- **No Assumptions:** Never guess vendor API syntax or class constructor shapes (especially Phoenix 6 vs 5). Stop and use `context7` or web search to verify the real-world signature first.
- **Simplicity First (Anti-Overengineering):** Write the cleanest, most minimalist code required to satisfy the immediate structural requirement or simulation goal.
- **Surgical Changes Only:** Modify *only* the specific files related to the active task. Do not rewrite, clean up, or change formatting in adjacent methods or unrelated classes.
- **Never Declare Success Early:** A task is not complete until you explicitly run `./gradlew compileJava` via the shell tool and confirm a zero-error output. If it breaks, fix it immediately.
- **Don't Invent Files:** If the user references a class that does not exist in `src/` (e.g. `FuelSim`), say so and stop. Do not create it from a reference-directory copy, and do not edit files under `temp_reference/`.

## 🕒 Current Task State
*Verified against the tree on 2026-07-16 at commit `39c7fee` + uncommitted working-tree changes (Vision/Superstructure/HubActiveState refactor, PathPlanner sync — not yet committed). `./gradlew compileJava` exits 0, 0 warnings.*

### Done
- **Intake refactor:** complete. `subsystems/intake/` holds `IntakeIO.java`, `IntakeIOReal.java`, `IntakeIOSim.java`, `Intake.java` (Singleton via `getInstance()`, zero vendor imports).
- **Shooter refactor:** complete. `subsystems/shooter/` holds `ShooterIO.java`, `ShooterIOReal.java`, `ShooterIOSim.java`, `Shooter.java` (Singleton via `getInstance()`, zero vendor imports).
- **Vision/Limelight isolation — complete end-to-end (Phase 2 + Phase 3, 2026-07-16):** `subsystems/vision/` holds `VisionIO.java`, `VisionIOReal.java`, `VisionIOSim.java`, `Vision.java` (Singleton via `getInstance(drivetrain)`). `LimelightHelpers` is imported only by `VisionIOReal.java`. Phase 3 closed the last leak: `VisionIO` gained `setIMUMode(int)`/`setIMUAssistAlpha(double)`; `Robot.java` now goes through `Vision.getInstance(...)` exclusively (no `LimelightHelpers` import anywhere outside `VisionIOReal.java`).
- **Superstructure — now a real 254-style state machine (2026-07-16):** Singleton via `getInstance(drivetrain)`. `SuperstructureState` enum (`OFF`/`INTAKING`/`STOWED`/`ALIGNING`/`SHOOTING`) + `periodic()` switch-case arbitration commands Shooter/Intake/Vision directly each tick (RPM targeting, indexer/agitator control, align→shoot phase transitions with teleop-vs-auto timing). `shootCmd()`/`shootingSequence(double)` are thin Command bridges (`requestShoot()`/`requestStow()` underneath) kept only because RobotContainer's `whileTrue` binding and PathPlanner's NamedCommands both require real Command objects — no change needed in `RobotContainer.java`.
- **Utility vendor isolation (2026-07-16):** `utility/HubActiveState.java` no longer imports `com.ctre.phoenix6` directly. The Hoot auto-log/auto-replay hookup (`HootAutoReplay`, `Utils`) is isolated in new `utility/HootReplayBridge.java` (package-private, the sole vendor-touching file in `utility/`).
- **`CommandSwerveDrivetrain` vendor-exception decision (2026-07-16):** recorded in Architecture Rule 1 — `CommandSwerveDrivetrain.java` / `MapleSimSwerveDrivetrain.java` are the singular sanctioned Phoenix 6 exception, left un-wrapped.
- **Deprecation & hygiene fixes (2026-07-16):** `AprilTagFields.loadAprilTagLayoutField()` replaced with `AprilTagFieldLayout.loadField(AprilTagFields)` at `FieldConstants.java:20` and `Vision.java:35`. Stale `@Logged` (Epilogue) annotation removed from `Robot.java`. The 3 dead `SmartDashboard` writes in `RobotContainer.java` moved into a new `RobotContainer.periodic()` called from `Robot.robotPeriodic()`, so they publish live instead of once at boot. `ShooterIOSim`'s agitator now uses `DCMotor.getNeo550(1)` (confirmed real hardware via `ShooterIOReal`'s `MotorArrangementValue.NEO550_JST`) instead of the `DCMotor.getNEO()` placeholder — the factory method exists in this WPILib version; the prior code comment claiming otherwise was wrong.
- **PathPlanner `settings.json` partially synced to code (2026-07-16):** `robotTrackwidth`/`driveWheelRadius`/`driveGearing`/`maxDriveSpeed`/`driveCurrentLimit` and all 8 module X/Y fields now match `TunerConstants.java` (the file that actually drives the real swerve hardware, not a placeholder). `robotMass`/`robotMOI`/`wheelCOF`/`robotWidth`/`robotLength`/bumper offsets are untouched — no code-side ground truth exists for them (`Constants.java`'s equivalents are themselves unmeasured `TODO` placeholders); still needs a real-robot measurement pass, see Backlog.
- **Sim field/alliance:** `FieldConstants.java` derives field parameters from the `k2026RebuiltAndymark` AprilTag layout and exposes `middleStartFor(Alliance)`. Never hard-code a field length — `16.51` is the 2022–24 field and is wrong for 2026. Gamepiece sim is maple-sim's `SimulatedArena`. **There is no custom project `FuelSim` class**; all simulator adjustments rely on the baseline maple-sim architecture. (The only `FuelSim.java` on this machine is 6328's, under reference dirs.)

### Next up
- Commit this session's working-tree changes (9 files + new `HootReplayBridge.java`) — currently uncommitted.
- Real-robot measurement pass for `Constants.java`/`settings.json` placeholders (see Backlog).

### Backlog
- **Static Calibrations Needed:**
  - Correct the matching front-left / back-right CANcoder offsets (both `0.066650390625` — almost certainly a calibration paste; re-zero BR in Tuner X and compare). *Unverified hypothesis — confirm on the real robot before editing.*
  - Measure the real robot's mass, MOI, wheel-COF, and bumper footprint to replace the remaining `TODO` placeholders in `Constants.java` and `settings.json` (`robotMass`, `robotMOI`, `wheelCOF`, `robotWidth`/`robotLength`, bumper offsets).
  - Audit the high flywheel feedforward `kA = 50` against `kV = 0.215` (~230×; not a plausible flywheel value). *Unverified.*
  - Confirm on the real robot whether Intake pivot/roller and Shooter shoot/index are Kraken X60 or Falcon 500 (both real motors are plain `TalonFX` in `*IOReal.java`, so it's unconfirmable from code) — updates `IntakeIOSim.java`/`ShooterIOSim.java`.

Full detail and evidence for every item above lives in the vault: `Best Practices & Upgrades.md`.

## 📝 The `/tldr` Exit Protocol (End of Session Action)
When the user invokes `/tldr`, immediately:
1. Update the **Current Task State** section of this local `CLAUDE.md` with what was completed during the session.
2. Compile a structured summary of today's code changes, dynamic fixes, and peer-benchmarked discoveries.
3. Write that summary as a new markdown note in `C:\Users\xdm\6767 frc bible\Projects\`, ensuring all newly established parameters or hardware quirks are synced back to the central master brain.

See `.claude/commands/tldr.md` for the full command definition.
