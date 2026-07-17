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
2. **Singleton Subsystems:** Every subsystem and the `Superstructure` must implement the Singleton pattern (`private` constructor and `public static Subsystem getInstance()`).
3. **Centralized States:** The `Superstructure` coordinates complex multi-subsystem states. Subsystems themselves only handle their immediate mechanism control.

## 🧠 Karpathy Guidelines (Anti-Failure Mode Protocol)
To ensure elite execution, strictly adhere to these behaviors:
- **No Assumptions:** Never guess vendor API syntax or class constructor shapes (especially Phoenix 6 vs 5). Stop and use `context7` or web search to verify the real-world signature first.
- **Simplicity First (Anti-Overengineering):** Write the cleanest, most minimalist code required to satisfy the immediate structural requirement or simulation goal.
- **Surgical Changes Only:** Modify *only* the specific files related to the active task. Do not rewrite, clean up, or change formatting in adjacent methods or unrelated classes.
- **Never Declare Success Early:** A task is not complete until you explicitly run `./gradlew compileJava` via the shell tool and confirm a zero-error output. If it breaks, fix it immediately.
- **Don't Invent Files:** If the user references a class that does not exist in `src/` (e.g. `FuelSim`), say so and stop. Do not create it from a reference-directory copy, and do not edit files under `temp_reference/`.

## 🕒 Current Task State
*Verified against the tree on 2026-07-16 at commit `d32388a`. `./gradlew compileJava` exits 0 (2 deprecation warnings).*

### Done
- **Intake refactor:** complete. `subsystems/intake/` holds `IntakeIO.java`, `IntakeIOReal.java`, `IntakeIOSim.java`, `Intake.java` (Singleton via `getInstance()`, zero vendor imports).
- **Shooter refactor:** complete. `subsystems/shooter/` holds `ShooterIO.java`, `ShooterIOReal.java`, `ShooterIOSim.java`, `Shooter.java` (Singleton via `getInstance()`, zero vendor imports).
- **Phase 2 — Vision/Limelight isolation:** complete in the subsystem layer. `subsystems/vision/` holds `VisionIO.java`, `VisionIOReal.java`, `VisionIOSim.java`, `Vision.java` (Singleton via `getInstance(drivetrain)`). `LimelightHelpers` is imported only by `VisionIOReal.java`. One leak remains outside the subsystem — see Phase 3 below.
- **Superstructure:** exists as a Singleton (`getInstance(drivetrain)`) and owns `shootCmd()` + `shootingSequence(timeout)`, composing Shooter + Intake + Vision.
- **Sim field/alliance:** `FieldConstants.java` derives field parameters from the `k2026RebuiltAndymark` AprilTag layout and exposes `middleStartFor(Alliance)`. Never hard-code a field length — `16.51` is the 2022–24 field and is wrong for 2026. Gamepiece sim is maple-sim's `SimulatedArena`. **There is no custom project `FuelSim` class**; all simulator adjustments rely on the baseline maple-sim architecture. (The only `FuelSim.java` on this machine is 6328's, under reference dirs.)

### Next up — Phase 3: finish hardware isolation
1. **`Robot.java` calls `LimelightHelpers` directly** (lines ~109, ~135–136, ~144–146: `SetIMUMode`, `SetIMUAssistAlpha`, `getBotPoseEstimate_wpiBlue`). This violates Architecture Rule 1. Push these behind `VisionIO` (e.g. `setIMUMode(int)` / an `Optional<Pose2d>` seed accessor) and have `Robot` go through `Vision`.
2. **`utility/HubActiveState.java` imports `com.ctre.phoenix6`** (`HootAutoReplay`, `Utils`). Decide whether the utility layer is exempt from Rule 1, or wrap it. **Record the decision in this file either way.**
3. **`subsystems/CommandSwerveDrivetrain.java` (613 LOC) is CTRE-generated and uses Phoenix 6 inline**, as does `utility/simulation/MapleSimSwerveDrivetrain.java`. This is the largest open exception to Rule 1. Decide explicitly: leave it as a sanctioned vendor-generated exception, or wrap it in a `DriveIO`. **Do not start this without a decision — it is the biggest single item left.**

### Backlog
- **Superstructure is command composition, not a state machine.** The project goal calls for 254-style centralized states; today there is no state enum and no `periodic()` arbitration. Revisit once Phase 3 settles.
- **Deprecation:** `AprilTagFields.loadAprilTagLayoutField()` is deprecated and marked for removal — 2 call sites, `FieldConstants.java:20` and `Vision.java:35`.
- **Static Calibrations Needed:**
  - Correct the matching front-left / back-right CANcoder offsets (both `0.066650390625` — almost certainly a calibration paste; re-zero BR in Tuner X and compare). *Unverified hypothesis — confirm on the real robot before editing.*
  - Calibrate PathPlanner's robot config to match actual code metrics (track width **0.8128 m** actual vs **0.546 m** in `settings.json`; 9 of 9 physical quantities disagree). **Measure the real robot first** — the code-side values are themselves unvalidated `TODO` estimates, so don't assume either side is right.
  - Clear the stale `@Logged` annotation on `Robot.java:26` — Epilogue is never bound and has no Gradle plugin, so it is inert.
  - Fix or delete the 3 dead dashboard writes in `RobotContainer.java:112-114` (`dashboard()` runs once from the constructor, so they publish 0.0 forever).
  - Audit the high flywheel feedforward `kA = 50` against `kV = 0.215` (~230×; not a plausible flywheel value). *Unverified.*
- **Sim placeholder constants:** masses, lengths, MOIs, `kWheelCOF`, bumper dims in `Constants.java` are estimates marked `TODO`. `IntakeIOSim`/`ShooterIOSim` assume Kraken X60; the Shooter agitator sim uses `DCMotor.getNEO()` for a real NEO 550.

Full detail and evidence for every item above lives in the vault: `Best Practices & Upgrades.md`.

## 📝 The `/tldr` Exit Protocol (End of Session Action)
When the user invokes `/tldr`, immediately:
1. Update the **Current Task State** section of this local `CLAUDE.md` with what was completed during the session.
2. Compile a structured summary of today's code changes, dynamic fixes, and peer-benchmarked discoveries.
3. Write that summary as a new markdown note in `C:\Users\xdm\6767 frc bible\Projects\`, ensuring all newly established parameters or hardware quirks are synced back to the central master brain.

See `.claude/commands/tldr.md` for the full command definition.
