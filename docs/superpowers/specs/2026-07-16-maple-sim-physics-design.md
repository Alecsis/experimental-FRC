# Maple-Sim Physics Integration Design

Date: 2026-07-16
Status: Approved

## Context

Phase 2 (Vision isolation) and the subsequent Vision/Superstructure cleanup pass are complete and committed on `refactor/hybrid`. This is Phase 3: replace the drivetrain's default (physics-free) WPILib simulation loop with `org.ironmaple:maplesim-java` (maple-sim), a real rigid-body physics engine for FRC sim, and give `IntakeIOSim` a real simulated game-piece-contact signal instead of no signal at all.

Game confirmed as 2026 "Rebuilt" (matches `AprilTagFields.k2026RebuiltAndymark` already used in `Vision.java`); its maple-sim game piece type string is `"Fuel"`.

All vendor API surfaces referenced below were verified via Context7 against `/shenzhen-robotics-alliance/maple-sim` (high source reputation) before being written into this spec, per CLAUDE.md's "No Assumptions" rule. The one unverified variable is whether maple-sim `0.3.9` (pinned to `frcYear: "2025"` in the only vendordep JSON available locally, from Team 254's 2025 codebase) resolves and links cleanly against this project's WPILib 2026.2.1 / GradleRIO 2026.2.1. `./gradlew compileJava` is the actual test of that; no newer version could be confirmed without web search, which was declined this session.

## Section 1 — `vendordeps/maple-sim.json`

Add the vendordep JSON (not a hand-typed `build.gradle` dependency), mirroring the existing `AdvantageKit.json`/`Phoenix6-26.1.3.json`/`PathplannerLib-2026.1.2.json`/`WPILibNewCommands.json` pattern already in this project's `vendordeps/`. `wpi.java.vendor.java()` (`build.gradle:66`) already auto-includes every vendordep JSON's `javaDependencies` and `mavenUrls` — no `build.gradle` edit needed beyond this file drop.

Content is the exact file read from `temp_reference/Team 254 Code/vendordeps/maple-sim.json` (`org.ironmaple:maplesim-java:0.3.9`, `org.dyn4j:dyn4j:5.0.2`, both real Maven coordinates from a working project).

## Section 2 — `frc.robot.utility.simulation.MapleSimSwerveDrivetrain`

New file, package `frc.robot.utility.simulation` (parallel to `frc.robot.utility.LimelightHelpers` — this project's existing convention for "vendor integration helper, not itself a subsystem"). Adapted from `temp_reference/Team 254 Code/.../MapleSimSwerveDrivetrain.java` (structural reference per CLAUDE.md — rewritten for this package, not copy-merged).

Public surface:
- Constructor: `(Time simPeriod, Mass robotMassWithBumpers, Distance bumperLengthX, Distance bumperWidthY, DCMotor driveMotorModel, DCMotor steerMotorModel, double wheelCOF, Translation2d[] moduleLocations, Pigeon2 pigeon, SwerveModule<TalonFX,TalonFX,CANcoder>[] modules, SwerveModuleConstants<TalonFXConfiguration,TalonFXConfiguration,CANcoderConfiguration>... moduleConstants)` — builds a `DriveTrainSimulationConfig`, constructs `mapleSimDrive` (`SwerveDriveSimulation`), wires each module's drive/steer `SimulatedMotorController` to the real `TalonFXSimState`/`CANcoderSimState`, registers with `SimulatedArena.getInstance()`.
- `public final SwerveDriveSimulation mapleSimDrive` — public field, exposed so `CommandSwerveDrivetrain` can read/reset it and so `IntakeIOSim` can reach it transitively.
- `public void update()` — runs `SimulatedArena.getInstance().simulationPeriodic()`, pushes the resulting pose/yaw into the `Pigeon2SimState`.
- `public static SwerveModuleConstants<?,?,?>[] regulateModuleConstantsForSimulation(SwerveModuleConstants<?,?,?>[])` — static, mutates+returns the array with sim-safe overrides (zeroed encoder offsets, disabled inversions, softened steer PID); internally no-ops via `RobotBase.isReal()` check, so it's safe to call unconditionally on both real and sim builds.

## Section 3 — `CommandSwerveDrivetrain.java`

- All 3 existing constructors wrap their `modules` vararg in `MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules)` before calling `super(...)`.
- New field `private MapleSimSwerveDrivetrain mapleSim;` (null on real hardware).
- `startSimThread()`: after `super(...)` completes (so `getModules()`/`getPigeon2()` exist), construct `mapleSim` using `TunerConstants`-derived values (module locations from each `SwerveModuleConstants.LocationX/Y`, `getPigeon2()`, `getModules()`, drive/steer `DCMotor` models, and new `Constants` physical placeholders — see Section 5). The `Notifier`'s periodic body switches from `updateSimState(deltaTime, RobotController.getBatteryVoltage())` to `mapleSim.update()`.
- New `@Override public void resetPose(Pose2d pose)`: `super.resetPose(pose);` then, if `mapleSim != null`, `mapleSim.mapleSimDrive.setSimulationWorldPose(pose);` — keeps the physics world's ground-truth pose in sync whenever odometry is externally reset (matches this project's existing `Vision.getPoseResetEstimate()` reset path and PathPlanner's `resetPose` autobuilder consumer).
- New `public AbstractDriveTrainSimulation getMapleSimDrive()`: returns `mapleSim == null ? null : mapleSim.mapleSimDrive`. `IntakeIOSim` reaches this via `RobotContainer.drivetrain.getMapleSimDrive()` — the same cross-reference style `CommandSwerveDrivetrain.java` itself already uses (`import frc.robot.RobotContainer;`, present in the file today), not a new architectural pattern.

## Section 4 — `IntakeIOSim.java`

- New field `private IntakeSimulation intakeSimulation;`, lazily constructed on first `updateInputs()` call (drivetrain's `mapleSim` isn't guaranteed constructed at `IntakeIOSim`'s own construction time, since `Intake.getInstance()` may run before `RobotContainer.drivetrain`'s sim thread starts — lazy init sidesteps ordering entirely):
  ```java
  if (intakeSimulation == null && RobotContainer.drivetrain.getMapleSimDrive() != null) {
    intakeSimulation = IntakeSimulation.InTheFrameIntake(
        "Fuel", RobotContainer.drivetrain.getMapleSimDrive(),
        Constants.kIntakeSimWidthMeters, Constants.kIntakeSimSide, Constants.kIntakeSimCapacity);
  }
  ```
- Intake "on" state drives `startIntake()`/`stopIntake()`: called whenever `setRollerVelocity(...)` is invoked with a positive (intake-direction) setpoint vs. `setRollerVoltage(0)`/negative (eject). Concretely: track a `rollerIntaking` boolean set in `setRollerVelocity`/`setRollerVoltage`, call `intakeSimulation.startIntake()`/`stopIntake()` from `updateInputs()` when that boolean's state has changed since the last call.
- `IntakeIOInputs` (in `IntakeIO.java`) gains one new field: `public boolean hasGamePiece = false;`, set in `updateInputs()` from `intakeSimulation != null && intakeSimulation.getGamePiecesAmount() > 0`.
- `IntakeIOReal.java` is untouched — `hasGamePiece` simply stays `false` there for now (no real beam-break/sensor exists yet; out of scope for this pass, matches "surgical changes only").

## Section 5 — `Constants.java` additions

New TODO-marked placeholder section (matching the existing Intake/Shooter sim-constants convention exactly):

```java
// Drivetrain simulation (maple-sim) physical constants (placeholder estimates -- TODO: measure/tune against real robot)
public static final double kRobotMassWithBumpersKg = 55.0; // TODO: weigh the real robot with bumpers
public static final double kBumperLengthXMeters = 0.9; // TODO: measure real bumper footprint
public static final double kBumperWidthYMeters = 0.9; // TODO: measure real bumper footprint
public static final double kWheelCOF = 1.2; // TODO: tune -- 1.2 is a typical rubber-tread-on-carpet estimate

// Intake simulation (maple-sim) physical constants (placeholder estimates -- TODO: measure/tune against real robot)
public static final double kIntakeSimWidthMeters = 0.7; // TODO: measure real intake width
public static final IntakeSimulation.IntakeSide kIntakeSimSide = IntakeSimulation.IntakeSide.FRONT; // TODO: confirm real mounting side
public static final int kIntakeSimCapacity = 1; // TODO: confirm real max held-piece count
```

## Out of scope

- `IntakeIOReal.java` sensor wiring for real game-piece detection (no hardware sensor decided yet).
- Shooter-side game-piece ejection/scoring simulation (shooting a "Fuel" piece back out into the field simulation) — this pass only wires intake collection, not scoring feedback.
- Overriding `SimulatedArena` to a specific `Rebuilt`-season arena subclass — relying on `SimulatedArena.getInstance()`'s documented "defaults to current season" behavior instead, since the exact `Arena<Season>` class name for 2026 wasn't confirmed and isn't required for this scope.

## Verification

`./gradlew compileJava` must succeed with zero errors. If maple-sim `0.3.9` fails to resolve or link against WPILib 2026.2.1, stop and report the exact failure rather than guessing a replacement version.
