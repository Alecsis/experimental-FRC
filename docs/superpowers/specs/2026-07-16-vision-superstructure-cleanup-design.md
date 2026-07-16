# Vision/Superstructure Cleanup Design

Date: 2026-07-16
Status: Approved

## Context

Phase 2 (Vision/Limelight IO isolation) is complete: `subsystems/vision/{VisionIO,VisionIOReal,VisionIOSim,Vision}.java` exist, `Vision` is a Singleton that owns MegaTag2 fusion and pushes measurements directly to `CommandSwerveDrivetrain`.

Before Phase 3 (simulation + Superstructure work) begins, this pass consolidates command/helper sprawl that's accumulated in `commands/` and `subsystems/`, and pulls the Superstructure singleton forward by one step so multi-subsystem orchestration lands in its permanent home instead of being written into `Shooter.java` and then immediately rewritten.

This does not touch `Intake`, `Shooter`, or `Vision`'s IO layers, hardware behavior, or the Phase 2 fusion logic. It is a structural consolidation only.

## Section 1 — Vision.java absorbs PoseHelpers + RoboMath

**Problem:** `PoseHelpers.java` and `Vision.java` both compute "where is the hub," two different ways:
- `PoseHelpers.getHubPosition()`: averages live `AprilTagFieldLayout` tag poses (`RedHubIds`/`BlueHubIds`) for the current alliance.
- `Vision.hubTranslation()`: hardcoded literal `Translation2d` constants per alliance.

**Change:**
- Delete `Vision.hubTranslation()`'s hardcoded constants. `Vision.hubDistanceMeters()` and all hub-tracking call sites use the AprilTag-layout-averaged position instead.
- Move into `Vision.java`, adapted to use the existing `drivetrain` field instead of a passed-in pose:
  - `AprilTagFieldLayout fieldLayout` + `RedHubIds`/`BlueHubIds`/`RedPassIds`/`BluePassIds`
  - `getHubPosition()` -> `Optional<Translation2d>`
  - `getHubHeading()` -> `Optional<Rotation2d>`
  - `getPassTargetPosition()` -> `Optional<Translation2d>`
  - `getPassHeading()` -> `Optional<Rotation2d>`
  - `getDistanceToHub()` folds into the existing `hubDistanceMeters()` (now sourced from `getHubPosition()` instead of the deleted hardcoded constants)
- Move `RoboMath.calculateRPMFromVision(Vision, Pose2d)` + `interpolate()` + the RPM lookup table into `Vision.java` as `Vision.calculateRPM()` (no-arg — Vision already holds `drivetrain`, no need for an externally supplied pose).
- Delete `subsystems/PoseHelpers.java` and `utility/RoboMath.java`.

**Consumers to rewire:** `RobotContainer` (Orbit NamedCommand, rightBumper/rightTrigger bindings — see Section 4/5), `Superstructure.shootCmd()`/`shootingSequence()` (RPM lookups — see Section 3).

## Section 2 — Intake command factories

**Finding:** `Intake_Stop.java`'s entire behavior (`Commands.runOnce(() -> setRoller(Roller.STOP))`) is already implemented, unused, as `Intake.stopRoller()`. `Intake_Start.java` (start/end roller-intake) has no existing equivalent but matches the shape of `Intake.eject()`.

**Change:**
- Add `Intake.startRoller()`:
  ```java
  public Command startRoller() {
    return Commands.startEnd(
        () -> setRoller(Roller.INTAKE),
        () -> setRoller(Roller.STOP),
        this);
  }
  ```
- Delete `commands/Intake_Start.java` and `commands/Intake_Stop.java`.
- `RobotContainer`: remove `cmd_Intake_Start` (currently dead — declared, never bound) and `cmd_Intake_Stop` fields. Repoint `NamedCommands.registerCommand("Intake Stop", ...)` to `intake.stopRoller()`.

## Section 3 — New Superstructure.java (pulled forward from Phase 3)

**Problem:** `ShootCmd` and `Shooting_Sequence` orchestrate Shooter + Intake + Vision + drivetrain state together with branching and timed sequencing — the exact shape CLAUDE.md's "Centralized States" rule reserves for a `Superstructure`, not a subsystem file.

**Change:** create `subsystems/Superstructure.java`, shaped after the `temp_reference/Lynk 2026/superstructure/Superstructure.java` precedent (structural reference only, not imported/merged):

```java
public class Superstructure extends SubsystemBase {
  private static Superstructure instance;

  public static Superstructure getInstance(CommandSwerveDrivetrain drivetrain) {
    if (instance == null) {
      instance = new Superstructure(drivetrain);
    }
    return instance;
  }

  private final Shooter shooter = Shooter.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Vision vision;
  private final CommandSwerveDrivetrain drivetrain;

  private Superstructure(CommandSwerveDrivetrain drivetrain) {
    this.vision = Vision.getInstance(drivetrain);
    this.drivetrain = drivetrain;
  }

  public Command shootCmd() { ... }                    // ShootCmd's body, ported
  public Command shootingSequence(double timeoutSeconds) { ... }  // Shooting_Sequence's body, ported
}
```

- `shootCmd()` and `shootingSequence()` build a fresh `Commands.parallel(...)`/`Commands.sequence(...)` graph on every call (matching `Intake.intake()`/`Shooter.agitate()`'s existing factory convention), rather than the current pattern of constructing one `Shooting_Sequence` instance in `RobotContainer` and reusing it across a button binding and two `NamedCommands.registerCommand` calls. Reusing a single live Command instance across concurrent bindings is a latent bug; building fresh per call removes it.
- Internal RPM lookups switch from `RoboMath.calculateRPMFromVision(vision, swerve.getState().Pose)` to `vision.calculateRPM()` (Section 1).
- Delete `commands/ShootCmd.java` and `commands/Shooting_Sequence.java`.

## Section 4 — CommandSwerveDrivetrain absorbs AutoAlignPOI + Target

**AutoAlignPOI** doesn't depend on Vision (only the static `POI` enum) -> becomes a drivetrain factory:
```java
public Command driveToPOI(POI poi) { ... }  // FunctionalCommand, ported from AutoAlignPOI's execute/isFinished/end
```

**Target** is generic (`Supplier<Optional<Translation2d>>`) but all 3 existing call sites pass either `poseHelpers::getHubPosition` or `poseHelpers::getPassTargetPosition` — never anything else. Collapse the unused genericity into two named factories taking `Vision` directly:
```java
public Command trackHub(Vision vision, double maxSpeed, DoubleSupplier x, DoubleSupplier y, boolean finishOnAlign) { ... }
public Command trackPassTarget(Vision vision, double maxSpeed, DoubleSupplier x, DoubleSupplier y, boolean finishOnAlign) { ... }
```
Both built with `FunctionalCommand` inline, porting `Target`'s `initialize`/`execute`/`isFinished`/`end` logic unchanged (including the `SmartDashboard.getNumber("offset", 0)` heading offset read).

Delete `commands/AutoAlignPOI.java` and `commands/Target.java`.

## Section 5 — RobotContainer cleanup

- Remove imports/fields: `AutoAlignPOI`, `Intake_Start`, `Intake_Stop`, `ShootCmd`, `Target`, `PoseHelpers`, `RoboMath`, `cmd_Intake_Start`, `cmd_Intake_Stop`, `cmd_Normal_Shooting`, `cmd_Quick_Shooting`, `cmd_ShootCmd`, `aPOI` (dead today), `poseHelpers`.
- Add `private final Superstructure superstructure = Superstructure.getInstance(drivetrain);`.
- Rewire call sites:
  - `povRight/povLeft/x/y/b` bindings -> `drivetrain.driveToPOI(POI.___)`
  - `rightBumper`/`rightTrigger` bindings -> `drivetrain.trackHub(vision, ...)` / `drivetrain.trackPassTarget(vision, ...)`
  - `controlBox.button(2)` -> `superstructure.shootCmd()`
  - `NamedCommands.registerCommand("Shooting Sequence"/"Quick Shooting", ...)` -> `superstructure.shootingSequence(5.0)` / `superstructure.shootingSequence(3.0)`
  - `NamedCommands.registerCommand("Orbit", ...)` -> `drivetrain.trackHub(vision, 0, () -> 0, () -> 0, true)`
  - `NamedCommands.registerCommand("Intake Stop", ...)` -> `intake.stopRoller()`

## Out of scope

- No changes to `Intake`/`Shooter`/`Vision` IO layers, hardware behavior, or Phase 2 fusion logic.
- No new tests (none exist in this project today).
- No further Phase 3 Superstructure state-machine work (shot configs, superstates, etc.) beyond hosting `shootCmd()`/`shootingSequence()`.

## Verification

`./gradlew compileJava` must succeed with zero errors before this is considered done, per CLAUDE.md's verification rule.
