# Vision/Superstructure Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Consolidate `PoseHelpers`/`RoboMath` into `Vision`, convert `Intake_Start`/`Intake_Stop`/`AutoAlignPOI`/`Target` into subsystem command factories, and create `Superstructure` to host `ShootCmd`/`Shooting_Sequence`'s multi-subsystem orchestration, per the approved design at `docs/superpowers/specs/2026-07-16-vision-superstructure-cleanup-design.md`.

**Architecture:** Pure structural consolidation — move existing, working logic to its correct home (single-subsystem logic into that subsystem, multi-subsystem orchestration into a new `Superstructure` Singleton) and delete the now-redundant `commands/` files. No hardware behavior, IO layer, or Phase 2 vision-fusion logic changes.

**Tech Stack:** Java 17, WPILib 2026, AdvantageKit (`Logger`), CTRE Phoenix 6 swerve generated code, PathPlanner (`NamedCommands`).

## Global Constraints

- **No unit tests exist in this project.** The only verification gate is `./gradlew compileJava` (per `CLAUDE.md`'s "Never Declare Success Early" rule) — every task below substitutes a compile check where the skill template would otherwise call for a test run.
- **Task ordering is fixed by user instruction**: Sections 1-5 execute in the exact order below. `RobotContainer.java` (Task 5) is the only file that wires everything together, so **Tasks 1-4 will each leave the project in a non-compiling state** (old command classes get deleted while `RobotContainer.java` still references them until Task 5). This is expected — do not treat mid-plan compile failures as bugs to fix; the real gate is Task 6, after Task 5 completes.
- **Singleton pattern required**: every subsystem and `Superstructure` needs `private` constructor + `public static X getInstance(...)`, per `CLAUDE.md`.
- **No vendor hardware APIs outside `*IOReal.java` files** — none of this plan's changes touch IO layers, so this constraint isn't exercised here, just don't reintroduce a vendor import anywhere in this pass.
- Preserve all ported logic byte-for-byte in behavior (same magic numbers, same control flow) unless a task explicitly calls out an intentional behavior change (there are exactly two: the hub-position dedup in Task 1, and fresh-command-graph-per-call in Task 3 — both already approved in the design doc).

---

### Task 1: Vision.java absorbs PoseHelpers + RoboMath

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/vision/Vision.java`
- Delete: `src/main/java/frc/robot/subsystems/PoseHelpers.java`
- Delete: `src/main/java/frc/robot/utility/RoboMath.java`

**Interfaces:**
- Produces (used by Task 3 and Task 4/5): `Vision.getHubPosition()` -> `Optional<Translation2d>`, `Vision.getPassTargetPosition()` -> `Optional<Translation2d>`, `Vision.calculateRPM()` -> `OptionalDouble`.
- `Vision.hubTranslation()` is **removed** — no other task or file may reference it after this task.

- [ ] **Step 1: Add AprilTag-layout fields and imports to Vision.java**

In `src/main/java/frc/robot/subsystems/vision/Vision.java`, replace the import block:

```java
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
```

with:

```java
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.TreeMap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
```

(Drops the now-unused `static edu.wpi.first.units.Units.Meters` import — only `hubTranslation()` used it, and that method is deleted in Step 3.)

- [ ] **Step 2: Add field-layout constants after the `kMaxOmegaRadPerSec` field**

Insert after `private static final double kMaxOmegaRadPerSec = 2 * Math.PI;`:

```java

  private final AprilTagFieldLayout fieldLayout =
      AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();
  private final int[] redHubIds = { 2, 5, 8, 9, 10, 11 };
  private final int[] blueHubIds = { 18, 21, 24, 25, 26, 27 };
  private final int[] redPassIds = { 9, 10, 15, 16 };
  private final int[] bluePassIds = { 25, 26, 31, 32 };

  private static final TreeMap<Double, Double> RPMLUT = new TreeMap<>();
  static {
    RPMLUT.put(12.4, 1625.0);
    RPMLUT.put(8.8, 1475.0);
    RPMLUT.put(14.5, 2150.0);
    RPMLUT.put(10.6, 1550.0);
    RPMLUT.put(7.5, 1500.0);
    RPMLUT.put(11.3, 1650.0);
  }
```

- [ ] **Step 3: Replace `hubTranslation()` and `hubDistanceMeters()` with the AprilTag-layout-averaged versions**

Replace this exact block:

```java
  public Translation2d hubTranslation() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return new Translation2d(Meters.of(11.915394), Meters.of(4.0132));
    } else {
      return new Translation2d(Meters.of(4.625594), Meters.of(4.0132));
    }
  }
```

with:

```java
  public Optional<Translation2d> getHubPosition() {
    DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
    int[] validIds = alliance == DriverStation.Alliance.Red ? redHubIds : blueHubIds;

    double totalX = 0;
    double totalY = 0;
    int count = 0;
    for (int id : validIds) {
      var pose = fieldLayout.getTagPose(id);
      if (pose.isPresent()) {
        Translation2d t = pose.get().toPose2d().getTranslation();
        totalX += t.getX();
        totalY += t.getY();
        count++;
      }
    }

    if (count == 0) {
      return Optional.empty();
    }
    return Optional.of(new Translation2d(totalX / count, totalY / count));
  }

  public Optional<Rotation2d> getHubHeading() {
    Optional<Translation2d> hubPosOpt = getHubPosition();
    if (hubPosOpt.isEmpty()) {
      return Optional.empty();
    }
    Translation2d delta = hubPosOpt.get().minus(drivetrain.getState().Pose.getTranslation());
    return Optional.of(delta.getAngle());
  }

  public Optional<Translation2d> getPassTargetPosition() {
    DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
    int[] validIds = alliance == DriverStation.Alliance.Red ? redPassIds : bluePassIds;

    Translation2d sum = new Translation2d();
    int count = 0;
    for (int id : validIds) {
      var pose = fieldLayout.getTagPose(id);
      if (pose.isPresent()) {
        sum = sum.plus(pose.get().toPose2d().getTranslation());
        count++;
      }
    }

    if (count == 0) {
      return Optional.empty();
    }
    return Optional.of(sum.div(count));
  }

  public Optional<Rotation2d> getPassHeading() {
    Optional<Translation2d> targetOpt = getPassTargetPosition();
    if (targetOpt.isEmpty()) {
      return Optional.empty();
    }
    Translation2d delta = targetOpt.get().minus(drivetrain.getState().Pose.getTranslation());
    return Optional.of(delta.getAngle());
  }
```

Then replace this exact block:

```java
  public OptionalDouble hubDistanceMeters(Pose2d currentRobotPose) {
    var poseOpt = getVisionPose(currentRobotPose);
    if (poseOpt.isEmpty()) {
      return OptionalDouble.empty();
    }
    Translation2d hub = hubTranslation();
    return OptionalDouble.of(poseOpt.get().getTranslation().getDistance(hub));
  }
```

with:

```java
  public OptionalDouble hubDistanceMeters(Pose2d currentRobotPose) {
    var poseOpt = getVisionPose(currentRobotPose);
    var hubOpt = getHubPosition();
    if (poseOpt.isEmpty() || hubOpt.isEmpty()) {
      return OptionalDouble.empty();
    }
    return OptionalDouble.of(poseOpt.get().getTranslation().getDistance(hubOpt.get()));
  }

  public OptionalDouble calculateRPM() {
    OptionalDouble distanceMetersOpt = hubDistanceMeters(drivetrain.getState().Pose);
    if (distanceMetersOpt.isEmpty()) {
      return OptionalDouble.empty();
    }
    double distanceFeet = Units.metersToFeet(distanceMetersOpt.getAsDouble());
    double rpm = interpolateRPM(distanceFeet);
    rpm = MathUtil.clamp(rpm, 1200, 6000);
    return OptionalDouble.of(rpm);
  }

  private static double interpolateRPM(double distanceFeet) {
    if (distanceFeet <= RPMLUT.firstKey()) {
      return RPMLUT.firstEntry().getValue();
    }
    if (distanceFeet >= RPMLUT.lastKey()) {
      return RPMLUT.lastEntry().getValue();
    }
    var lo = RPMLUT.floorEntry(distanceFeet);
    var hi = RPMLUT.ceilingEntry(distanceFeet);
    double t = (distanceFeet - lo.getKey()) / (hi.getKey() - lo.getKey());
    return lo.getValue() + t * (hi.getValue() - lo.getValue());
  }
```

- [ ] **Step 4: Delete the two absorbed files**

```bash
rm "src/main/java/frc/robot/subsystems/PoseHelpers.java"
rm "src/main/java/frc/robot/utility/RoboMath.java"
```

- [ ] **Step 5: Confirm no other file still references the deleted symbols**

Run: `grep -rn "PoseHelpers\|RoboMath\|hubTranslation" src/main/java`
Expected: no matches (RobotContainer.java's references get cleaned up in Task 5, not here — if this search finds hits only in `RobotContainer.java`, that's expected at this point in the plan; any hit in another file is a bug to fix now).

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/vision/Vision.java
git rm src/main/java/frc/robot/subsystems/PoseHelpers.java src/main/java/frc/robot/utility/RoboMath.java
git commit -m "Consolidate PoseHelpers/RoboMath into Vision subsystem"
```

---

### Task 2: Intake command factories

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/intake/Intake.java`
- Delete: `src/main/java/frc/robot/commands/Intake_Start.java`
- Delete: `src/main/java/frc/robot/commands/Intake_Stop.java`

**Interfaces:**
- Produces (used by Task 5): `Intake.startRoller()` -> `Command`. (`Intake.stopRoller()` already exists, unchanged.)

- [ ] **Step 1: Add `startRoller()` to Intake.java**

In `src/main/java/frc/robot/subsystems/intake/Intake.java`, insert immediately after the existing `eject()` method:

```java

  public Command startRoller() {
    return Commands.startEnd(
        () -> setRoller(Roller.INTAKE),
        () -> setRoller(Roller.STOP),
        this);
  }
```

- [ ] **Step 2: Delete the two superseded command files**

```bash
rm "src/main/java/frc/robot/commands/Intake_Start.java"
rm "src/main/java/frc/robot/commands/Intake_Stop.java"
```

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/intake/Intake.java
git rm src/main/java/frc/robot/commands/Intake_Start.java src/main/java/frc/robot/commands/Intake_Stop.java
git commit -m "Add Intake.startRoller() factory, remove superseded command files"
```

---

### Task 3: Superstructure.java

**Files:**
- Create: `src/main/java/frc/robot/subsystems/superstructure/Superstructure.java`
- Delete: `src/main/java/frc/robot/commands/ShootCmd.java`
- Delete: `src/main/java/frc/robot/commands/Shooting_Sequence.java`

**Interfaces:**
- Consumes: `Vision.calculateRPM()` -> `OptionalDouble` (Task 1), `Intake.getInstance()`, `Shooter.getInstance()`, `Vision.getInstance(CommandSwerveDrivetrain)` (all pre-existing Singletons).
- Produces (used by Task 5): `Superstructure.getInstance(CommandSwerveDrivetrain)` -> `Superstructure`; `Superstructure.shootCmd()` -> `Command`; `Superstructure.shootingSequence(double timeoutSeconds)` -> `Command`.

- [ ] **Step 1: Create the Superstructure singleton**

Create `src/main/java/frc/robot/subsystems/superstructure/Superstructure.java`:

```java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Roller;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

/**
 * Singleton Superstructure. Coordinates Shooter + Intake + Vision + drivetrain state together for
 * complex multi-subsystem sequences (shooting). Individual subsystems only handle their own
 * mechanism control; cross-subsystem orchestration lives here.
 */
public class Superstructure extends SubsystemBase {
  private static Superstructure instance;

  /** Creates (on first call) or returns the Superstructure Singleton. */
  public static Superstructure getInstance(CommandSwerveDrivetrain drivetrain) {
    if (instance == null) {
      instance = new Superstructure(drivetrain);
    }
    return instance;
  }

  private final Shooter shooter = Shooter.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Vision vision;

  /** Creates a new Superstructure. Use {@link #getInstance(CommandSwerveDrivetrain)} instead of constructing directly. */
  private Superstructure(CommandSwerveDrivetrain drivetrain) {
    this.vision = Vision.getInstance(drivetrain);
  }

  public Command shootCmd() {
    return Commands.parallel(
        intake.agitatePivot(),
        Commands.sequence(
            Commands.run(() -> {
              if (shooter.shooterTuningModeEnable) {
                shooter.targetRPMShooter(shooter.getTargetRPM());
              } else {
                var rpmOpt = vision.calculateRPM();
                rpmOpt.ifPresent(shooter::targetRPMShooter);
              }
            }).until(() -> shooter.shooterAtSpeed(shooter.getTargetRPM())),
            Commands.waitSeconds(0.1),
            Commands.run(() -> {
              shooter.indexControl(Shooter.indexing.INDEX);
              shooter.setAgitator(Shooter.Agitate.IN);

              if (shooter.shooterTuningModeEnable) {
                shooter.targetRPMShooter(shooter.getTargetRPM());
              } else {
                var rpmOpt = vision.calculateRPM();
                rpmOpt.ifPresent(shooter::targetRPMShooter);
                rpmOpt.ifPresent(rpm -> Logger.recordOutput("Shooter/TargetRPM", rpm));
              }
            }, shooter)))
        .finallyDo(() -> {
          intake.setRoller(Roller.STOP);
          shooter.indexControl(Shooter.indexing.STOP);
          shooter.setAgitator(Shooter.Agitate.STOP);
          shooter.targetRPMShooter(700);
          intake.goTo(Intake.PivotState.DOWN);
        });
  }

  public Command shootingSequence(double timeoutSeconds) {
    return Commands.parallel(
        intake.agitatePivot(),
        Commands.sequence(
            Commands.run(() -> {
              var rpmOpt = vision.calculateRPM();
              rpmOpt.ifPresent(shooter::targetRPMShooter);
              rpmOpt.ifPresent(rpm -> Logger.recordOutput("Shooter/Auto_TargetRPM", rpm));
            }, shooter).until(() -> shooter.shooterAtSpeed(shooter.getTargetRPM())).withTimeout(1),
            Commands.waitSeconds(0.3),
            Commands.run(() -> {
              shooter.setAgitator(Shooter.Agitate.IN);
              shooter.indexControl(Shooter.indexing.INDEX);
              var rpmOpt = vision.calculateRPM();
              rpmOpt.ifPresent(shooter::targetRPMShooter);
              rpmOpt.ifPresent(rpm -> Logger.recordOutput("Shooter/Auto_TargetRPM", rpm));
            }, shooter).withTimeout(timeoutSeconds)))
        .finallyDo(() -> {
          shooter.indexControl(Shooter.indexing.STOP);
          shooter.setAgitator(Shooter.Agitate.STOP);
          intake.goTo(Intake.PivotState.DOWN);
          shooter.targetRPMShooter(700);
        });
  }
}
```

- [ ] **Step 2: Delete the two ported command files**

```bash
rm "src/main/java/frc/robot/commands/ShootCmd.java"
rm "src/main/java/frc/robot/commands/Shooting_Sequence.java"
```

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/superstructure/Superstructure.java
git rm src/main/java/frc/robot/commands/ShootCmd.java src/main/java/frc/robot/commands/Shooting_Sequence.java
git commit -m "Add Superstructure singleton, migrate ShootCmd/Shooting_Sequence into it"
```

---

### Task 4: CommandSwerveDrivetrain factories

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`
- Delete: `src/main/java/frc/robot/commands/AutoAlignPOI.java`
- Delete: `src/main/java/frc/robot/commands/Target.java`

**Interfaces:**
- Consumes: `Vision.getHubPosition()`, `Vision.getPassTargetPosition()` (Task 1); `POI.get()`, `POI.getTargetRotation()` (pre-existing, unchanged).
- Produces (used by Task 5): `CommandSwerveDrivetrain.driveToPOI(POI)` -> `Command`; `CommandSwerveDrivetrain.trackHub(Vision, double, DoubleSupplier, DoubleSupplier, boolean)` -> `Command`; `CommandSwerveDrivetrain.trackPassTarget(Vision, double, DoubleSupplier, DoubleSupplier, boolean)` -> `Command`.

- [ ] **Step 1: Add new imports to CommandSwerveDrivetrain.java**

Add these imports (alongside the existing import block, before the `@SuppressWarnings("unused")` line):

```java
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.POI;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.Vision;
```

- [ ] **Step 2: Add `driveToPOI` after `sysIdDynamic`**

Insert immediately after the closing brace of `sysIdDynamic` (before `@Override public void periodic()`):

```java

    private static final double kAlignTolerance = 0.20;
    private static final double kAlignRotationTolerance = Math.toRadians(1);
    private static final double kAlignMaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static final double kAlignMaxRotationalSpeed = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /** Drives to and holds a field point-of-interest, ported from the old AutoAlignPOI command. */
    public Command driveToPOI(POI targetPOI) {
        SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        PIDController x = new PIDController(1, 0, 0);
        PIDController y = new PIDController(1, 0, 0);
        PIDController rot = new PIDController(3, 0, 0);
        x.setTolerance(kAlignTolerance);
        y.setTolerance(kAlignTolerance);
        rot.setTolerance(kAlignRotationTolerance);
        rot.enableContinuousInput(-Math.PI, Math.PI);

        return new FunctionalCommand(
                () -> {},
                () -> {
                    Translation2d target = targetPOI.get();
                    Translation2d current = getState().Pose.getTranslation();
                    Translation2d vectorToHub = target.minus(current);
                    Translation2d opVector = new Translation2d(
                            vectorToHub.getX() * getOperatorForwardDirection().getCos()
                                    + vectorToHub.getY() * getOperatorForwardDirection().getSin(),
                            -vectorToHub.getX() * getOperatorForwardDirection().getSin()
                                    + vectorToHub.getY() * getOperatorForwardDirection().getCos());

                    double vx = MathUtil.clamp(x.calculate(0, opVector.getX()), -kAlignMaxSpeed, kAlignMaxSpeed);
                    double vy = MathUtil.clamp(y.calculate(0, opVector.getY()), -kAlignMaxSpeed, kAlignMaxSpeed);

                    double currentRotation = getState().Pose.getRotation().getRadians();
                    double targetRotation = targetPOI.getTargetRotation().getRadians();
                    double rotError = MathUtil.angleModulus(targetRotation - currentRotation);
                    double dxRotRadiansPerSecond = rot.calculate(currentRotation, currentRotation + rotError);
                    double dxRot = MathUtil.clamp(
                            dxRotRadiansPerSecond / (2 * Math.PI), -kAlignMaxRotationalSpeed, kAlignMaxRotationalSpeed);

                    setControl(
                            request
                                    .withVelocityX(MetersPerSecond.of(vx))
                                    .withVelocityY(MetersPerSecond.of(vy))
                                    .withRotationalRate(dxRot));
                },
                interrupted -> idle(),
                () -> {
                    boolean atPos = getState().Pose.getTranslation().getDistance(targetPOI.get()) < kAlignTolerance;
                    boolean atRot = Math.abs(MathUtil.angleModulus(
                            getState().Pose.getRotation().getRadians()
                                    - targetPOI.getTargetRotation().getRadians())) < kAlignRotationTolerance;
                    return atPos && atRot;
                },
                this);
    }
```

- [ ] **Step 3: Add `trackHub`, `trackPassTarget`, and the shared `trackTarget` helper immediately after `driveToPOI`**

```java

    private static final double kTrackHeadingToleranceRad = Math.toRadians(2.0);

    /** Rotates to face the hub (from Vision's AprilTag-layout-averaged position) while driving. Ported from Target. */
    public Command trackHub(Vision vision, double maxSpeed, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            boolean finishOnAlign) {
        return trackTarget(vision::getHubPosition, maxSpeed, xSupplier, ySupplier, finishOnAlign);
    }

    /** Rotates to face the pass target while driving. Ported from Target. */
    public Command trackPassTarget(Vision vision, double maxSpeed, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            boolean finishOnAlign) {
        return trackTarget(vision::getPassTargetPosition, maxSpeed, xSupplier, ySupplier, finishOnAlign);
    }

    private Command trackTarget(Supplier<Optional<Translation2d>> targetSupplier, double maxSpeed,
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, boolean finishOnAlign) {
        PIDController headingPID = new PIDController(5.0, 0.0, 0.15);
        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        SwerveRequest.FieldCentric driveReq = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.Velocity);

        return new FunctionalCommand(
                () -> {
                    headingPID.reset();
                    double currentHeading = getState().Pose.getRotation().getRadians();
                    headingPID.calculate(currentHeading, currentHeading);
                },
                () -> {
                    Optional<Translation2d> targetOpt = targetSupplier.get();
                    if (targetOpt.isEmpty()) {
                        setControl(new SwerveRequest.Idle());
                        return;
                    }

                    Pose2d robotPose = getState().Pose;
                    double target = trackingAngle(targetOpt.get(), robotPose);
                    double rotationOutput = headingPID.calculate(robotPose.getRotation().getRadians(), target);

                    double vx = -ySupplier.getAsDouble() * maxSpeed;
                    double vy = -xSupplier.getAsDouble() * maxSpeed;

                    setControl(
                            driveReq
                                    .withDriveRequestType(DriveRequestType.Velocity)
                                    .withVelocityX(vx)
                                    .withVelocityY(vy)
                                    .withRotationalRate(rotationOutput));
                },
                interrupted -> setControl(new SwerveRequest.Idle()),
                () -> {
                    if (!finishOnAlign) {
                        return false;
                    }
                    Optional<Translation2d> targetOpt = targetSupplier.get();
                    if (targetOpt.isEmpty()) {
                        return false;
                    }
                    Pose2d robotPose = getState().Pose;
                    double target = trackingAngle(targetOpt.get(), robotPose);
                    double error = Math.abs(MathUtil.angleModulus(robotPose.getRotation().getRadians() - target));
                    return error < kTrackHeadingToleranceRad;
                },
                this);
    }

    private static double trackingAngle(Translation2d targetPos, Pose2d robotPose) {
        double offsetDeg = SmartDashboard.getNumber("offset", 0);
        Translation2d toTarget = targetPos.minus(robotPose.getTranslation());
        return toTarget.getAngle().getRadians() + Math.toRadians(offsetDeg);
    }
```

- [ ] **Step 4: Delete the two ported command files**

```bash
rm "src/main/java/frc/robot/commands/AutoAlignPOI.java"
rm "src/main/java/frc/robot/commands/Target.java"
```

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
git rm src/main/java/frc/robot/commands/AutoAlignPOI.java src/main/java/frc/robot/commands/Target.java
git commit -m "Add driveToPOI/trackHub/trackPassTarget drivetrain factories, remove ported commands"
```

---

### Task 5: RobotContainer cleanup and rewiring

**Files:**
- Modify: `src/main/java/frc/robot/RobotContainer.java`

**Interfaces:**
- Consumes: `Superstructure.getInstance(CommandSwerveDrivetrain)` (Task 3), `CommandSwerveDrivetrain.driveToPOI/trackHub/trackPassTarget` (Task 4), `Intake.stopRoller()` (pre-existing).

- [ ] **Step 1: Replace the import block**

Replace:

```java
import frc.robot.commands.AutoAlignPOI;
import frc.robot.commands.Intake_Start;
import frc.robot.commands.Intake_Stop;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.Target;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PoseHelpers;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Roller;
import frc.robot.subsystems.intake.Intake.PivotState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.indexing;
import frc.robot.subsystems.shooter.Shooter.Agitate;
import frc.robot.commands.AutoAlignPOI;
import frc.robot.commands.Shooting_Sequence;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.RoboMath;
```

with:

```java
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Roller;
import frc.robot.subsystems.intake.Intake.PivotState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.indexing;
import frc.robot.subsystems.shooter.Shooter.Agitate;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;
```

- [ ] **Step 2: Replace the field declarations**

Replace:

```java
        private final AutoAlignPOI aPOI = new AutoAlignPOI(drivetrain, POI.Hub);
        private final PoseHelpers poseHelpers = new PoseHelpers(drivetrain);
        private final Vision vision = Vision.getInstance(drivetrain);
        private boolean ctrlBtn;
        private final Intake_Start cmd_Intake_Start = new Intake_Start(intake);
        private final Intake_Stop cmd_Intake_Stop = new Intake_Stop(intake);
        private final Shooting_Sequence cmd_Normal_Shooting = new Shooting_Sequence(shooter, vision, intake, drivetrain,
                        5.0);
        private final Shooting_Sequence cmd_Quick_Shooting = new Shooting_Sequence(shooter, vision, intake, drivetrain,
                        3.0);
        private final ShootCmd cmd_ShootCmd = new ShootCmd(shooter, vision, drivetrain, intake);
        private SendableChooser<Command> autoChooser;
```

with:

```java
        private final Vision vision = Vision.getInstance(drivetrain);
        private final Superstructure superstructure = Superstructure.getInstance(drivetrain);
        private boolean ctrlBtn;
        private SendableChooser<Command> autoChooser;
```

- [ ] **Step 3: Rewire the constructor's NamedCommands registrations**

Replace:

```java
                NamedCommands.registerCommand("Orbit",
                                new Target(drivetrain, poseHelpers::getHubPosition, 0, () -> 0, () -> 0, true));
                NamedCommands.registerCommand(
                                "Shooting Sequence", cmd_Normal_Shooting);
                NamedCommands.registerCommand(
                                "Quick Shooting", cmd_Quick_Shooting);
                NamedCommands.registerCommand(
                                "Intake Start Sequence", Commands.parallel(
                                                intake.intakeJamReverse(),
                                                Commands.runOnce(() -> shooter.setAgitator(Agitate.IN))));
                NamedCommands.registerCommand(
                                "Intake Stop", cmd_Intake_Stop);
```

with:

```java
                NamedCommands.registerCommand("Orbit",
                                drivetrain.trackHub(vision, 0, () -> 0, () -> 0, true));
                NamedCommands.registerCommand(
                                "Shooting Sequence", superstructure.shootingSequence(5.0));
                NamedCommands.registerCommand(
                                "Quick Shooting", superstructure.shootingSequence(3.0));
                NamedCommands.registerCommand(
                                "Intake Start Sequence", Commands.parallel(
                                                intake.intakeJamReverse(),
                                                Commands.runOnce(() -> shooter.setAgitator(Agitate.IN))));
                NamedCommands.registerCommand(
                                "Intake Stop", intake.stopRoller());
```

- [ ] **Step 4: Rewire the target-tracking and POI bindings in `configureBindings()`**

Replace:

```java
                joystick.rightBumper().whileTrue(
                                new Target(drivetrain, poseHelpers::getHubPosition, MaxSpeed, joystick::getLeftX,
                                                joystick::getLeftY, false));
                joystick.rightTrigger().whileTrue(
                                new Target(drivetrain, poseHelpers::getPassTargetPosition, MaxSpeed,
                                                joystick::getLeftX, joystick::getLeftY, false));
```

with:

```java
                joystick.rightBumper().whileTrue(
                                drivetrain.trackHub(vision, MaxSpeed, joystick::getLeftX, joystick::getLeftY, false));
                joystick.rightTrigger().whileTrue(
                                drivetrain.trackPassTarget(vision, MaxSpeed, joystick::getLeftX, joystick::getLeftY,
                                                false));
```

Replace:

```java
                // joystick.povUp().whileTrue(new AutoAlignPOI(drivetrain, POI.Hub));
                // joystick.povLeft().whileTrue(new AutoAlignPOI(drivetrain, POI.poi1));
                joystick.povRight().whileTrue(new AutoAlignPOI(drivetrain, POI.Right));
                joystick.povLeft().whileTrue(new AutoAlignPOI(drivetrain, POI.Left));
                joystick.x().whileTrue(new AutoAlignPOI(drivetrain, POI.LeftStage));
                joystick.y().whileTrue(new AutoAlignPOI(drivetrain, POI.CenterStage));
                joystick.b().whileTrue(new AutoAlignPOI(drivetrain, POI.RightStage));
                controlBox.button(2).whileTrue(new ShootCmd(shooter, vision, drivetrain, intake));
```

with:

```java
                // joystick.povUp().whileTrue(drivetrain.driveToPOI(POI.Hub));
                // joystick.povLeft().whileTrue(drivetrain.driveToPOI(POI.poi1));
                joystick.povRight().whileTrue(drivetrain.driveToPOI(POI.Right));
                joystick.povLeft().whileTrue(drivetrain.driveToPOI(POI.Left));
                joystick.x().whileTrue(drivetrain.driveToPOI(POI.LeftStage));
                joystick.y().whileTrue(drivetrain.driveToPOI(POI.CenterStage));
                joystick.b().whileTrue(drivetrain.driveToPOI(POI.RightStage));
                controlBox.button(2).whileTrue(superstructure.shootCmd());
```

- [ ] **Step 5: Confirm no old symbols remain**

Run: `grep -rn "AutoAlignPOI\|Intake_Start\|Intake_Stop\|ShootCmd\|Shooting_Sequence\|Target(\|PoseHelpers\|RoboMath\|cmd_Intake\|cmd_Normal_Shooting\|cmd_Quick_Shooting\|cmd_ShootCmd\|\baPOI\b\|poseHelpers" src/main/java`
Expected: no matches anywhere in `src/main/java`.

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/RobotContainer.java
git commit -m "Rewire RobotContainer to Vision/Superstructure/CommandSwerveDrivetrain factories"
```

---

### Task 6: Full build verification

**Files:** none (verification only)

- [ ] **Step 1: Run the full compile**

Run: `./gradlew compileJava`
Expected: `BUILD SUCCESSFUL`

- [ ] **Step 2: If it fails, fix and re-run**

Read the compiler error carefully — it will name the exact file, line, and missing/mismatched symbol. Common failure classes to expect given this plan:
- A missed import in `CommandSwerveDrivetrain.java` (Task 4, Step 1) — check the exact type named in the error is in that import block.
- A leftover reference to a deleted class somewhere `grep` in Task 5 Step 5 didn't catch (e.g. inside a comment that still constructs an old class, or an autonomous/test file outside `src/main/java`).
- A method-reference type mismatch on `vision::getHubPosition` / `vision::getPassTargetPosition` against the `Supplier<Optional<Translation2d>>` parameter in `trackTarget` — both methods must return exactly `Optional<Translation2d>`.

Re-run `./gradlew compileJava` after each fix until `BUILD SUCCESSFUL`.

- [ ] **Step 3: Commit any fixes**

```bash
git add -A
git commit -m "Fix compile errors from Vision/Superstructure cleanup"
```

(Skip this step if Task 6 Step 1 already passed clean — no fixes to commit.)
