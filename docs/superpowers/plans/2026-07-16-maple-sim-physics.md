# Maple-Sim Physics Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the drivetrain's physics-free WPILib simulation loop with maple-sim's rigid-body physics engine, and give `IntakeIOSim` a real simulated game-piece-contact signal.

**Architecture:** A new `frc.robot.utility.simulation.MapleSimSwerveDrivetrain` adapter (structurally adapted from Team 254's public 2025 codebase, not copy-merged) injects maple-sim physics into the existing CTRE `TalonFXSimState`/`CANcoderSimState`/`Pigeon2SimState` objects `CommandSwerveDrivetrain` already drives. `IntakeIOSim` reaches the resulting `AbstractDriveTrainSimulation` via `RobotContainer.drivetrain.getMapleSimDrive()` (mirroring the `import frc.robot.RobotContainer;` reference `CommandSwerveDrivetrain.java` already has) to build an `IntakeSimulation` for game-piece contact detection.

**Tech Stack:** WPILib 2026.2.1 / GradleRIO 2026.2.1, CTRE Phoenix 6, `org.ironmaple:maplesim-java:0.3.9` + `org.dyn4j:dyn4j:5.0.2`, AdvantageKit.

## Global Constraints

- No unit tests exist in this project. `./gradlew compileJava` is the verification gate for every task (same substitution used in the prior cleanup plan).
- `maple-sim.json` is pinned to `frcYear: "2025"`; this project is on GradleRIO `2026.2.1`. No newer version could be confirmed (web search was declined this session). Task 6 is the real test — if it fails to resolve/link, stop and report the exact error rather than guessing a different version number.
- Every vendor API surface used below (class names, method signatures, field names) was verified via Context7 against `/shenzhen-robotics-alliance/maple-sim` and `/websites/api_ctr-electronics_phoenix6_stable_java` (both high source reputation), or copied verbatim from `temp_reference/Team 254 Code`'s real working adapter file. None are guessed.
- Vendor dependencies go through `vendordeps/*.json`, picked up automatically by `wpi.java.vendor.java()` (`build.gradle:66`) — this project's existing convention for `AdvantageKit`/`Phoenix6`/`PathplannerLib`/`WPILibNewCommands`/`LumynLabs`. No `build.gradle` edits.

---

### Task 1: Add the maple-sim vendordep

**Files:**
- Create: `vendordeps/maple-sim.json`

**Interfaces:**
- Produces: `org.ironmaple:maplesim-java:0.3.9` and `org.dyn4j:dyn4j:5.0.2` become available on the compile classpath for all later tasks.

- [ ] **Step 1: Create the vendordep file**

Create `vendordeps/maple-sim.json` with this exact content (copied verbatim from `temp_reference/Team 254 Code/vendordeps/maple-sim.json`, a real working file from a published team codebase):

```json
{
    "fileName": "maple-sim.json",
    "name": "maplesim",
    "version": "0.3.9",
    "frcYear": "2025",
    "uuid": "c39481e8-4a63-4a4c-9df6-48d91e4da37b",
    "mavenUrls": [
        "https://shenzhen-robotics-alliance.github.io/maple-sim/vendordep/repos/releases",
        "https://repo1.maven.org/maven2"
    ],
    "jsonUrl": "https://shenzhen-robotics-alliance.github.io/maple-sim/vendordep/maple-sim.json",
    "javaDependencies": [
        {
            "groupId": "org.ironmaple",
            "artifactId": "maplesim-java",
            "version": "0.3.9"
        },
        {
            "groupId": "org.dyn4j",
            "artifactId": "dyn4j",
            "version": "5.0.2"
        }
    ],
    "jniDependencies": [],
    "cppDependencies": []
}
```

- [ ] **Step 2: Commit**

```bash
git add vendordeps/maple-sim.json
git commit -m "ogga bogga maple sim vendordep drop on ground"
```

(No compile check yet — nothing references the new classes until Task 2. Task 6 is the first full-tree compile.)

---

### Task 2: MapleSimSwerveDrivetrain adapter

**Files:**
- Create: `src/main/java/frc/robot/utility/simulation/MapleSimSwerveDrivetrain.java`

**Interfaces:**
- Consumes: `org.ironmaple:maplesim-java` (Task 1).
- Produces (used by Task 3): `MapleSimSwerveDrivetrain(Time, Mass, Distance, Distance, DCMotor, DCMotor, double, Translation2d[], Pigeon2, SwerveModule<TalonFX,TalonFX,CANcoder>[], SwerveModuleConstants<?,?,?>...)` constructor; `public final SwerveDriveSimulation mapleSimDrive` field; `public void update()`; `public static SwerveModuleConstants<?,?,?>[] regulateModuleConstantsForSimulation(SwerveModuleConstants<?,?,?>[])`.

- [ ] **Step 1: Create the adapter class**

Create `src/main/java/frc/robot/utility/simulation/MapleSimSwerveDrivetrain.java`:

```java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility.simulation;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Injects maple-sim rigid-body physics into a CTRE swerve drivetrain, replacing CTRE's
 * physics-free SimSwerveDrivetrain. Structurally adapted from Team 254's public 2025 codebase
 * (temp_reference/Team 254 Code) for this project's TalonFX + CANcoder + Pigeon2 swerve setup.
 */
public class MapleSimSwerveDrivetrain {
  private final Pigeon2SimState pigeonSim;
  public final SwerveDriveSimulation mapleSimDrive;

  public MapleSimSwerveDrivetrain(
      Time simPeriod,
      Mass robotMassWithBumpers,
      Distance bumperLengthX,
      Distance bumperWidthY,
      DCMotor driveMotorModel,
      DCMotor steerMotorModel,
      double wheelCOF,
      Translation2d[] moduleLocations,
      Pigeon2 pigeon,
      SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
      SwerveModuleConstants<?, ?, ?>... moduleConstants) {
    this.pigeonSim = pigeon.getSimState();

    DriveTrainSimulationConfig simulationConfig = DriveTrainSimulationConfig.Default()
        .withRobotMass(robotMassWithBumpers)
        .withBumperSize(bumperLengthX, bumperWidthY)
        .withGyro(COTS.ofPigeon2())
        .withCustomModuleTranslations(moduleLocations)
        .withSwerveModule(new SwerveModuleSimulationConfig(
            driveMotorModel,
            steerMotorModel,
            moduleConstants[0].DriveMotorGearRatio,
            moduleConstants[0].SteerMotorGearRatio,
            Volts.of(moduleConstants[0].DriveFrictionVoltage),
            Volts.of(moduleConstants[0].SteerFrictionVoltage),
            Meters.of(moduleConstants[0].WheelRadius),
            KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
            wheelCOF));
    mapleSimDrive = new SwerveDriveSimulation(simulationConfig, new Pose2d());

    SwerveModuleSimulation[] moduleSimulations = mapleSimDrive.getModules();
    for (int i = 0; i < moduleSimulations.length; i++) {
      moduleSimulations[i].useDriveMotorController(
          new TalonFXMotorControllerSim(modules[i].getDriveMotor()));
      moduleSimulations[i].useSteerMotorController(
          new TalonFXMotorControllerWithRemoteCanCoderSim(
              modules[i].getSteerMotor(), modules[i].getEncoder()));
    }

    SimulatedArena.overrideSimulationTimings(simPeriod, 1);
    SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
  }

  /** Advances the physics simulation one tick and injects the results into the CTRE sim devices. */
  public void update() {
    SimulatedArena.getInstance().simulationPeriodic();
    pigeonSim.setRawYaw(mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
    pigeonSim.setAngularVelocityZ(
        RadiansPerSecond.of(
            mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative().omegaRadiansPerSecond));
  }

  public static class TalonFXMotorControllerSim implements SimulatedMotorController {
    private final TalonFXSimState talonFXSimState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
      this.talonFXSimState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonFXSimState.setRawRotorPosition(encoderAngle);
      talonFXSimState.setRotorVelocity(encoderVelocity);
      talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      return talonFXSimState.getMotorVoltageMeasure();
    }
  }

  public static class TalonFXMotorControllerWithRemoteCanCoderSim extends TalonFXMotorControllerSim {
    private final CANcoderSimState remoteCancoderSimState;

    public TalonFXMotorControllerWithRemoteCanCoderSim(TalonFX talonFX, CANcoder cancoder) {
      super(talonFX);
      this.remoteCancoderSimState = cancoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      remoteCancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      remoteCancoderSimState.setRawPosition(mechanismAngle);
      remoteCancoderSimState.setVelocity(mechanismVelocity);
      return super.updateControlSignal(mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }

  /** Regulates all module constants for simulation. No-ops on real hardware. */
  public static SwerveModuleConstants<?, ?, ?>[] regulateModuleConstantsForSimulation(
      SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
    for (SwerveModuleConstants<?, ?, ?> moduleConstant : moduleConstants) {
      regulateModuleConstantForSimulation(moduleConstant);
    }
    return moduleConstants;
  }

  private static void regulateModuleConstantForSimulation(SwerveModuleConstants<?, ?, ?> moduleConstants) {
    if (RobotBase.isReal()) {
      return;
    }
    moduleConstants
        .withEncoderOffset(0)
        .withDriveMotorInverted(false)
        .withSteerMotorInverted(false)
        .withEncoderInverted(false)
        .withSteerMotorGains(moduleConstants.SteerMotorGains.withKP(70).withKD(4.5))
        .withDriveFrictionVoltage(Volts.of(0.1))
        .withSteerFrictionVoltage(Volts.of(0.15))
        .withSteerInertia(KilogramSquareMeters.of(0.05));
  }
}
```

- [ ] **Step 2: Commit**

```bash
git add src/main/java/frc/robot/utility/simulation/MapleSimSwerveDrivetrain.java
git commit -m "ogga bogga adapter class grow from 254 bones"
```

---

### Task 3: Constants.java physical placeholders

**Files:**
- Modify: `src/main/java/frc/robot/Constants.java`

**Interfaces:**
- Produces (used by Task 4 and Task 5): `Constants.kRobotMassWithBumpersKg`, `Constants.kBumperLengthXMeters`, `Constants.kBumperWidthYMeters`, `Constants.kWheelCOF`, `Constants.kIntakeSimWidthMeters`, `Constants.kIntakeSimCapacity` — all plain `double`/`int`, no new imports required.

- [ ] **Step 1: Append the new constants section**

In `src/main/java/frc/robot/Constants.java`, insert after the existing `// Shooter/Index/Agitator sim & physical constants` block (before the closing `}` of the class):

```java

  // Drivetrain simulation (maple-sim) physical constants (placeholder estimates -- TODO: measure/tune against real robot)
  public static final double kRobotMassWithBumpersKg = 55.0; // TODO: weigh the real robot with bumpers
  public static final double kBumperLengthXMeters = 0.9; // TODO: measure real bumper footprint
  public static final double kBumperWidthYMeters = 0.9; // TODO: measure real bumper footprint
  public static final double kWheelCOF = 1.2; // TODO: tune -- 1.2 is a typical rubber-tread-on-carpet estimate

  // Intake simulation (maple-sim) physical constants (placeholder estimates -- TODO: measure/tune against real robot)
  public static final double kIntakeSimWidthMeters = 0.7; // TODO: measure real intake width
  public static final int kIntakeSimCapacity = 1; // TODO: confirm real max held-piece count
```

- [ ] **Step 2: Commit**

```bash
git add src/main/java/frc/robot/Constants.java
git commit -m "ogga bogga constants grow more numbers"
```

---

### Task 4: Wire MapleSimSwerveDrivetrain into CommandSwerveDrivetrain

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`

**Interfaces:**
- Consumes: `MapleSimSwerveDrivetrain` constructor/`update()`/`regulateModuleConstantsForSimulation` (Task 2), `Constants.kRobotMassWithBumpersKg` etc. (Task 3).
- Produces (used by Task 5): `CommandSwerveDrivetrain.getMapleSimDrive()` -> `AbstractDriveTrainSimulation` (nullable — null on real hardware or before the sim thread starts).

- [ ] **Step 1: Add new imports**

In `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`, add to the import block (after the existing `frc.robot.subsystems.vision.Vision` import):

```java
import edu.wpi.first.math.system.plant.DCMotor;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import frc.robot.Constants;
import frc.robot.utility.simulation.MapleSimSwerveDrivetrain;
```

- [ ] **Step 2: Replace the sim-thread fields and add module-constants storage**

Replace:

```java
    private static final double kSimLoopPeriod = 0.02; // 20 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
```

with:

```java
    private static final double kSimLoopPeriod = 0.02; // 20 ms
    private Notifier m_simNotifier = null;
    private SwerveModuleConstants<?, ?, ?>[] moduleConstantsForSim;
    private MapleSimSwerveDrivetrain mapleSim;
```

- [ ] **Step 3: Regulate module constants and store them in all 3 constructors**

Replace:

```java
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }
```

with:

```java
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        moduleConstantsForSim = modules;
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }
```

Replace:

```java
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }
```

with:

```java
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency,
                MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        moduleConstantsForSim = modules;
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }
```

Replace:

```java
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }
```

with:

```java
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        moduleConstantsForSim = modules;
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }
```

- [ ] **Step 4: Replace startSimThread() to build and drive the maple-sim adapter**

Replace:

```java
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
```

with:

```java
    private void startSimThread() {
        // TODO: confirm actual drive/steer motors -- assumed Kraken X60
        mapleSim = new MapleSimSwerveDrivetrain(
                Seconds.of(kSimLoopPeriod),
                Kilograms.of(Constants.kRobotMassWithBumpersKg),
                Meters.of(Constants.kBumperLengthXMeters),
                Meters.of(Constants.kBumperWidthYMeters),
                DCMotor.getKrakenX60(1),
                DCMotor.getKrakenX60(1),
                Constants.kWheelCOF,
                getModuleLocations(),
                getPigeon2(),
                getModules(),
                moduleConstantsForSim);

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(mapleSim::update);
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
```

- [ ] **Step 5: Add resetPose() override and getMapleSimDrive() accessor**

Insert immediately after `startSimThread()`'s closing brace:

```java

    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
        if (mapleSim != null) {
            mapleSim.mapleSimDrive.setSimulationWorldPose(pose);
        }
    }

    /** Returns the maple-sim drivetrain simulation, or null on real hardware / before the sim thread starts. */
    public AbstractDriveTrainSimulation getMapleSimDrive() {
        return mapleSim == null ? null : mapleSim.mapleSimDrive;
    }
```

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
git commit -m "ogga bogga drivetrain wheels spin with real physics now"
```

---

### Task 5: IntakeIOSim game-piece contact detection

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/intake/IntakeIO.java`
- Modify: `src/main/java/frc/robot/subsystems/intake/IntakeIOSim.java`

**Interfaces:**
- Consumes: `RobotContainer.drivetrain.getMapleSimDrive()` (Task 4), `Constants.kIntakeSimWidthMeters`/`kIntakeSimCapacity` (Task 3).
- Produces: `IntakeIOInputs.hasGamePiece` (boolean field, read by `Intake.java`/AdvantageKit logging in future work — not consumed elsewhere in this plan).

- [ ] **Step 1: Add hasGamePiece to IntakeIOInputs**

In `src/main/java/frc/robot/subsystems/intake/IntakeIO.java`, add to `IntakeIOInputs` (after the `rollerTempCelsius` field):

```java
    public double rollerTempCelsius = 0.0;

    public boolean hasGamePiece = false;
```

(Replaces just the `rollerTempCelsius` line shown — add the new field directly below it, keeping the class's closing brace unchanged.)

- [ ] **Step 2: Add imports to IntakeIOSim.java**

In `src/main/java/frc/robot/subsystems/intake/IntakeIOSim.java`, add to the import block:

```java
import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.units.measure.Distance;
import frc.robot.RobotContainer;
```

- [ ] **Step 3: Add the lazily-built IntakeSimulation field and intake-running tracking**

Replace:

```java
  private double rollerAppliedVolts = 0.0;
  private boolean rollerClosedLoop = false;
  private double rollerVelocitySetpointRadPerSec = 0.0;
```

with:

```java
  private double rollerAppliedVolts = 0.0;
  private boolean rollerClosedLoop = false;
  private double rollerVelocitySetpointRadPerSec = 0.0;

  private IntakeSimulation intakeSimulation;
  private boolean intakeRunning = false;
```

- [ ] **Step 4: Update setRollerVoltage/setRollerVelocity to track intake-running state**

Replace:

```java
  @Override
  public void setRollerVoltage(double volts) {
    rollerClosedLoop = false;
    rollerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setRollerVelocity(double velocityRadPerSec) {
    rollerClosedLoop = true;
    rollerVelocitySetpointRadPerSec = velocityRadPerSec;
  }
```

with:

```java
  @Override
  public void setRollerVoltage(double volts) {
    rollerClosedLoop = false;
    rollerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    intakeRunning = false;
  }

  @Override
  public void setRollerVelocity(double velocityRadPerSec) {
    rollerClosedLoop = true;
    rollerVelocitySetpointRadPerSec = velocityRadPerSec;
    intakeRunning = velocityRadPerSec > 0;
  }
```

- [ ] **Step 5: Sync the IntakeSimulation and report hasGamePiece in updateInputs()**

Replace:

```java
    inputs.rollerConnected = true;
    inputs.rollerPositionRads = rollerSim.getAngularPositionRad();
    inputs.rollerVelocityRadsPerSec = rollerSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerStatorCurrentAmps = rollerSim.getCurrentDrawAmps();
    inputs.rollerSupplyCurrentAmps = rollerSim.getCurrentDrawAmps();
    inputs.rollerTempCelsius = 0.0;
  }
```

with:

```java
    inputs.rollerConnected = true;
    inputs.rollerPositionRads = rollerSim.getAngularPositionRad();
    inputs.rollerVelocityRadsPerSec = rollerSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerStatorCurrentAmps = rollerSim.getCurrentDrawAmps();
    inputs.rollerSupplyCurrentAmps = rollerSim.getCurrentDrawAmps();
    inputs.rollerTempCelsius = 0.0;

    if (intakeSimulation == null) {
      AbstractDriveTrainSimulation driveSim = RobotContainer.drivetrain.getMapleSimDrive();
      if (driveSim != null) {
        Distance width = Meters.of(Constants.kIntakeSimWidthMeters);
        intakeSimulation = IntakeSimulation.InTheFrameIntake(
            "Fuel", driveSim, width, IntakeSimulation.IntakeSide.FRONT, Constants.kIntakeSimCapacity);
      }
    }

    if (intakeSimulation != null) {
      if (intakeRunning && !intakeSimulation.isRunning()) {
        intakeSimulation.startIntake();
      } else if (!intakeRunning && intakeSimulation.isRunning()) {
        intakeSimulation.stopIntake();
      }
    }

    inputs.hasGamePiece = intakeSimulation != null && intakeSimulation.getGamePiecesAmount() > 0;
  }
```

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/intake/IntakeIO.java src/main/java/frc/robot/subsystems/intake/IntakeIOSim.java
git commit -m "ogga bogga intake feel fuel touch it"
```

---

### Task 6: Full build verification

**Files:** none (verification only)

- [ ] **Step 1: Run the full compile**

Run: `./gradlew compileJava`
Expected: `BUILD SUCCESSFUL`

- [ ] **Step 2: If it fails, diagnose before touching version numbers**

Read the compiler error. Expected failure classes, in order of likelihood:
- **Maven resolution failure** (`Could not resolve org.ironmaple:maplesim-java:0.3.9` or similar): this is the frcYear 2025-vs-2026 risk flagged in Global Constraints. Stop and report the exact error to the user rather than guessing a different version — this needs either a confirmed newer vendordep version or explicit user direction.
- **Missing import**: check the exact type named in the error is in the relevant file's import block from Task 2, 4, or 5.
- **Generic type mismatch on `modules[i].getDriveMotor()`/`.getSteerMotor()`/`.getEncoder()`**: these require `SwerveModule<TalonFX, TalonFX, CANcoder>[]`, not a wildcarded array — confirm `getModules()`'s return type matches (it should, since `TunerSwerveDrivetrain` is CTRE-generated with concrete `TalonFX`/`TalonFX`/`CANcoder` type parameters).

Re-run `./gradlew compileJava` after each fix until `BUILD SUCCESSFUL`, unless blocked on the Maven resolution failure above — that one requires stopping and reporting back.

- [ ] **Step 3: Commit any fixes**

```bash
git add -A
git commit -m "ogga bogga fix broken rock make build green"
```

(Skip if Step 1 already passed clean.)
