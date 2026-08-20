# Experimental Results

## Isolation

| Item | Result |
|---|---|
| Primary worktree | C:\Users\xdm\Claude\experimental-FRC |
| Primary state | Already dirty with user changes; not modified by this study |
| Accepted base | 58031a57d0b32755d6bff8d580487f9272739c6d |
| Experimental branch | experiment/2027-systemcore-commands-v3 |
| Experimental worktree | C:\Users\xdm\Claude\experimental-FRC-2027 |
| Push/merge | Neither performed |
| Importer/tooling install | 2027 WPILib not installed; no importer run; no large tooling installed |

[EXPERIMENTALLY VERIFIED]

## Graphify and source audit

Graphify CLI 0.9.48 was run against C:\Users\xdm\Claude\experimental-FRC-graphify. The graph had 1,281 nodes and no wiki index. Broad and lifecycle-focused queries surfaced Robot, RobotContainer, OperatorControls, Superstructure, Intake, Shooter, CommandSwerveDrivetrain, NamedCommands/PathPlanner, and the critical lifecycle tests. Source was then read directly to resolve stale-node and same-name collisions. [EXPERIMENTALLY VERIFIED]

Representative query expansions:

- command, robot, container, superstructure, intake, shooter, drivetrain, named, pathplanner, operator, teleop, homing
- operator, teleop, cancellation, clear, deadline, disabled, idle, schedule, sequence, shooting, request, intake

## 2026 baseline

Command:

    .\gradlew.bat compileJava compileTestJava --offline --no-daemon

Results:

- With the machine default Java 25.0.1: build-script evaluation failed in the test configuration with “Type T not present.” This is a Gradle 8.11/Java 25 environment mismatch, not a source regression.
- With JAVA_HOME=C:\Users\Public\wpilib\2026\jdk (Java 17.0.16): compileJava and compileTestJava both succeeded in 13 seconds.

Conclusion: accepted 2026 source has a green compile baseline on its intended toolchain. [EXPERIMENTALLY VERIFIED]

## Context7 result

Resolved library: /wpilibsuite/wpilib-docs (High reputation). Context7 supplied 2027 overview/changelog/importer/Systemcore/Commands v2/simulation/hardware/known-issues/NetworkTables material. Commands v3 behavioral details were incomplete, so the official allwpilib design doc, module source, vendordeps, and Hatchbot example were used for gaps as requested. [CONTEXT7 DOCS] [OFFICIAL SOURCE]

The Context7 “New for 2027” snapshot describes the Alpha 5-era migration: Java 25, package reorganization, Commands v3, constructor replacing robotInit, Gradle 9.4.1, and importer changes. Official public Alpha 6 installs into 2027_alpha5, but its tagged GradleRIO template uses Java 21 and Gradle 9.2.0. Current GradleRIO main uses Java 25 with newer application configuration but still a 9.2.0 wrapper. [CONTEXT7 DOCS] [OFFICIAL SOURCE] [EXPERIMENTALLY VERIFIED]

## Importer experiment

The 2027 WPILib installation directory C:\Users\Public\wpilib\2027_alpha5 does not exist. Only the 2026 installation is present. Per instructions, no installer was downloaded and the importer was not run. Therefore there is no importer-generated diff to claim. [EXPERIMENTALLY VERIFIED]

Expected importer behavior from Context7:

- copies/imports into a new directory rather than upgrading in place;
- regenerates Gradle components for the 2027 GradleRIO model;
- updates Java target/package namespaces;
- requires custom build.gradle logic to be manually reapplied;
- requires vendor libraries to be reimported.

[CONTEXT7 DOCS]

## Manual control-port reconstruction

The isolated worktree was manually changed to approximate the documented importer output and expose blockers:

- GradleRIO plugin ID: edu.wpi.first.GradleRIO to org.wpilib.GradleRIO.
- Plugin version: 2026.2.1 to 2027.0.0-alpha-6, the current public artifact.
- Java source/target: 17 to 25.
- Gradle wrapper: 8.11 to the Context7-documented 9.4.1.
- target: RoboRIO to SystemCore.
- team/debug extension: project.frc to project.wpilib.
- deploy path: /home/lvuser/deploy to /home/systemcore/deploy.
- JNI configurations: roborio to systemcore/linuxsystemcore.
- GradleRIO manifest package: edu.wpi.first.gradlerio to org.wpilib.gradlerio.
- settings local WPILib folder: 2026 to 2027_alpha5.
- removed empty Robot.robotInit override.
- changed real AdvantageKit path from media/sda1/logs to /U/logs.
- applied the official source-derived package mapping across 57 Java files.
- removed unsupported/unused 2026 vendor descriptors and added Commands v2, CTRE, PathPlanner, and AdvantageKit 2027 descriptors.
- did not add Commands v3.

The exact manual experiment diff remains available with git diff in the experimental worktree. It is a diagnostic port, not a production-ready commit. [EXPERIMENTALLY VERIFIED]

## Build attempts

| Attempt | Result | Meaning |
|---|---|---|
| Gradle 9.4.1 download/start under Java 25 | SUCCESS | Documented wrapper runs on host. |
| First 2027 configuration with old vendordeps | FAIL | Old WPILib-New-Commands descriptor used frcYear/2026 format; plugin expected wpilibYear 2027_alpha5. Import/reimport is mandatory. |
| Current-main GradleRIO application template against published Alpha 6 plugin | FAIL | WPILibJavaArtifact lacked debugJni. Main template is ahead of published plugin. |
| Published Alpha 6 fat-JAR configuration with Java 25/Gradle 9.4.1 | CONFIGURATION SUCCESS | Tagged plugin can configure under the documented Java/Gradle experiment after using its older API. |
| Commands v2, PathPlanner, AdvantageKit resolution | SUCCESS | Exact 2027 artifacts appear on compileClasspath. |
| CTRE latest jsonUrl from official descriptor | FAIL HTTP 404 | Catalog descriptor exists, but its CTRE latest URL was unavailable during the study. |
| CTRE descriptor from official WPILib vendor catalog | SUCCESS | com.ctre.phoenix6:wpiapi-java:26.50.0-alpha-1 resolves. |
| compileJava after namespace pass | FAIL at javac with 100-error cap | Remaining blockers are specific API/vendor issues listed below. |
| compileTestJava | NOT RUN | compileJava failed first. |
| Basic tests | NOT RUN | no compiled control port. |
| Commands v3 vertical slice | NOT RUN | phase gate requires a working control port first. |

[EXPERIMENTALLY VERIFIED]

## Final compile blockers observed

1. MapleSim packages absent after removing the incompatible 2026 descriptor.
2. ChassisSpeeds and SwerveModuleState absent; official artifact contains ChassisVelocities and SwerveModuleVelocity.
3. CTRE ApplyRobotSpeeds absent; official artifact contains ApplyRobotVelocity.
4. CommandXboxController absent; official Commands v2 artifact contains CommandGamepad and NI-specific generated controllers.
5. LinearSystemId absent; intake/shooter simulation factories must be replaced.
6. LimelightHelpers v1.14 imports Jackson, which is no longer on the WPILib compile classpath.
7. Additional renamed-method/field errors are hidden behind javac’s 100-error cap and must be addressed after the above.
8. PathPlanner Alpha 3 remains Commands v2-only, which is compatible with the control path but blocks v3 autonomous.

[EXPERIMENTALLY VERIFIED]

## Resolved artifact checks

Bytecode inspection confirmed:

- Commands v2: org.wpilib.command2 and CommandGamepad.
- WPIMath: ChassisVelocities fields vx, vy, omega; SwerveModuleVelocity fields velocity and angle.
- CTRE: SwerveRequest.ApplyRobotVelocity with ChassisVelocities.
- PathPlanner: AutoBuilder and NamedCommands use org.wpilib.command2.Command and ChassisVelocities.
- AprilTag: org.wpilib.vision.apriltag.
- Core simulation classes such as DriverStationSim and SimHooks remain present.

[OFFICIAL SOURCE] [EXPERIMENTALLY VERIFIED]

## Status

**Control migration status: BLOCKED at compileJava.**  
**Commands v3 experiment status: intentionally not started.**

This result does not show that Commands v2 is unusable. Commands v2 resolves and is the correct control framework. The blockers are the incomplete/discordant alpha toolchain plus repo-specific WPIMath, CTRE, MapleSim, Limelight, controller, and simulation API migrations.

