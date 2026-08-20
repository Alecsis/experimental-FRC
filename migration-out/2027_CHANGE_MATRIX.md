# 2027 Repo-Relevant Change Matrix

Study snapshot: 2026-08-20  
Accepted source commit: 58031a57d0b32755d6bff8d580487f9272739c6d  
Experimental branch: experiment/2027-systemcore-commands-v3

## Evidence labels

- CONTEXT7 DOCS — retrieved from Context7 library /wpilibsuite/wpilib-docs.
- OFFICIAL SOURCE — WPILib, GradleRIO, or SystemcoreTesting source/release material.
- VENDOR DOCS — vendor release notes, documentation, source, or officially published descriptor.
- EXPERIMENTALLY VERIFIED — observed in this worktree or its resolved artifacts.
- INFERRED — conclusion from the cited evidence and this repository.
- UNKNOWN — current evidence is insufficient.

## Matrix

| 2027 change | Repo use | Classification | Repo-specific result and action | Evidence |
|---|---|---|---|---|
| Java 25 | build.gradle declares Java 17; tests and main are Java | MIGRATION REQUIRED | Importer is expected to set source/target 25. The host has Java 25.0.1. The 2026 Gradle 8.11 build fails under Java 25 but succeeds with the 2026 WPILib Java 17 JDK. | CONTEXT7 DOCS; EXPERIMENTALLY VERIFIED |
| GradleRIO/importer changes | Custom deployment, replayWatch, gversion, JUnit policy, simulation, fat JAR | MIGRATION REQUIRED / HIGH RISK | Import to a new directory. Manually reapply custom Gradle sections. The reconstructed Systemcore target uses org.wpilib.GradleRIO, project.wpilib, SystemCore, linuxsystemcore, and /home/systemcore/deploy. | CONTEXT7 DOCS; OFFICIAL SOURCE; EXPERIMENTALLY VERIFIED |
| Gradle 9.4.1 | Wrapper is 8.11 | MIGRATION REQUIRED | Wrapper 9.4.1 runs on Java 25. Published GradleRIO Alpha 6 templates still use Gradle 9.2.0 and Java 21, while current template source uses Java 25 but also 9.2.0. Treat the Context7 toolchain as documentation ahead of the published tag. | CONTEXT7 DOCS; OFFICIAL SOURCE; EXPERIMENTALLY VERIFIED |
| Java package reorganization | 361 edu.wpi.first references in Java at baseline | MIGRATION REQUIRED / HIGH RISK | 57 Java files changed in the namespace experiment. This is not a single prefix substitution: examples include wpilibj to framework/driverstation/system/simulation, math classes moving to util/linalg/system, and commands v2 to org.wpilib.command2. | CONTEXT7 DOCS; OFFICIAL SOURCE; EXPERIMENTALLY VERIFIED |
| robotInit removal | Robot has an empty robotInit; initialization is already in its constructor | MIGRATION REQUIRED / EASY | Delete the empty override. No behavior needs to move. | CONTEXT7 DOCS; EXPERIMENTALLY VERIFIED |
| Gamepad replacement | CommandXboxController on ports 0, 2, and 3; CommandGenericHID on port 1 | MIGRATION REQUIRED / SEMANTIC PORT | Resolved Commands v2 Alpha 6 has CommandGamepad and no CommandXboxController. Xbox a/b/x/y and POV bindings must be translated to south/east/west/north face and dpad names, then operator tests rerun. | CONTEXT7 DOCS; OFFICIAL SOURCE; EXPERIMENTALLY VERIFIED |
| Commands v2 | Entire robot, PathPlanner, CTRE swerve, and regression tests use it | THIS REPO USES IT / CONTROL PATH | Commands v2 remains a separate 2027 vendordep and resolves as org.wpilib.commandsv2:commandsv2-java. Keep it for the first deployable port. | CONTEXT7 DOCS; OFFICIAL SOURCE; EXPERIMENTALLY VERIFIED |
| Commands v3 | Not currently used | POSSIBLE RISK / OPTIONAL | Optional Java framework. The official vendordeps explicitly conflict, so v2 and v3 cannot coexist in one robot project. Evaluate v3 only on a separate branch/module after the v2 control is green. | CONTEXT7 DOCS; OFFICIAL SOURCE |
| NetworkTables v3 removal | Telemetry uses typed NT4 publishers; LimelightHelpers uses classic NetworkTableEntry APIs; SmartDashboard.put* and putData are used | POSSIBLE RISK | Typed NT4 publishers are the safe surface. NetworkTableEntry remains as an NT4 compatibility API. The removed SmartDashboard desktop application affects dashboards, not necessarily SmartDashboard robot-code calls. Limelight 2027 output is the larger risk. | CONTEXT7 DOCS; VENDOR DOCS; INFERRED |
| Shuffleboard/SmartDashboard desktop removal | SmartDashboard API publishes chooser, telemetry, and mechanism data | POSSIBLE RISK | Select Elastic or AdvantageScope and verify every published object. Do not equate removal of the desktop app with removal of the Java SmartDashboard API. | CONTEXT7 DOCS |
| Systemcore CAN buses | Named CANivore Swerve; default-bus TalonFX/TalonFXS mechanisms | MIGRATION REQUIRED / TOPOLOGY CHANGE | Keep named CANivore initially, install CTRE CANivore Systemcore packages, and let no-argument CTRE devices use can_s0. Explicitly inventory every ID per bus before distributing devices across can_s1–can_s4. | CONTEXT7 DOCS; VENDOR DOCS |
| Systemcore onboard IMU | Robot uses Pigeon2 on the Swerve CANivore | NOT USED / NO IMPACT initially | Keep Pigeon2 for the control port. Onboard IMU is a later experiment; AdvantageKit requires IO-layer logging for replay if adopted. | CONTEXT7 DOCS; VENDOR DOCS |
| Removed roboRIO hardware APIs | No Relay, AnalogOutput, AnalogGyro, SPI, DMA, Counter, Interrupt, Ultrasonic, Servo, Nidec, AxisCamera, or DigitalGlitchFilter imports found | NOT USED / NO DIRECT IMPACT | Re-audit any late-added hardware before migration. Analog input accumulation/averaging limitations are also not used. | CONTEXT7 DOCS; EXPERIMENTALLY VERIFIED |
| Smart I/O electrical behavior | No direct DigitalInput/AnalogInput use found in accepted commit | NOT USED / HARDWARE WATCH | If switches move onto Smart I/O later, account for 3.3 V and alpha/beta pull resistor differences; do not reuse roboRIO wiring assumptions. | CONTEXT7 DOCS; OFFICIAL SOURCE |
| WPIMath velocity API rename | Drivetrain, telemetry, PathPlanner controller, tests use ChassisSpeeds and SwerveModuleState | MIGRATION REQUIRED / HIGH RISK | Published Alpha 6 provides ChassisVelocities and SwerveModuleVelocity. CTRE provides ApplyRobotVelocity. Port field names and logging schemas, not only class names. | OFFICIAL SOURCE; EXPERIMENTALLY VERIFIED |
| Linear-system simulation API | IntakeIOSim and ShooterIOSim use LinearSystemId factories | MIGRATION REQUIRED | LinearSystemId is absent in the resolved 2027 WPIMath artifact. Rebuild these plants with the replacement system-model APIs after selecting the exact released WPILib snapshot. | OFFICIAL SOURCE; EXPERIMENTALLY VERIFIED |
| WPILib desktop simulation | DriverStationSim, SimHooks, DCMotorSim, FlywheelSim, SingleJointedArmSim and many lifecycle tests | THIS REPO USES IT / POSSIBLE RISK | Core simulation classes resolve in 2027, but renamed math APIs and MapleSim prevent compilation and therefore prevent test execution. | CONTEXT7 DOCS; EXPERIMENTALLY VERIFIED |
| Driver Station changes | Mode lifecycle and simulation tests depend on DS transitions | POSSIBLE RISK | Systemcore image 10+ and WPILib Alpha 5+ are required for new DS features. Keep existing lifecycle cleanup tests and add real DS transition smoke tests. | CONTEXT7 DOCS; OFFICIAL SOURCE |
| Utility/Test mode naming | testInit/test mode is used for cancelAll and sysid bindings | MIGRATION REQUIRED / POSSIBLE RISK | Docs rename Test to Utility. Verify lifecycle callback compatibility in the exact installed release before changing code or student procedures. | CONTEXT7 DOCS; UNKNOWN |
| Logging/storage | AdvantageKit writes real logs to media/sda1/logs; CTRE hoot uses ./logs/example.hoot; USB logging required | MIGRATION REQUIRED / HIGH RISK | Systemcore USB mounts are /U, /V, etc. The experiment changes AdvantageKit to /U/logs. Decide and test CTRE SignalLogger location; relative working-directory behavior is not yet verified. | OFFICIAL SOURCE; EXPERIMENTALLY VERIFIED |
| Jackson transitive dependency | LimelightHelpers v1.14 imports Jackson directly | MIGRATION REQUIRED / VENDOR PORT | 2027 WPILib uses Avaje internally and no longer supplies Jackson on this compile classpath. Adopt official LimelightLib2/2027 helper or declare an explicitly supported JSON dependency; do not rely on WPILib transitivity. | CONTEXT7 DOCS; VENDOR DOCS; EXPERIMENTALLY VERIFIED |

## Context7 pages used

The following exact source pages were returned by Context7:

- source/docs/yearly-overview/index.rst — 2027 Overview
- source/docs/yearly-overview/yearly-changelog.rst — New for 2027
- source/docs/yearly-overview/known-issues.rst — Known Issues
- source/docs/yearly-overview/returning-quickstart.rst — Returning Teams Quick Start
- source/docs/yearly-overview/removed-features.rst — removed features and hardware APIs
- source/docs/software/vscode-overview/importing-last-years-robot-code.rst — project importer
- source/docs/software/vscode-overview/3rd-party-libraries.rst — vendor and Commands vendordeps
- source/docs/software/systemcore-info/systemcore-introduction.rst — Systemcore introduction
- source/docs/software/hardware-apis/sensors/analog-inputs-software.rst — analog API limits
- source/docs/hardware/sensors/digital-inputs-hardware.rst — Smart I/O electrical behavior
- source/docs/software/commandbased/commands-v2/index.rst
- source/docs/software/commandbased/commands-v2/commands.rst
- source/docs/software/commandbased/commands-v2/command-scheduler.rst
- source/docs/software/commandbased/commands-v2/command-compositions.rst
- source/docs/software/commandbased/commands-v2/binding-commands-to-triggers.rst
- source/docs/software/wpilib-tools/robot-simulation/introduction.rst
- source/docs/software/networktables/nt4-migration-guide.rst
- source/docs/software/networktables/listening-for-change.rst
- source/docs/software/networktables/client-side-program.rst

## Version interpretation

The retrieved New for 2027 material explicitly identifies the Java 25 importer/tooling section as 2027 Alpha 5 and describes Gradle 9.4.1. The current official public installer is v2027.0.0-alpha-6 and deliberately installs into 2027_alpha5. However, the published GradleRIO Alpha 6 tag uses Java 21/Gradle 9.2.0, while current GradleRIO main uses Java 25 but still Gradle 9.2.0. This report therefore calls the documentation snapshot “Alpha 5-era 2027 docs” and treats the public Alpha 6 artifacts as a separate, skewed experimental target.

