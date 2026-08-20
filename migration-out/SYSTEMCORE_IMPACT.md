# Systemcore Impact for This Robot

## Current topology

The accepted code has two CAN domains:

1. A named CANivore, Swerve, containing four TalonFX drive motors, four TalonFX steer motors, four CANcoders, and Pigeon2 ID 1. TunerConstants constructs new CANBus("Swerve", "./logs/example.hoot"). [EXPERIMENTALLY VERIFIED]
2. The default robot CAN bus containing mechanism TalonFX devices (intake pivot ID 2, intake roller ID 5, shooter ID 3, indexer ID 4) and a TalonFXS agitator ID 6. Constructors do not specify a bus. [EXPERIMENTALLY VERIFIED]

Other data/storage links are two Ethernet Limelights over NetworkTables, AdvantageKit NT4/log receivers, CTRE SignalLogger, and USB log storage. [EXPERIMENTALLY VERIFIED]

## Dependency audit

| Dependency | Classification | This robot’s action | Evidence |
|---|---|---|---|
| CANivore | TOPOLOGY CHANGE / VENDOR SUPPORT NEEDED | Keep named Swerve for the control port. Install canivore-usb-kernel first, then canivore-usb, and power-cycle Systemcore. Confirm Tuner can see the deployed robot; it cannot deploy its own temporary diagnostic server in the current alpha. | VENDOR DOCS |
| TalonFX drive/steer | API CHANGE | Phoenix 26.50.0-alpha-1 supports Systemcore, but CTRE’s swerve surface uses ChassisVelocities and ApplyRobotVelocity. Regenerate or carefully port TunerConstants and CommandSwerveDrivetrain. | VENDOR DOCS; EXPERIMENTALLY VERIFIED |
| TalonFX mechanism motors | UNCHANGED electrically / TOPOLOGY CHANGE | The no-bus constructor maps to Systemcore can_s0. Preserve IDs and validate bus load. Prefer explicit CANBus.systemCore(0) after the control build to make topology reviewable. | VENDOR DOCS |
| TalonFXS agitator | UNCHANGED electrically / TOPOLOGY CHANGE | No-bus TalonFXS also lands on can_s0. Verify Phoenix feature parity and the existing current/neutral configuration on hardware. | VENDOR DOCS; INFERRED |
| CANcoder | API CHANGE through CTRE swerve | Devices remain on named CANivore; generated swerve constants/types changed with Phoenix Alpha. Regenerate using a matching Tuner release if available. | VENDOR DOCS; EXPERIMENTALLY VERIFIED |
| Pigeon2 | UNCHANGED hardware / API CHANGE through CTRE swerve | Keep Pigeon2 on named CANivore as the control-port IMU. Do not simultaneously introduce the onboard IMU. | VENDOR DOCS; INFERRED |
| Onboard IMU | NOT USED | Optional later. If adopted, isolate it behind the Vision/IO-style logging boundary so AdvantageKit replay receives logged values instead of live Systemcore system stats. | CONTEXT7 DOCS; VENDOR DOCS |
| Multiple native CAN buses | TOPOLOGY CHANGE | Systemcore exposes can_s0 through can_s4. Initial plan: mechanisms on can_s0, existing named CANivore for swerve. A later bus redistribution needs an ID/bus wiring table and load capture. | CONTEXT7 DOCS; OFFICIAL SOURCE |
| Limelight bow/intake | API CHANGE / VENDOR SUPPORT NEEDED | Current LimelightHelpers v1.14 uses classic per-topic NT and Jackson. LimelightOS 2027 defaults to a MessagePack topic and disables classic NT output unless enabled per pipeline. Update firmware and use LimelightLib2/current helper, or explicitly re-enable classic output as a short-lived bridge. | VENDOR DOCS; EXPERIMENTALLY VERIFIED |
| Network topology | TOPOLOGY CHANGE | Verify both cameras, DS, Systemcore, and CANivore access over the chosen Ethernet/radio layout. Systemcore’s Wi-Fi, USB, and Ethernet IPs differ from roboRIO conventions; host can be robot.local/default Systemcore host. | OFFICIAL SOURCE |
| AdvantageKit | API CHANGE / SUPPORTED | 27.0.0-alpha-4 supports WPILib Alpha 6 and Systemcore image 11, Java 25, and LoggedRobot. OpModeRobot and Alert logging are not supported. Current LoggedRobot architecture can remain. | VENDOR DOCS; EXPERIMENTALLY VERIFIED |
| AdvantageKit WPILOGWriter | STORAGE PATH CHANGE | Existing media/sda1/logs is a roboRIO-era relative path. Systemcore USB mounts at /U, /V. Experiment uses /U/logs. Validate mount presence and fall back to /home/systemcore/logs or default writer policy. | OFFICIAL SOURCE; EXPERIMENTALLY VERIFIED |
| CTRE SignalLogger | STORAGE/API RISK | Code calls SignalLogger.start() from Telemetry, so it does not rely on alpha auto-start. The configured ./logs/example.hoot path is relative; verify final location, available space, and FMS file naming on Systemcore. | VENDOR DOCS; EXPERIMENTALLY VERIFIED; UNKNOWN |
| USB logging | STORAGE PATH CHANGE | Define an operations rule for “USB present”, “USB absent”, and full disk. Systemcore uses /U, /V instead of /media/sda1. | OFFICIAL SOURCE |
| Smart I/O / Expansion Hub | NOT USED | No direct DIO/analog APIs were found. No control-port wiring change is required. Future use must respect 3.3 V and alpha/beta pull resistor differences. | CONTEXT7 DOCS; EXPERIMENTALLY VERIFIED |
| Removed roboRIO hardware | NO IMPACT | No source imports for removed Relay, analog output/gyro, SPI, DMA, Counter, interrupts, ultrasonic, servo, Nidec, AxisCamera, or DigitalGlitchFilter APIs. | CONTEXT7 DOCS; EXPERIMENTALLY VERIFIED |
| WPILib simulation | API CHANGE | Core desktop simulation remains, but ChassisSpeeds/SwerveModuleState and LinearSystemId are absent from resolved Alpha 6 artifacts. Port to ChassisVelocities/SwerveModuleVelocity and current system-model factories. | CONTEXT7 DOCS; OFFICIAL SOURCE; EXPERIMENTALLY VERIFIED |
| MapleSim | VENDOR SUPPORT NEEDED | Current vendordep is 0.4.0-beta, frcYear 2026. Official main still uses GradleRIO 2026.2.1/Java 17 and exposes no 2027 branch/release. It is a hard compile and test blocker. | VENDOR DOCS; EXPERIMENTALLY VERIFIED |

## CAN migration detail

Phoenix’s documented Systemcore selection is CANBus.systemCore(busId), where busId 0 maps to can_s0. A no-argument device also uses can_s0. Named CANivore construction remains new CANBus("name"). String overloads on individual device constructors were removed earlier in the alpha line; this repository already passes a CANBus object through generated swerve configuration, so the named-bus model is structurally correct. [VENDOR DOCS] [EXPERIMENTALLY VERIFIED]

Recommended control topology:

| Bus | Devices | Reason |
|---|---|---|
| named CANivore Swerve | 8 TalonFX, 4 CANcoder, Pigeon2 | Preserve known swerve timing, generated configuration, and IDs. |
| can_s0 | Intake, shooter, indexer TalonFX; agitator TalonFXS | Matches current default-bus semantics with the fewest simultaneous changes. |
| can_s1–can_s4 | Empty for first deploy | Adds rollback clarity; redistribute only after capture of utilization and wiring validation. |

Before hardware enable:

- Install both CTRE CANivore IPKs in the documented order and power-cycle.
- Confirm Swerve resolves by name and all 13 swerve devices report.
- Confirm mechanism devices appear only on can_s0 and that no duplicate ID exists on that bus.
- Run disabled connectivity, steer encoder, Pigeon yaw, and motor direction checks before any closed-loop enable.
- Capture CAN utilization and CTRE status after migration; only then consider using additional native buses.

## Networking and vision

VisionIOReal is the only intended caller of LimelightHelpers, which is a useful containment boundary. However, the bundled helper is explicitly v1.14 for LimelightOS 2026 and directly imports Jackson. The 2027 WPILib classpath contains Avaje Jsonb for Gradle internals but not Jackson, so compile fails. LimelightOS 2027’s atomic MessagePack transport is also a behavior change, not merely a missing dependency. [VENDOR DOCS] [EXPERIMENTALLY VERIFIED]

Control-port choice:

1. Update both cameras to a documented matching LimelightOS 2027 build.
2. Replace v1.14 with official current LimelightLib2/helpers.
3. Keep all vendor calls inside VisionIOReal.
4. Re-run latency, timestamp, MegaTag 1/2, IMU mode, and disconnect/reconnect tests.
5. If a short-term classic-NT bridge is used, enable it per pipeline and record the setting as technical debt.

## Storage and logging

Current real-mode AdvantageKit output, media/sda1/logs, lacks a leading slash and is not a Systemcore mount. The experimental replacement /U/logs matches the first documented USB mount. CTRE’s ./logs/example.hoot is relative and therefore UNKNOWN until a deployed process’s working directory is confirmed. [OFFICIAL SOURCE] [EXPERIMENTALLY VERIFIED]

Acceptance checks:

- Boot with no USB: robot starts and logs to a deliberate internal/fallback location.
- Boot with USB mounted as /U: AdvantageKit and CTRE logs are written and readable.
- Reinsert/reboot: mount behavior and file naming remain stable.
- Fill threshold: disk pressure does not kill robot code.
- Replay: one WPILOG from a Systemcore run replays on desktop and contains all existing keys.

## Simulation conclusion

Simulation compatibility is currently **BLOCKED**, not merely “untested.” Core WPILib simulation is available, but this repository compiles MapleSim types into main source and uses MapleSim in drivetrain/intake tests. With no 2027 MapleSim artifact, compileJava stops before compileTestJava. A control migration must choose one:

- wait for an official MapleSim 2027 release and port against it; or
- temporarily replace MapleSim integration with WPILib/CTRE simulation adapters while preserving test interfaces; or
- isolate MapleSim behind a separate source set/module that is not needed for the robot artifact, then maintain a reduced 2027 core-sim test suite.

The first option minimizes behavior change and is recommended unless offseason timing demands an earlier deploy.

## Sources

Context7: systemcore-introduction.rst, analog-inputs-software.rst, digital-inputs-hardware.rst, removed-features.rst, robot-simulation/introduction.rst, yearly-changelog.rst.

Official/vendor material:

- https://github.com/wpilibsuite/SystemcoreTesting
- https://github.com/wpilibsuite/SystemcoreTesting/blob/main/CTR-Phoenix.md
- https://github.com/wpilibsuite/SystemcoreTesting/blob/main/AdvantageKit.md
- https://github.com/wpilibsuite/SystemcoreTesting/blob/main/LimelightVision.md
- https://github.com/wpilibsuite/SystemcoreTesting/blob/main/PathPlannerLib.md
- https://github.com/Shenzhen-Robotics-Alliance/maple-sim

