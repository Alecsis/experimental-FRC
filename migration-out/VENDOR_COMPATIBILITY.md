# 2027 Vendor Compatibility

Snapshot date: 2026-08-20. Alpha compatibility is volatile; pin every descriptor and recheck before hardware deployment.

## Compatibility table

| Library | Current repo version/use | Official 2027 status | Classification | Repo-specific conclusion | Evidence |
|---|---|---|---|---|---|
| CTRE Phoenix 6 | 26.1.3; TalonFX, TalonFXS, CANcoder, Pigeon2, swerve, SignalLogger, CANivore | 26.50.0-alpha-1 for WPILib 2027 Alpha 5/6; 26.x firmware supported | SUPPORTED WITH API/TOPOLOGY CHANGES | Required. Update vendordep, regenerate/port swerve for ChassisVelocities/ApplyRobotVelocity, install CANivore IPKs. | VENDOR DOCS; EXPERIMENTALLY VERIFIED |
| PathPlannerLib | 2026.1.2; AutoBuilder, NamedCommands, paths/autos, custom controller | 2027.0.0-alpha-3 for Alpha 5/6 | SUPPORTED WITH API CHANGES; V2 ONLY | Required for control. Resolved API uses ChassisVelocities and org.wpilib.command2.Command. It blocks a direct v3 autonomous conversion. | VENDOR DOCS; EXPERIMENTALLY VERIFIED |
| AdvantageKit | 26.0.2; LoggedRobot, WPILOGWriter, NT4Publisher, annotations | 27.0.0-alpha-4 for WPILib Alpha 6 and Systemcore image 11 | SUPPORTED WITH KNOWN ISSUES | LoggedRobot is supported. OpModeRobot and Alert logging are not; console may lag about 250 ms. Update storage path. | VENDOR DOCS; EXPERIMENTALLY VERIFIED |
| MapleSim | 0.4.0-beta; main-source drivetrain/intake simulation and tests | No official 2027 release/branch found; official main still GradleRIO 2026.2.1 and Java 17 | BLOCKER | Hard compile/test blocker. Wait for official support or replace/isolate MapleSim before the control build can pass. | VENDOR DOCS; EXPERIMENTALLY VERIFIED |
| Limelight | Bundled LimelightHelpers v1.14; two cameras; classic NT, MegaTag, IMU mode | 2027 beta work is active; LimelightOS 2027 defaults to atomic MessagePack NT and classic output is disabled unless enabled per pipeline; LimelightLib2 is the current object API | VENDOR SUPPORT/API MIGRATION NEEDED | Update camera OS and helper together. Current helper also fails compile because Jackson is no longer transitively present. | VENDOR DOCS; EXPERIMENTALLY VERIFIED |
| REVLib | No REV vendordep/imports/devices found | 2027.0.0-alpha-2 listed for Alpha 5/6 | NOT RELEVANT | Do not add it. Re-evaluate only if hardware changes. | VENDOR DOCS; EXPERIMENTALLY VERIFIED |
| LumynLabs | Descriptor 2026.2.0 present, but no com.lumyn imports found | No official 2027 compatibility evidence gathered | UNUSED / UNKNOWN | Remove from the control import unless a non-source deployment dependency is identified. It currently blocks vendor-year validation without providing code. | EXPERIMENTALLY VERIFIED; UNKNOWN |

## CTRE details

Official SystemcoreTesting states:

- Phoenix 6 26.50.0-alpha-1 matches WPILib 2027 Alpha 5/6.
- Any Phoenix 6 device firmware in the 26.x line is compatible.
- CANBus.systemCore(int) selects native Systemcore buses; no bus selects can_s0.
- Named CANivore remains new CANBus("name").
- The individual-device string bus overload is removed; pass CANBus.
- Phoenix 5 is unavailable.
- Tuner cannot deploy its temporary diagnostic server to Systemcore; deploy robot code with a Phoenix device instantiated.
- CANivore requires canivore-usb-kernel then canivore-usb and a power cycle.

[Vendor docs] https://github.com/wpilibsuite/SystemcoreTesting/blob/main/CTR-Phoenix.md

Experimental notes:

- The WPILib vendor catalog contains Phoenix6-26.50.0-alpha-1.json and the artifact resolves successfully.
- The descriptor’s CTRE latest jsonUrl returned HTTP 404 during GradleRIO vendordep import. Importing the catalog’s raw descriptor worked. This is a tooling/catalog distribution issue to recheck, not evidence that the Maven artifact is absent.
- Resolved bytecode provides SwerveRequest.ApplyRobotVelocity, not ApplyRobotSpeeds.

## PathPlanner details

Official URL:

https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLibSystemCoreAlpha.json

The resolved Alpha 3 signatures show:

- NamedCommands.registerCommand(String, org.wpilib.command2.Command)
- AutoBuilder.configure suppliers/consumers use ChassisVelocities
- AutoBuilder follow/pathfind/buildAuto return org.wpilib.command2.Command
- buildAutoChooser returns SendableChooser<org.wpilib.command2.Command>

Therefore PathPlanner is compatible with the control migration but **not** with an in-project Commands v3 port under current packaging. No official v3 adapter was found. [VENDOR DOCS] [EXPERIMENTALLY VERIFIED]

## AdvantageKit details

Official URL:

https://github.com/Mechanical-Advantage/AdvantageKit/releases/download/v27.0.0-alpha-4/AdvantageKit.json

Relevant compatibility:

- WPILib 2027.0.0-alpha-6
- Systemcore alpha/beta image 11
- Java 25
- LoggedRobot supported
- OpModeRobot unavailable
- Alert logging temporarily unavailable
- onboard IMU system-stat data is not automatically replay-safe; log it through IO if used
- console capture can be delayed by about 250 ms

Current Robot extends LoggedRobot, so no base-class change is needed for the control port. The real log path must change from media/sda1/logs to /U/logs, a tested default, or /home/systemcore/logs. [VENDOR DOCS] [EXPERIMENTALLY VERIFIED]

## MapleSim details

The repository’s vendordep declares frcYear 2026 and version 0.4.0-beta. Official maple-sim main was checked and still declares GradleRIO 2026.2.1, Java 17, and edu.wpi.first artifacts; no 2027 branch/release was found. This makes MapleSim the clearest vendor blocker in this codebase because its types appear in main source, not only test source. [VENDOR DOCS] [EXPERIMENTALLY VERIFIED]

Files directly affected:

- src/main/java/frc/robot/utility/simulation/MapleSimSwerveDrivetrain.java
- src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
- src/main/java/frc/robot/subsystems/intake/IntakeIOSim.java
- MapleSimIntakeToggleRaceTest and multiple full-loop simulation tests

## Limelight details

Current helper header says “LimelightHelpers v1.14 (REQUIRES LLOS 2026.0 OR LATER).” LimelightOS 2027 changes transport and coordinate conventions, and current vendor notes direct teams toward LimelightLib2. Classic NT output may be enabled per pipeline as a bridge, but that should be temporary because it preserves a 2026 interface rather than validating the 2027 path. [VENDOR DOCS] [EXPERIMENTALLY VERIFIED]

A safe upgrade is atomic:

1. Snapshot camera configs.
2. Update both Limelights to the chosen 2027 firmware.
3. Replace helper/library with that firmware’s official Java API.
4. Port VisionIOReal only.
5. Validate robot orientation publishing, MegaTag1/2 estimates, timestamps/latency, target loss, and reconnect behavior.
6. Compare logged poses and rejection counts to the 2026 baseline.

## Vendor blocker ranking

1. **MapleSim — hard blocker:** no compatible artifact; compileJava and tests cannot pass.
2. **Limelight — hard source/behavior blocker:** v1.14 requires missing Jackson and uses the old transport.
3. **CTRE swerve — significant API port:** supported artifact exists, but regenerated constants and velocity APIs changed.
4. **PathPlanner — significant API port:** supported for v2; uses renamed math types; blocks v3 autonomous.
5. **AdvantageKit — manageable:** compatible release exists; storage and known alpha limitations remain.
6. **LumynLabs — unknown but unused:** remove unless ownership/use is demonstrated.
7. **REV — no impact:** not used.

## Source index

- Systemcore vendor matrix: https://github.com/wpilibsuite/SystemcoreTesting
- CTRE: https://github.com/wpilibsuite/SystemcoreTesting/blob/main/CTR-Phoenix.md
- PathPlanner: https://github.com/wpilibsuite/SystemcoreTesting/blob/main/PathPlannerLib.md
- AdvantageKit: https://github.com/wpilibsuite/SystemcoreTesting/blob/main/AdvantageKit.md
- Limelight: https://github.com/wpilibsuite/SystemcoreTesting/blob/main/LimelightVision.md
- MapleSim source: https://github.com/Shenzhen-Robotics-Alliance/maple-sim
- WPILib vendor catalog: https://github.com/wpilibsuite/vendor-json-repo

