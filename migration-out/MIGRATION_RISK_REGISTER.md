# 2027 Migration Risk Register

Scales: likelihood and impact are Low, Medium, or High. “Trigger to close” is the evidence required to retire the risk.

| ID | Risk | Likelihood | Impact | Evidence/classification | Mitigation | Trigger to close |
|---|---|---:|---:|---|---|---|
| R1 | Documentation and published artifact skew (Java 25/Gradle 9.4.1 docs vs published Alpha 6 Java 21/Gradle 9.2 template/current main mix) | High | High | CONTEXT7 DOCS; OFFICIAL SOURCE; EXPERIMENTALLY VERIFIED | Pin one complete installer/toolchain snapshot; never mix main templates with tagged plugins without a recorded exception. | Official installer/template/plugin agree on Java, Gradle, package/API set and build locally. |
| R2 | Official 2027 importer absent | High now | High | CONTEXT7 DOCS; EXPERIMENTALLY VERIFIED | Do not install large tooling. Preserve manual reconstruction as study-only; rerun official importer when available. | Exact importer run and diff captured in isolated worktree. |
| R3 | MapleSim lacks 2027 compatibility | High | High | VENDOR DOCS; EXPERIMENTALLY VERIFIED | Wait for vendor release or isolate/replace simulation layer while preserving interfaces/tests. | Official 2027 vendordep resolves and all MapleSim-backed tests pass. |
| R4 | Limelight v1.14 fails compile and uses superseded 2027 transport | High | High | VENDOR DOCS; EXPERIMENTALLY VERIFIED | Update firmware and LimelightLib2/helper atomically; port only VisionIOReal. | compileJava plus two-camera pose/latency/reconnect hardware tests pass. |
| R5 | CTRE swerve API renamed around ChassisVelocities/ApplyRobotVelocity | High | High | VENDOR DOCS; EXPERIMENTALLY VERIFIED | Use matching Phoenix/Tuner; regenerate constants; reapply only documented customizations. | Drivetrain compiles, all devices enumerate, odometry and direction smoke tests pass. |
| R6 | CANivore requires Systemcore packages and changed diagnostic workflow | Medium | High | VENDOR DOCS | Preload both IPKs in order; document rollback; deploy minimal Phoenix robot before Tuner use. | Named Swerve bus and all 13 devices remain stable through reboot. |
| R7 | Gamepad replacement changes physical button names | High | High | CONTEXT7 DOCS; EXPERIMENTALLY VERIFIED | Create an explicit old-to-new mapping; student review; run operator intent and maintained-switch tests. | Port 0/2/3 mapping checklist and all operator tests pass. |
| R8 | WPIMath speed/state rename changes field names and telemetry schemas | High | High | OFFICIAL SOURCE; EXPERIMENTALLY VERIFIED | Port ChassisSpeeds to ChassisVelocities and SwerveModuleState to SwerveModuleVelocity; update CTRE/PathPlanner/log consumers together. | compileJava and path/drivetrain/telemetry regression tests pass with reviewed key/schema changes. |
| R9 | LinearSystemId removal breaks intake/shooter sim plants | High | Medium | OFFICIAL SOURCE; EXPERIMENTALLY VERIFIED | Recreate plants with current official system model factories; compare step responses. | Simulation response tolerances and subsystem tests match baseline. |
| R10 | Systemcore USB path/logging semantics lose data | High | High | OFFICIAL SOURCE; EXPERIMENTALLY VERIFIED | Use /U or documented default; explicit no-USB fallback; validate CTRE relative path. | Logs survive reboot, no-USB boot, near-full disk, and replay. |
| R11 | AdvantageKit alpha lacks Alerts and OpModeRobot | Medium | Medium | VENDOR DOCS | Keep LoggedRobot; do not adopt OpMode framework in control port; use existing telemetry for critical alerts. | Supported AKit release or accepted workaround validated. |
| R12 | NetworkTables/dashboard changes hide critical telemetry | Medium | Medium | CONTEXT7 DOCS | Keep typed NT4; inventory SmartDashboard publications; migrate operator dashboard to Elastic/AdvantageScope. | Dashboard acceptance checklist passes without NT3 clients. |
| R13 | Commands v2/v3 cannot coexist | High | High | OFFICIAL SOURCE | Treat v3 as separate project/branch after control, not file-by-file production mixing. | Official packaging removes conflict or full v3 project compiles without v2 vendors. |
| R14 | PathPlanner/NamedCommands are v2-only | High | High for v3 | VENDOR DOCS; EXPERIMENTALLY VERIFIED | Keep all autonomous v2. Do not build an unofficial adapter during Systemcore control migration. | Official v3 PathPlanner support plus repeated-auto/deadline tests pass. |
| R15 | V3 cancellation does not execute the abandoned coroutine body | High | High | OFFICIAL SOURCE | Put safe idempotent cleanup in normal exit and whenCanceled; test both. | start/end, timeout, interruption and disable tests pass. |
| R16 | operatorPolicy default/scoped semantics change in v3 | High | High | OFFICIAL SOURCE; INFERRED | Keep v2. Later port as a dedicated vertical slice after simpler mechanisms. | Maintained-switch, arbitration, autonomous isolation and cleanup suites all pass. |
| R17 | V3 disabled behavior under LoggedRobot is undocumented/unclear | High | High | CONTEXT7 DOCS; OFFICIAL SOURCE; UNKNOWN | Do not port disabled Idle or scheduler until official contract exists; add disabled hardware tests. | Official docs plus disabled transition test prove outputs/cleanup. |
| R18 | Repeated autonomous wrapper identity/finalizer behavior changes | Medium | High | EXPERIMENTALLY VERIFIED; OFFICIAL SOURCE | Preserve v2 control tests; in v3 use per-run telemetry and separate completion/cancel hooks. | RepeatedAutonomousEntryTest and end-reason telemetry equivalent pass twice consecutively. |
| R19 | Custom build.gradle logic is lost by importer | High | Medium | CONTEXT7 DOCS; EXPERIMENTALLY VERIFIED | Checklist replayWatch, gversion, JUnit 5, forkEvery, golden flags, headless sim, source/vendor backup. | Imported Gradle diff reviewed and all custom tasks verified. |
| R20 | LumynLabs descriptor is unsupported but perhaps operationally needed | Low | Medium | EXPERIMENTALLY VERIFIED; UNKNOWN | Confirm ownership/use. Default to removing because no imports exist. | Team confirms unused or vendor publishes matching 2027 support. |
| R21 | Generated TunerConstants hand-port diverges from calibrated robot | Medium | High | EXPERIMENTALLY VERIFIED | Prefer regeneration; diff CAN IDs, offsets, ratios, gains, inversions, bus and geometry values line by line. | Calibration checksum/review plus wheel-direction and odometry tests pass. |
| R22 | Graphify audit contains stale/colliding nodes | Medium | Low | EXPERIMENTALLY VERIFIED | Treat graph as navigation only and verify all claims against accepted source. | Fresh graph generated from the exact experiment commit or all cited source rechecked. |

## Highest-risk files

1. src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java — CTRE, PathPlanner, WPIMath, Commands v2, SysId, MapleSim, cleanup callbacks.
2. src/main/java/frc/robot/subsystems/superstructure/Superstructure.java — scheduler ownership, manual child commands, maintained policy, cleanup invariants.
3. src/main/java/frc/robot/Robot.java — lifecycle, AdvantageKit, auto scheduling/finalization, repeated entry, storage.
4. src/main/java/frc/robot/RobotContainer.java — NamedCommands, PathPlanner chooser, disabled behavior, homing/sysid mode bindings.
5. src/main/java/frc/robot/OperatorControls.java — Gamepad rename, defaults, maintained switches and trigger semantics.
6. src/main/java/frc/robot/subsystems/intake/Intake.java — homing timeout, repeated agitation, finalizers and SysId.
7. src/main/java/frc/robot/utility/simulation/MapleSimSwerveDrivetrain.java — unsupported vendor and renamed CTRE/WPIMath APIs.
8. src/main/java/frc/robot/utility/LimelightHelpers.java and VisionIOReal.java — missing Jackson and 2027 Limelight transport.
9. src/main/java/frc/robot/generated/TunerConstants.java — generated CTRE API and calibrated constants.
10. src/test/java/frc/robot/SimRobotLoop.java plus lifecycle/path tests — regression harness depends on the full simulation stack.

## Stop/go gates

- Gate A: one coherent 2027 installer/toolchain exists.
- Gate B: importer runs and vendor descriptors validate.
- Gate C: compileJava passes with Commands v2.
- Gate D: compileTestJava and non-Maple core tests pass.
- Gate E: Maple/full-loop simulation tests pass or an explicitly accepted replacement suite exists.
- Gate F: Systemcore bench deploy, CAN, vision, logging and DS transitions pass.
- Gate G: only then create a separate Commands v3 branch and vertical slice.

