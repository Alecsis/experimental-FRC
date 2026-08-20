# Recommended 2027 Migration Plan

## Recommendation

Choose **Plan A: WPILib 2027/Systemcore first, Commands v2 initially, Commands v3 later**.

This is the only plan that preserves the existing PathPlanner/CTRE command integrations, keeps the current regression suite meaningful, and separates hardware/toolchain uncertainty from scheduler-semantic uncertainty. [VENDOR DOCS] [OFFICIAL SOURCE] [INFERRED]

Plan C is not currently a deployable mixed-framework option because the official Commands v2 and v3 vendordeps conflict. A “selective v3” study must be a separate branch/project, not a few v3 classes inside the v2 robot. [OFFICIAL SOURCE]

## Plan comparison

| Criterion | Plan A: Systemcore + v2, then v3 | Plan B: Systemcore + v3 together | Plan C: mixed/selective v3 |
|---|---|---|---|
| Behavior preservation | Best; existing scheduler semantics remain test oracle | Poor; toolchain, hardware, vendor and scheduler change together | Not supported in one project by vendordeps |
| Regression coverage | Existing v2 tests can run with import/API edits | Tests require framework rewrite before they can diagnose hardware/API issues | Would require separate module/project and duplicated fixtures |
| Vendor compatibility | Matches PathPlanner Alpha 3 and CTRE command surfaces | PathPlanner/NamedCommands block direct conversion | Packaging conflict |
| Sim compatibility | Still blocked by MapleSim, but blocker is isolated | MapleSim plus v3 rewrite multiply unknowns | Same Maple blocker plus packaging |
| Student maintainability | Familiar framework during hardware transition | Highest cognitive load | Two mental models and nonstandard build layout |
| Offseason learning value | Strong staged experiments with clear baselines | High novelty but weak causal diagnosis | Useful only as a separate lab |
| Recommendation | **YES** | No | No for robot; yes as later isolated lab |

## Sequence

### 0. Freeze evidence and toolchain

- Keep accepted commit 58031a57d0b32755d6bff8d580487f9272739c6d as the 2026 reference.
- Preserve the 2026 Java 17 baseline result.
- Select one complete official 2027 installer snapshot. Do not combine a main-branch Java 25 template with a tagged plugin unless the exact skew is recorded.
- Recheck Context7 and official release/vendor matrices on the migration day.

Exit: Java, Gradle, GradleRIO plugin, wpilibYear folder, WPILib artifact version, Systemcore image, DS, and vendor versions are one reviewed set.

### 1. Run the official importer

The importer is not installed in this study environment. When installed:

- Open the 2026 project using 2027 WPILib VS Code.
- Import into a new directory/worktree; never in place.
- Capture git diff before manually restoring custom build logic.
- Reimport vendors from official 2027 descriptors.
- Review namespace transformations rather than bulk accepting them.

Reapply and verify:

- replayWatch
- gversion/BuildConstants
- JUnit 5 launcher
- forkEvery = 1
- golden-update properties
- headless simulation switch
- deployment source/vendor backup policy, if still supported
- AdvantageKit annotation processor
- Systemcore target and /home/systemcore/deploy

Exit: Gradle configuration succeeds with no source compilation requested.

### 2. Establish the Commands v2 control build

- Install CommandsV2.json, not CommandsV3.json.
- Port Gamepad mappings explicitly.
- Apply real 2027 package/class mappings.
- Remove robotInit.
- Port AprilTag, DriverStation, framework, timer, math/linalg/system, NT, units and command2 packages.
- Port ChassisSpeeds/SwerveModuleState to current velocity types throughout robot, tests, telemetry, CTRE and PathPlanner boundaries.
- Rebuild LinearSystemId-based simulations with current official APIs.
- Update or isolate Limelight helper.
- Resolve MapleSim using an official release or agreed isolation strategy.

Exit:

- compileJava succeeds.
- compileTestJava succeeds.
- Core tests succeed.
- Full simulation suite status is explicit, not silently skipped.

### 3. Vendor and hardware bench

- CTRE 26.50.0-alpha-1; install CANivore IPKs.
- PathPlanner 2027 Alpha 3.
- AdvantageKit 27.0.0-alpha-4.
- Matching Limelight 2027 firmware/library.
- MapleSim compatible release if used.
- Systemcore image matching the selected WPILib/DS release.

Bench order:

1. Disabled boot and DS communications.
2. can_s0 mechanism device enumeration.
3. Named CANivore enumeration and Pigeon/CANcoder health.
4. Motor inversion/current/neutral tests at low output.
5. Odometry and pose reset.
6. Limelight connectivity, timestamps and pose.
7. /U and no-USB logging.
8. Teleop maintained switches and cleanup.
9. Autonomous twice consecutively.
10. Utility/Test/SysId only after lifecycle behavior is proven.

Exit: a tagged, deployable Commands v2 Systemcore control result with logs.

### 4. Restore regression confidence

Prioritize:

- OperatorStateCleanupTest
- OperatorTeleopExitIsolationTest
- OperatorBoxMaintainedSwitchLifecycleTest
- OperatorBoxAutonomousIsolationTest
- SuperstructureCommandRequirementsTest
- RepeatedAutonomousEntryTest
- RobotAutoTerminationTelemetryTest
- AutoNamedCommandResolutionTest
- path safety/regression tests
- intake/shooter jam tests
- drivetrain sysid and reset tests

Record unavoidable 2027 telemetry schema changes separately from behavior changes.

Exit: a written list of tests that pass, tests replaced for valid reasons, and hardware-only acceptance tests.

### 5. Commands v3 isolated vertical slice

Create a separate v3-only branch/project from the green control. Replace CommandsV2.json with CommandsV3.json; do not attempt coexistence.

First slice:

- required one-shot based on stowCmd
- small start/end motor command with normal and cancellation cleanup
- timeout
- sequence
- required/optional deadline
- onTrue and whileTrue
- one single-mechanism default command
- interruption test

Do not include PathPlanner, operatorPolicyCmd, Intake.homing, or autonomous wrappers in the first slice.

Exit: behavior tests prove success, timeout, interruption, disable, and reschedule paths.

### 6. Decide whether to adopt v3

Proceed only if:

- official v3 docs cover disabled behavior and the chosen robot lifecycle base;
- PathPlanner and CTRE integrations have official v3 support, or the team accepts owning adapters;
- student developers can explain yield, await/fork, requirements, scope, normal cleanup and cancellation cleanup;
- v3 regression results are at least as strong as the v2 control.

Otherwise keep v2 for the season and continue v3 as an offseason lab.

## Commands v3 conversion order after approval

1. stowCmd and isolated one-shots.
2. simple Intake/Shooter start/end factories.
3. timeout and small sequences.
4. trigger bindings with no vendor commands.
5. single-mechanism defaults.
6. Intake.homing.
7. Shooter jam/repeat/finalizer behavior.
8. Superstructure child scheduling and ownership.
9. operatorPolicyCmd.
10. autonomous wrapper and repeated entry.
11. PathPlanner/NamedCommands only after official v3 support.

## Rollback strategy

- 2026 reference stays untouched.
- Systemcore control branch uses v2 only.
- V3 work starts from a known-green control tag/commit and remains separate.
- Each phase stores the exact installer, Systemcore image, DS and vendor versions.
- Hardware changes are made one topology at a time: named CANivore retained first, onboard IMU deferred, extra native CAN buses deferred.

## Final acceptance definition

Migration is complete only when:

- compileJava and compileTestJava pass on the pinned Java 25 toolchain.
- Required tests pass or have reviewed, equivalent replacements.
- Systemcore boots repeatedly and communicates with the chosen DS.
- All CTRE devices and both Limelights are healthy.
- Autonomous can run twice without stale wrappers/state.
- teleopExit and interruption cleanup leave Superstructure neutral.
- Disabled Idle behavior is verified.
- AdvantageKit and CTRE logs are recoverable from intended storage.
- Simulation status is explicitly supported, replaced, or accepted as a documented limitation.

