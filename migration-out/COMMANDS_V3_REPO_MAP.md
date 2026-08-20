# Commands v3 Repository Map

## Method

Graphify was used first against C:\Users\xdm\Claude\experimental-FRC-graphify, then the accepted-commit source and tests were read directly. Queries covered command, robot, container, superstructure, intake, shooter, drivetrain, NamedCommands, PathPlanner, operator, teleop, homing, cancellation, deadline, disabled, and repeated autonomous entry. [EXPERIMENTALLY VERIFIED]

Graph caveat: the audit graph contains pre-accepted-commit node IDs and same-name collisions in some broad results. All conclusions below were therefore verified against source at commit 58031a57d0b32755d6bff8d580487f9272739c6d. [EXPERIMENTALLY VERIFIED]

## Production command surfaces

| File/surface | Current behavior | Classification | V3 disposition |
|---|---|---|---|
| Robot.java | Runs CommandScheduler each loop; schedules/cancels auto; cancelAll in test; wraps auto with finallyDo telemetry; teleopExit delegates cleanup | SEMANTIC PORT / HIGH RISK | KEEP V2 INITIALLY |
| RobotContainer.java | Constructs subsystems, homing mode trigger, NamedCommands, AutoBuilder chooser, disabled Idle binding, sysid bindings | SEMANTIC PORT / HIGH RISK | KEEP V2 INITIALLY |
| OperatorControls.java | Three command HID wrappers, drive defaults, whileTrue/onTrue bindings, maintained-switch suppliers, Superstructure default policy | SEMANTIC PORT / HIGH RISK | KEEP V2 INITIALLY |
| Superstructure.java | Default policy command, request state machine, startEnd/runOnce factories, intake/shoot sequences, direct child schedule/cancel and scheduler ownership query | SEMANTIC PORT / HIGH RISK | KEEP V2 INITIALLY |
| Intake.java | runOnce/startEnd, repeated agitation sequence, finalizers, homing timeout/sequence, SysId routines | Mixed MECHANICAL/SEMANTIC; homing HIGH RISK | Slice simple factories only |
| Shooter.java | startEnd/runOnce/run, spin/wait sequence, repeated jam command with finalizer, deferred dashboard spin | Mixed MECHANICAL/SEMANTIC | Slice simple factories only |
| CommandSwerveDrivetrain.java | CTRE command subsystem, FunctionalCommand end callbacks, CTRE SysId, PathPlanner AutoBuilder, request factories | VENDOR-BOUND / HIGH RISK | BLOCKED by CTRE/PathPlanner v2 |
| Vision.java | SubsystemBase with periodic data; no rich command factories | MECHANICAL if converted | Low priority |
| NamedCommands | Registers homing, orbit timeout, shooting, quick shooting, intake timeout, intake stop | VENDOR-BOUND / HIGH RISK | KEEP V2; PathPlanner accepts v2 only |
| PathPlanner autos | AutoBuilder chooser and event command composition/deadlines | VENDOR-BOUND / HIGH RISK | KEEP V2; no v3 API in resolved alpha |
| Tests | Scheduler ownership, mode lifecycle, repeated auto, finalizers, timeout, sim race, sysid and path safety | REQUIRED REGRESSION ORACLE | Port assertions only after v3 branch compiles |

## Per-pattern inventory

### Robot lifecycle

- Robot constructor performs all initialization; the empty robotInit is an easy 2027 removal. [MECHANICAL PORT]
- robotPeriodic calls CommandScheduler.getInstance().run(). A v3 branch would call Scheduler.getDefault().run(), but only after all scheduled types in that project are v3. [SEMANTIC PORT]
- autonomousInit caches/wraps the selected command and schedules it. The wrapper is deliberately reused because v2 finallyDo marks the receiver composed. [HIGH RISK]
- autonomousExit and teleopInit cancel the cached autonomous command. [SEMANTIC PORT]
- teleopExit calls RobotContainer.teleopExit so operator state is cleared independent of scheduler cancellation timing. [PRESERVE]
- testInit calls cancelAll. [MECHANICAL API PORT; semantic retest]

### RobotContainer

- teleop RobotModeTrigger schedules Intake.homing. [SEMANTIC PORT]
- NamedCommands:
  - Home Intake — homing sequence.
  - Orbit — drive tracking with 2-second timeout.
  - Shooting Sequence / Quick Shooting — Superstructure sequence.
  - Intake — bounded intake sequence.
  - Intake Stop — request stop.
  [KEEP V2 INITIALLY]
- disabled RobotModeTrigger holds a CTRE idle request using ignoringDisable(true). Current v3 source has no equivalent documented contract. [BLOCKED/UNKNOWN]
- sysid triggers bind quasistatic/dynamic commands supplied by CTRE/WPILib v2. [KEEP V2 INITIALLY]
- teleopExit calls clearOperatorRequest. [PRESERVE]

### OperatorControls

- Drivetrain default commands continuously read live axes. [SEMANTIC PORT]
- Track-target and drive-to-POI actions use whileTrue. V3 whileTrue does not restart a naturally finished command; retryWhileTrue does. Preserve whileTrue initially. [MECHANICAL PORT with behavior test]
- Pose reset and related one-shots use onTrue/runOnce. [MECHANICAL PORT]
- The Superstructure default operatorPolicyCmd polls five BooleanSuppliers every cycle rather than binding each maintained switch to an edge. This is intentional maintained-switch behavior. [HIGH RISK]
- 2027 CommandGamepad renames a/b/x/y to south/east/west/north face and POV to dpad directions. [SEMANTIC PORT]

### Superstructure

- operatorPolicyCmd is Commands.run(..., this).finallyDo(interrupted -> clearOperatorRequest()).withName("OperatorPolicy"). [HIGH RISK]
- clearOperatorRequest only clears operator-owned intent and deliberately does not overwrite autonomous intent. [PRESERVE]
- updateIntakeCommand directly schedules/cancels mAgitateCommand and checks CommandScheduler.requiring(intake). V3 structured children, trees, and mechanism-owner lists change this contract. [HIGH RISK]
- shootCmd, intakeCmd, and ejectCmd use startEnd and requestStow cleanup. [SEMANTIC PORT]
- stowCmd is a required runOnce. [MECHANICAL PORT / easiest candidate]
- shootingSequence runs a required request then waits for ShotState.FINISHED. [SEMANTIC PORT]
- intakeSequence is intakeCmd().withTimeout(...). [SEMANTIC PORT]

### Intake

- roller SysId prep is runOnce.andThen(sysid).finallyDo. [SEMANTIC PORT]
- agitation is a repeating sequence with a pivot-down finalizer. [HIGH RISK]
- eject/intake roller commands are startEnd. [MECHANICAL shape, cleanup tests required]
- homing is a sequence: drive until hardstop with a 1-second timeout; zero/set homed; command down. Timeout must cancel the motion stage and still leave a safe motor state. [HIGH RISK]
- stopRoller is runOnce without an Intake requirement. This is an existing ownership smell to correct explicitly during either v2 or v3 work. [POSSIBLE RISK]

### Shooter

- agitate/index commands are startEnd; agitateStop is run without a requirement. [POSSIBLE RISK]
- indexJam composes a sequence/repetition and a final eject action. [SEMANTIC PORT / HIGH RISK]
- spin uses runOnce then waitUntil; dashboard spin is deferred. [SEMANTIC PORT]

### Drivetrain and autonomous

- CTRE request application, SwerveDrivetrain base integration, and SysId return v2 commands. [KEEP V2 INITIALLY]
- driveToPOI and trackTarget use FunctionalCommand with explicit end callbacks that restore idle/stop behavior. [HIGH RISK]
- PathPlanner 2027 Alpha 3 AutoBuilder and NamedCommands bytecode explicitly use org.wpilib.command2.Command and Subsystem. [BLOCKED for v3]
- Existing path deadline/event semantics therefore remain a v2-only surface until PathPlanner publishes v3 support or a team-owned adapter is designed. [KEEP V2 INITIALLY]

## Required Phase 8 decisions

| Behavior | V3 result | Reason and proof required |
|---|---|---|
| operatorPolicyCmd.finallyDo | CHANGES | Natural completion and cancellation use separate v3 paths. Implement idempotent cleanup in both and rerun OperatorStateCleanupTest. |
| clearOperatorRequest | PRESERVES as a plain method | Ownership rule can remain unchanged; call sites and scope timing must be retested. |
| teleopExit cleanup | PRESERVES | It is framework-independent lifecycle cleanup and should remain the backstop. |
| Intake.homing | UNKNOWN / likely CHANGES | Timeout is a race/cancel in v3; verify motor stop, homed flag, and down command for success, timeout, disable, and interruption. |
| shootingSequence | CHANGES | Built-in sequence can preserve full ownership, but coroutine await would release mechanisms between children. Choose built-in sequence first. |
| intakeCmd | CHANGES | Recreate start/end with explicit normal and cancellation cleanup. |
| PathPlanner deadlines | BREAKS for direct v3 port | Published PathPlanner Alpha 3 consumes v2 Command only. |
| NamedCommands | BREAKS for direct v3 port | Registry signature is registerCommand(String, org.wpilib.command2.Command). |
| repeated autonomous entry | UNKNOWN | V3 creates per-run IDs/coroutines, but the actual selected PathPlanner command is v2 and vendordeps conflict. |
| disabled Idle behavior | UNKNOWN / possible BREAKS | Current v3 source lacks ignoringDisable; LoggedRobot is not OpModeRobot. Hardware-disabled behavior needs official guidance and tests. |
| finalizer/interruption telemetry | CHANGES | V3 onCancel has no interrupted boolean and does not run on natural completion. |

## Regression tests to preserve

Highest-value existing tests:

- OperatorStateCleanupTest
- OperatorTeleopExitIsolationTest
- OperatorBoxMaintainedSwitchLifecycleTest
- OperatorBoxAutonomousIsolationTest
- OperatorBoxArbitrationTest
- RepeatedAutonomousEntryTest
- RobotAutoTerminationTelemetryTest
- SuperstructureCommandRequirementsTest
- AutoNamedCommandResolutionTest
- AutoCommandSafetyTest and path regression tests
- IntakeJamRecoveryTest
- CommandSwerveDrivetrain SysId workflow tests
- ResetPoseHeadingSimTest
- MapleSimIntakeToggleRaceTest

For a v3 branch, first duplicate a small subset as black-box behavior tests. Do not rewrite the existing v2 oracle in place.

## Vertical slice decision

The requested v3 vertical slice was **not executed** because the control WPILib 2027/Systemcore/Java 25/Commands v2 port does not compile. This obeys the phase gate. Once the control is green, use a separate v3-only branch and cover:

1. stowCmd — required one-shot.
2. a small Shooter or Intake start/end command — normal and canceled cleanup.
3. withTimeout around that command.
4. a two-step sequence.
5. a required/optional deadline.
6. onTrue and whileTrue bindings.
7. one single-mechanism default.
8. a deliberately interrupted command with cleanup assertions.

Do not select operatorPolicyCmd, PathPlanner, homing, or autonomous wrapping as the first slice.

