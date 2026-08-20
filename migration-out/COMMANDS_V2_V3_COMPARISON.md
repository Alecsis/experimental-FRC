# Commands v2 vs Commands v3 for This Repository

## Direct answers

1. **Is Commands v2 still supported in 2027?** Yes. Context7 still contains the production Commands v2 documentation, and the official 2027 CommandsV2.json resolves org.wpilib.commandsv2:commandsv2-java. [CONTEXT7 DOCS] [OFFICIAL SOURCE] [EXPERIMENTALLY VERIFIED]
2. **Is Commands v3 optional?** Yes. It is a separate Java vendordep/module, not an automatic replacement in imported v2 projects. [CONTEXT7 DOCS] [OFFICIAL SOURCE]
3. **Can v2 and v3 coexist?** No, not in one normal GradleRIO robot project. Both official vendordeps declare a conflict. This rules out Plan C as an in-project mixture under the present Alpha 5/6 packaging. [OFFICIAL SOURCE]
4. **What provides v3?** CommandsV3.json version 1.0.0, Maven artifact org.wpilib:commands3-java, Java package org.wpilib.command3, and HID helpers under org.wpilib.command3.button. [OFFICIAL SOURCE]
5. **What changes in scheduling?** V3 commands are coroutine-backed, queued top-level commands start on a scheduler run, inner commands may start immediately, requirements are mechanism-based with priorities, command trees are visible, and bindings/defaults/manual schedules have scopes. [OFFICIAL SOURCE]
6. **What changes in command operations?** See the table below.
7. **What is coroutine/continuation behavior?** Each run has a continuation. Command code runs until it returns or explicitly yields. await/fork expose child commands to the scheduler. Cancellation abandons the continuation, calls onCancel, and cancels descendants. All scheduler operations must remain on one thread; running on virtual or multiple threads is explicitly unsafe. [OFFICIAL SOURCE]
8. **What compile-time checks exist?** Staged builders require requirements/no-requirements, an execution body, and a name before a Command can be produced. Commands are annotated NoDiscard. The 2027 changelog describes javac/Error Prone checks for unsafe coroutine use. [CONTEXT7 DOCS] [OFFICIAL SOURCE]

## API and semantic comparison

| Concern | Commands v2 | Commands v3 | Port consequence for this repo | Evidence |
|---|---|---|---|---|
| Core type | org.wpilib.command2.Command after 2027 import | org.wpilib.command3.Command | Vendor and robot command types are not assignment-compatible. | OFFICIAL SOURCE |
| Subsystem model | Subsystem/SubsystemBase | Mechanism | Every subsystem inheritance boundary changes in a v3 port. CTRE command drivetrain and PathPlanner remain v2. | OFFICIAL SOURCE; VENDOR DOCS |
| Scheduler | CommandScheduler singleton; v2 lifecycle | Scheduler; coroutine trees, queued top-level work, priorities, scopes | Robot.robotPeriodic must call only the selected framework scheduler. Tests that inspect requiring(subsystem) need new tree/list assertions. | CONTEXT7 DOCS; OFFICIAL SOURCE |
| Execution body | initialize/execute/isFinished/end or factories | A normal-looking function that explicitly yields or awaits | Infinite or periodic loops without coroutine.yield can hang the robot unrecoverably. | OFFICIAL SOURCE |
| Requirements | Static union on a command/composition | Mechanism ownership; dynamic child use is possible; built-in groups still own all child mechanisms | Hand-written coroutine sequences may release mechanisms between stages, changing existing v2 ownership behavior. Built-in sequence/parallel is safer for parity. | OFFICIAL SOURCE |
| Conflict policy | interruption behavior flag | Integer priorities; equal/higher priority interrupts, lower priority is rejected; suspend/resume is available | Avoid priorities in the first slice; reproduce v2 equal-priority interruption first. | OFFICIAL SOURCE |
| Triggers | onTrue/onFalse/whileTrue/toggle; global unless custom loops/lifecycle code | Similar bindings plus retryWhileTrue/retryWhileFalse, risingEdge/fallingEdge, and automatic command/opmode scopes | Maintained operator switches must keep whileTrue non-restart semantics unless tests prove retry behavior is desired. | CONTEXT7 DOCS; OFFICIAL SOURCE |
| Cancellation | end(true) runs; composed wrappers mediate cleanup | Canceled continuation is never remounted; onCancel/whenCanceled runs; descendants are canceled | Cleanup placed after a coroutine loop does not run on cancellation. Explicit whenCanceled is mandatory. | OFFICIAL SOURCE |
| Natural completion | end(false) runs | Command body returns normally; onCancel is not called | A v2 finallyDo that runs for both outcomes needs cleanup after the body plus the same idempotent cleanup in whenCanceled. | OFFICIAL SOURCE; INFERRED |
| Timeout | withTimeout decorator; interrupted flag propagates through wrapper | withTimeout is a race against waitFor; losing command is canceled | Verify timeout cleanup and reason telemetry; do not assume a v2 finallyDo interrupted flag has a direct equivalent. | CONTEXT7 DOCS; OFFICIAL SOURCE |
| Sequence | Commands.sequence owns union of requirements | Command.sequence / andThen built-in group owns all child mechanisms for full duration | Built-in v3 sequence best preserves v2 mutual exclusion but preserves the existing “owned but idle” behavior too. | OFFICIAL SOURCE |
| Parallel-all | Commands.parallel ends when all finish | Command.parallel marks children required and ends when all finish | Mechanical if requirements are disjoint and cleanup is explicit. | CONTEXT7 DOCS; OFFICIAL SOURCE |
| Race | Commands.race ends on first; interrupts others | Command.race marks all children optional and cancels losers | Timeout and competition behavior is similar at a high level, but tree cancellation timing needs tests. | CONTEXT7 DOCS; OFFICIAL SOURCE |
| Deadline | Commands.deadline(deadline, others) | Parallel builder represents required/deadline children and optional followers | PathPlanner deadline groups must stay v2 while PathPlanner returns v2 Command. A later v3 replacement needs completion/interruption tests. | CONTEXT7 DOCS; OFFICIAL SOURCE; VENDOR DOCS |
| Default command | Subsystem default must require subsystem | Mechanism defaults are scoped and must require exactly that mechanism; every mechanism starts with a lowest-priority idle default | Superstructure’s default operator policy cannot be a direct mechanical port until it requires only Superstructure and its child scheduling policy is redesigned. | OFFICIAL SOURCE |
| Interruption cleanup | end(interrupted) or finallyDo(BooleanConsumer) | onCancel/whenCanceled is cancellation-only | operatorPolicyCmd.finallyDo and autonomous telemetry finalizers are semantic ports, not renames. | OFFICIAL SOURCE |
| Disabled behavior | Commands cancel on disable by default; ignoringDisable(true) opts in | Current v3 public source has no ignoringDisable/runsWhenDisabled surface and no DriverStation disabled gate in Scheduler; design centers on opmode scopes | Exact parity is UNKNOWN. The repo’s disabled Idle binding cannot be ported until the release documents and tests the LoggedRobot/TimedRobot integration. | CONTEXT7 DOCS for v2; OFFICIAL SOURCE; INFERRED; UNKNOWN |
| Naming | Optional with withName | Required before construction; groups can auto-name | Helpful telemetry improvement; factory methods must provide stable names. | OFFICIAL SOURCE |
| Scheduling child commands | v2 composition/proxy rules; ScheduleCommand exists | await/fork are visible in command tree; children cannot outlive parent; no ScheduleCommand analogue | Direct mAgitateCommand scheduling in Superstructure should become a structured child or remain v2 initially. | OFFICIAL SOURCE |
| Threading | Single-thread assumptions | Explicitly single thread; virtual/multithreaded scheduler use can crash the JVM | Existing simulation loop/thread harness needs close review before v3 testing. | OFFICIAL SOURCE |
| Compile safety | Runtime checks for composed-command misuse and requirements | Staged builder types, forced names, NoDiscard, unsafe coroutine compiler checks | Several current repeated-wrapper/composition failures could move earlier, but vendor boundaries remain compile blockers. | CONTEXT7 DOCS; OFFICIAL SOURCE |

## Repo comparison

### Keep v2 initially

- PathPlanner AutoBuilder, NamedCommands, PathPlannerAuto and chooser APIs return or accept org.wpilib.command2.Command. The resolved 2027.0.0-alpha-3 bytecode confirms this. [VENDOR DOCS] [EXPERIMENTALLY VERIFIED]
- CTRE CommandSwerveDrivetrain integrates with Commands v2 subsystem and SysId command types. [VENDOR DOCS] [EXPERIMENTALLY VERIFIED]
- The regression suite directly inspects CommandScheduler ownership and v2 composition rules. [EXPERIMENTALLY VERIFIED]
- The official vendordep conflict prevents a selective in-project v2/v3 mixture. [OFFICIAL SOURCE]

### Candidate v3 equivalents after the control port

| Existing pattern | Candidate v3 shape | Risk |
|---|---|---|
| Commands.runOnce(this::requestStow, this) | Mechanism command with a one-shot body and required name | Low |
| Commands.startEnd(start, stop, this) | Mechanism body plus explicit normal cleanup and whenCanceled cleanup | Medium |
| withTimeout | v3 withTimeout, with assertions for canceled cleanup | Medium |
| Commands.sequence | Built-in v3 sequence for v2-like full requirement ownership | Medium |
| Commands.deadline | Required/optional parallel builder after isolated scheduler tests | Medium-high |
| Trigger.onTrue/whileTrue | v3 Trigger equivalents; avoid retry variants initially | Medium |
| setDefaultCommand | Mechanism single-requirement default, scoped globally | Medium-high |
| finallyDo | No one-token equivalent: natural and cancellation paths must both call an idempotent cleanup | High |

## Specific hazards from this repo

- operatorPolicyCmd.finallyDo(clearOperatorRequest): **CHANGES**. whenCanceled covers interruption only; normal completion cleanup must remain explicit. The default command/scoped scheduling architecture also changes. [OFFICIAL SOURCE] [INFERRED]
- Robot autonomous wrapper finallyDo(interrupted): **CHANGES**. V3 has separate natural completion and cancellation hooks, and run IDs eliminate some wrapper-identity telemetry needs but not end-reason behavior. [OFFICIAL SOURCE] [INFERRED]
- Repeated autonomous entry: **UNKNOWN** until v3 scheduler tests recreate the same selected command twice. V3 allocates a new run/coroutine ID per schedule, but the PathPlanner command is v2 and cannot cross the boundary. [OFFICIAL SOURCE] [VENDOR DOCS]
- Disabled Idle behavior: **UNKNOWN** and potentially **BREAKS** under LoggedRobot because v3 does not expose the v2 ignoringDisable contract in current source. [OFFICIAL SOURCE] [INFERRED]
- Direct CommandScheduler.requiring(intake): **BREAKS mechanically**. V3 exposes lists/tree ownership per Mechanism, not the same singleton method contract. [OFFICIAL SOURCE]
- mAgitateCommand manual scheduling/cancel: **CHANGES** toward structured fork/await or scoped scheduling. [OFFICIAL SOURCE]

## Sources

Context7 pages: commands-v2/index.rst, commands.rst, command-scheduler.rst, command-compositions.rst, binding-commands-to-triggers.rst, yearly-changelog.rst, and 3rd-party-libraries.rst.

Official source:

- https://github.com/wpilibsuite/allwpilib/blob/main/design-docs/commands-v3.md
- https://github.com/wpilibsuite/allwpilib/tree/main/commandsv3
- https://github.com/wpilibsuite/allwpilib/blob/main/commandsv3/CommandsV3.json
- https://github.com/wpilibsuite/allwpilib/blob/main/commandsv2/CommandsV2.json
- https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/org/wpilib/examples/hatchbotcmdv3

