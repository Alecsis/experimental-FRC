# Controls Academy

A self-paced controls curriculum, built alongside the production robot codebase during the
pre-September hardware-learning phase. It never modifies `src/main/java/frc/robot` — it is a
separate Gradle subproject that only runs in desktop simulation.

## Why this exists

Team 4935's production robot-code work is currently blocked on real-hardware SysId data
(see `docs/Path_Following_Tuning_Readiness_Audit.md`'s September Tuning Playbook). Rather than
sit idle, this curriculum rebuilds the same control concepts — TalonFX velocity control, PID,
feedforward, SysId, swerve modules, kinematics, odometry, pose estimation, PathPlanner, and
AdvantageScope debugging — from first principles, in simulation, on a minimal standalone stack.
The goal is that by the time hardware access returns, the September playbook's steps (SysId →
apply FF → retune drive Slot0 → chassis PID → discretization) are *familiar*, not first-time.

## Where this lives

```
controls-academy/
  build.gradle                 new Gradle subproject: WPILib sim + Phoenix6 sim + maple-sim,
                                no roboRIO/athena deploy target — desktop simulation only
  README.md                    this file
  lessons/
    01-talonfx-velocity-basics.md
    02-pid-intuition.md
    03-feedforward.md
    04-sysid-in-simulation.md
    05-steer-motor-tuning.md
    06-swerve-module-behavior.md
    07-chassis-speeds.md
    08-kinematics.md
    09-odometry.md
    10-pose-estimation.md
    11-pathplanner-controller.md
    12-autonomous-path-following.md
    13-debugging-with-advantagescope.md
    14-full-autonomous-tuning-workflow.md
  src/main/java/academy/...    cumulative demo code, one growing package tree
  src/test/java/academy/...    JUnit entry points — how a lesson is actually "run"
```

`settings.gradle` gains one line (`include ':controls-academy'`) when the subproject is
scaffolded. Root project (`compileJava`, `test`, `simulateJava` against `frc.robot`) is
unaffected. Production files (`TunerConstants.java`, `CommandSwerveDrivetrain.java`, etc.) are
referenced as **read-only teaching examples** throughout — never copied wholesale, never edited.

**Known open risk (deferred to first implementation session, not solved here):** whether
GradleRIO's vendordep resolution (WPILib/Phoenix6/maple-sim JSONs under `vendordeps/`) attaches
cleanly to a second subproject, or needs its own copies. Verify this before lesson 1's demo code
is written — it's infrastructure, not a lesson.

## Code architecture: cumulative, not disconnected

Each lesson's demo class is the substrate the next lesson edits or wraps, mirroring how the real
stack composes (single motor → module → swerve chassis → autonomous path):

```
academy.motor        (L1)  bare TalonFXSimState, no drivetrain
academy.pid          (L2)  wraps L1, sweeps P/I/D
academy.feedforward  (L3)  adds SimpleMotorFeedforward to L1/L2
academy.sysid        (L4)  runs a SysIdRoutine against L1's motor
academy.steer        (L5)  position control, same PID/FF ideas, CANcoder-backed
academy.module       (L6)  composes L2/L3/L5 into one SwerveModule
academy.chassis       (L7-L8)  ChassisSpeeds + kinematics over 4 of L6's modules
academy.odometry     (L9-L10)  pose integration + fused pose estimation
academy.auto         (L11-L14) PathPlanner controller, path following, logging, full workflow
```

A bug introduced early (e.g. in L1's motor sim) will ripple forward by design — that's
intentional and matches the real robot's dependency graph. Fix it at the source, don't patch
downstream.

## Lesson ordering rationale

The sequence is deliberately bottom-up and matches the September Tuning Playbook's own order
(SysId → feedforward/gains → chassis-level control → autonomous), so finishing the academy
*is* a dry run of that playbook:

1. **L1-L4** (motor-level: velocity control, PID, feedforward, SysId) establish the single-motor
   vocabulary everything else depends on.
2. **L5-L6** (steer + module) extend that vocabulary to position control and compose the first
   real hardware unit (a swerve module).
3. **L7-L10** (chassis speeds → kinematics → odometry → pose estimation) build the math layer
   that turns 4 modules into a drivable, localized chassis — no path-following yet.
4. **L11-L14** (PathPlanner → path following → AdvantageScope → full workflow) close the loop:
   this is where the 16th-session unclamped-feedback finding and the trajectory-error
   instrumentation design (`docs/claudex/design/trajectory-error-instrumentation.md`) become
   guided exercises instead of read-only case studies.

No fixed calendar pacing — work through lessons opportunistically, one session at a time.
Each lesson's **Success criteria** section is the gate for moving to the next.

## Reused infrastructure (read, don't duplicate)

- `SKILLS/parse_akit_log.py` — importable Python `.wpilog` parser, used from L13 onward to read
  every prior lesson's own logs.
- `SKILLS/run_headless_sim.py` — pattern reference for boot-smoke-testing academy demos.
- `AutoRegressionTestBase` / `WpilogTrajectoryErrorReader` / `WpilogStallAnalyzer`
  (`src/test/java/frc/robot/`) — template for L12/L14's "did this actually complete" harnesses.
- `CommandSwerveDrivetrain.java`'s three `SysIdRoutine`s (`:109-161`) — L4's reference
  implementation.
- `MapleSimSwerveDrivetrain.java` — L6's module-sim-config reference.
- `docs/claudex/design/sim-timing-determinism.md` — required reading before L9/L10/L12: sim
  pose noise is real-time-clock-driven, so tracking-error comparisons need mocked suppliers, not
  live sim runs, for anything claiming determinism.
