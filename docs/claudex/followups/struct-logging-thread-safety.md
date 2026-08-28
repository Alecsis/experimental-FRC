# Follow-up: cross-thread AdvantageKit Struct logging races

**Status:** OPEN — deliberately NOT fixed on `feature/hardware-bringup-tuning`.
**Scope:** pre-existing, baseline, autonomous-critical logging. Needs its own focused branch and
its own regression run.
**Found:** 2026-08-28, during the hardware bring-up / shooter tuning review.

## Mechanism

`org.littletonrobotics.junction.LogTable` serialises every struct-backed output through a shared
buffer keyed by struct **type string** (akit-java 26.0.2, `LogTable.java`):

```java
private final Map<String, StructBuffer<?>> structBuffers;   // line 43, plain HashMap
...
if (!structBuffers.containsKey(struct.getTypeString())) {   // line 646
  structBuffers.put(struct.getTypeString(), StructBuffer.create(struct));
}
StructBuffer<T> buffer = (StructBuffer<T>) structBuffers.get(struct.getTypeString());
ByteBuffer bb = buffer.write(value);
byte[] array = new byte[bb.position()];
bb.position(0);
bb.get(array);
```

Three consequences:

1. The map is a plain `HashMap` and is shared with every subtable (`LogTable.java:106`).
2. `Pose2d` and `Pose2d[]` map to the **same** buffer instance (same type string), so scalar and
   array writes of one geometry type collide with each other.
3. The race window is the whole `put()` — pack, then `position(0)`, then `get(array)` — not just
   `pack()`. There is no synchronisation anywhere on this path.

Two threads calling `Logger.recordOutput` with the same struct type therefore corrupt each other's
buffer position. The observed symptom is a `BufferOverflowException` that escapes `robotPeriodic()`
and terminates `startCompetition()`:

```
java.nio.BufferOverflowException
  at java.base/java.nio.DirectByteBuffer.putDouble
  at edu.wpi.first.math.geometry.struct.Translation2dStruct.pack
  at edu.wpi.first.math.geometry.struct.Pose2dStruct.pack
  at edu.wpi.first.util.struct.StructBuffer.write
  at org.littletonrobotics.junction.LogTable.put
```

## The two writer threads

- **CTRE Phoenix odometry thread** — `Telemetry.telemeterize()` is registered via
  `drivetrain.registerTelemetry(logger::telemeterize)` and CTRE invokes it from the odometry thread
  at roughly 250 Hz. Already documented in-repo at
  `CommandSwerveDrivetrain.java:665-666` ("the CTRE odometry thread's ~250Hz cadence that
  Telemetry.java's SwerveStates/Measured and SwerveStates/Setpoints run on").
- **Robot thread** — subsystem `periodic()`, command `execute()`, and the PathPlanner logging
  callbacks registered in `CommandSwerveDrivetrain.configureAutoBuilder()`.

## Inventory

| Site | Struct type | Thread | Verdict |
| --- | --- | --- | --- |
| `Telemetry.java:130` `SwerveStates/Measured` | `SwerveModuleState[]` | odometry | RACE |
| `Telemetry.java:131` `SwerveStates/Setpoints` | `SwerveModuleState[]` | odometry | RACE |
| `Telemetry.java:132` `Odometry/Robot` | `Pose2d` | odometry | RACE |
| `Telemetry.java:133` `DriveState/Speeds` | `ChassisSpeeds` | odometry | RACE |
| `CommandSwerveDrivetrain.java:425` `Odometry/TrajectorySetpoint` | `Pose2d` | robot, **every autonomous tick** | RACE vs `Odometry/Robot` |
| `CommandSwerveDrivetrain.java:417` `Odometry/Trajectory` | `Pose2d[]` | robot, each path start | RACE vs `Odometry/Robot` (same type key) |
| `LoggingHolonomicDriveController.java:40,41,42` | `ChassisSpeeds` x3 | robot, **every autonomous tick** | RACE vs `DriveState/Speeds` |
| `CommandSwerveDrivetrain.java:655` `SwerveStates/Setpoints` (empty) | `SwerveModuleState[]` | robot, `periodic()`, disabled only | RACE vs the two above |
| `CommandSwerveDrivetrain.java:727` `Drivetrain/SimSpawnPose` | `Pose2d` | robot, sim + disabled, once | RACE (sim only) |
| `CommandSwerveDrivetrain.java:363` `Trajectory/BoundedCommandedSpeeds` | `double[]` | robot | SAFE, not struct-backed |
| `Intake.java:324` `Intake/TargetPivotAngle` | `Angle` measure | robot | SAFE, serialises to a double |
| `Telemetry.java:46` `drivePose` | NT `StructPublisher<Pose2d>` | odometry | SAFE, NT publishers own a private buffer |
| everything else in `src/main` | primitives / String / enum / `double[]` | robot | SAFE |

Highest priority, both live for the whole autonomous period:

1. `Pose2d`: `Odometry/TrajectorySetpoint` (50 Hz, robot) vs `Odometry/Robot` (~250 Hz, odometry).
2. `ChassisSpeeds`: three writes per tick from `LoggingHolonomicDriveController` (robot) vs
   `DriveState/Speeds` (~250 Hz, odometry).

## Why the gates never caught it

`SKILLS/run_headless_sim.py` boots the robot **disabled with no DriverStation attached**, so
PathPlanner never runs and the robot thread logs no structs at all. The baseline therefore has
exactly one struct-logging thread during that gate. The bring-up branch's first draft of
`ShooterTuning` was the first code to log a `Pose2d` from the robot thread *while disabled*, which
is why it surfaced there rather than in autonomous.

Measured through the same harness, with those `Pose2d` outputs present: **3 failures in 9 launches**
(and 0 in 6 raw-capture launches, which sampled a shorter window). Pristine base commit: **0 of 5**.
After switching `ShooterTuning` to scalar component outputs: **8 of 8 clean**.

That the autonomous races have not been observed is not evidence they cannot fire — the auto
regression suite runs each route once per invocation, and the collision window is microseconds wide.

## Recommended fix (separate branch)

Have `Telemetry.telemeterize()` store the `SwerveDriveState` and let
`CommandSwerveDrivetrain.periodic()` perform the `Logger.recordOutput` calls on the robot thread.
That collapses every row above to a single writer thread without changing what is logged. It does
change the sample cadence of `SwerveStates/*`, `Odometry/Robot` and `DriveState/Speeds` from ~250 Hz
to 50 Hz, which affects log-analysis tooling — so it needs the full regression suite plus a replay
check, not a drive-by edit.

Until then, the rule for new code is simple and is the one this branch follows:

> **Do not `Logger.recordOutput` a struct-backed WPILib geometry type from the robot thread.**
> Log scalar components instead.
