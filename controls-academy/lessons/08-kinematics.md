# Lesson 8: Kinematics

## Learning objective
Wire `SwerveDriveKinematics` between Lesson 7's `ChassisSpeeds` and Lesson 6's 4 modules —
inverse kinematics (chassis → module states) to drive, forward kinematics (module states →
chassis speeds) to check yourself, and wheel-speed desaturation as the last defense against an
unachievable command.

## Concepts introduced
- `SwerveDriveKinematics.toSwerveModuleStates(ChassisSpeeds)` (inverse kinematics)
- `SwerveDriveKinematics.toChassisSpeeds(SwerveModuleState...)` (forward kinematics) as a
  correctness check — round-tripping should recover the original command
- `SwerveDriveKinematics.desaturateWheelSpeeds()` — the exact algorithm behind this repo's own
  `CommandSwerveDrivetrain.sanitizeAutoSpeeds()` (seventeenth-session production fix)

## Existing code to reuse
`CommandSwerveDrivetrain.sanitizeAutoSpeeds(ChassisSpeeds)` is the direct reference: it calls
`getKinematics().toSwerveModuleStates(speeds)` → `desaturateWheelSpeeds()` →
`getKinematics().toChassisSpeeds(states)`, exactly the round-trip this lesson teaches. Read it
(and `CommandSwerveDrivetrainSanitizeSpeedsTest.java` for the TDD example) rather than reinventing
the pattern — the goal is recognizing it, not rederiving it from scratch.

## New simulation command / demo
`academy.chassis.KinematicsRoundTripTest` (JUnit): construct a `SwerveDriveKinematics` from 4
academy module positions (your own, not copied from `TunerConstants`), drive an over-saturating
`ChassisSpeeds` through inverse kinematics, desaturate, then forward-kinematics back and confirm
the result matches the desaturated command (not the original oversized one).

## What to observe in Phoenix Tuner X
N/A (pure math + Lesson 6's simulated modules).

## What to observe in AdvantageScope
Per-module speed before and after desaturation on one plot — confirm all 4 modules get scaled by
the *same* factor (desaturation preserves the commanded motion's shape, just scales its
magnitude) rather than clipped independently, which would distort the intended path.

## Common failure modes
- Getting module position signs wrong when constructing `SwerveDriveKinematics` (front-left vs.
  back-right mirror-imaged) — produces a robot that spins in place instead of translating.
- Desaturating per-module independently instead of by one shared scale factor — breaks the
  intended direction of travel, not just its speed.
- Confusing "desaturation" with "the FF/PID clamp from Lesson 3-ish territory" — they solve
  different problems: desaturation is a kinematic ceiling (can't exceed max wheel speed),
  distinct from a controller output clamp.

## Small experiments to build intuition
1. Command a `ChassisSpeeds` requiring one corner module to exceed max speed — confirm
   desaturation scales *all four* modules down proportionally, and that the robot's direction of
   travel (not just speed) survives the round trip.
2. Verify the forward-kinematics round trip recovers the *desaturated* speeds, not the original
   oversized command — this is the actual behavior your production `sanitizeAutoSpeeds()` exists
   to guarantee.
3. Try a 3-module (deliberately broken) kinematics configuration and observe how forward
   kinematics stops round-tripping correctly — cheap way to see why module geometry has to be
   exactly right.

## Success criteria
`KinematicsRoundTripTest` passes, and you can point to the exact three-call sequence
(`toSwerveModuleStates` → `desaturateWheelSpeeds` → `toChassisSpeeds`) from memory, matching
`sanitizeAutoSpeeds()`. Move to Lesson 9 once you have a full 4-module chassis driving from a
single `ChassisSpeeds` command.
