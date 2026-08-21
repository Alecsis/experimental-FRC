# Lesson 6: Swerve Module Behavior

## Learning objective
Compose Lessons 2-5 (drive velocity + feedforward, steer position) into one real unit: a swerve
module that takes a `SwerveModuleState` and drives both motors to match it. This is the first
lesson producing a genuinely reusable artifact — Lessons 7-14 all sit on top of 4 of these.

## Concepts introduced
- `SwerveModuleState` (speed + angle) as the module-level command interface
- Cosine compensation / state optimization (`SwerveModuleState.optimize()`) — why a module should
  prefer flipping 180° + reversing drive direction over rotating the long way
- Module-level closed loop composition: two independent control loops (drive, steer) driven from
  one shared command

## Existing code to reuse
`MapleSimSwerveDrivetrain`'s `SwerveModuleSimulationConfig` construction (built once from module 0
and applied to all four, since modules are symmetric) — the reference for how this repo already
builds a module-level sim config from `TunerConstants`. Do not import `TunerConstants` values
directly into the academy module (per `CLAUDE.md`: never copy numbers out of reference/production
code) — invent your own plausible academy-scale constants instead.

## New simulation command / demo
`academy.module.SwerveModule` (composes `academy.pid`/`academy.feedforward`'s drive loop with
`academy.steer`'s position loop). `academy.module.ModuleOptimizeTest` (JUnit): command a state
requiring >90° of rotation, confirm `optimize()` produces the flip-and-reverse behavior and that
it's cheaper (less total rotation) than the naive path.

## What to observe in Phoenix Tuner X
N/A (simulation).

## What to observe in AdvantageScope
`Academy/Module/CommandedState` vs `Academy/Module/MeasuredState` (speed+angle pair), plus a
before/after optimize() comparison of total steer travel for the same requested state.

## Common failure modes
- Forgetting to optimize before commanding — the module still "works" but takes needlessly long
  paths, especially visible on repeated small-direction-change commands (a drivetrain "wobble").
- Applying cosine compensation to the wrong loop (it scales *drive* speed by the *steer* error's
  cosine, not the reverse) — easy to swap by mistake when writing it for the first time.
- Treating the module as a single control loop instead of two independent ones sharing a command
  — coupling them incorrectly can make debugging steer issues look like drive issues.

## Small experiments to build intuition
1. Command states requiring 179° vs 181° of raw rotation — confirm `optimize()` picks the flip
   for the 181° case and not the 179° case (the crossover is the whole point of the algorithm).
2. Disable cosine compensation and command a large steer error simultaneously with a drive speed
   — observe the drive motor briefly "fighting" the module's actual direction of travel.
3. Chain a sequence of small state changes (like a real teleop joystick trace) and confirm no
   spurious full-circle steer spins appear.

## Success criteria
`ModuleOptimizeTest` passes, and given any two `SwerveModuleState`s you can predict by hand
(before running it) which optimize() will choose the flip. Move to Lesson 7 once you have 4 of
these modules instantiated and independently commandable.
