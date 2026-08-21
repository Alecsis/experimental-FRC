# Lesson 5: Steer Motor Tuning

## Learning objective
Transfer Lessons 2-3's PID/feedforward intuition from *velocity* control to *position* control,
and understand why this repo runs different steer gains in sim vs. real hardware.

## Concepts introduced
- Position closed loop (`PositionVoltage`/`MotionMagicVoltage`) vs. velocity closed loop
- Continuous wrap-around for a steer axis (angle error should never be reported as e.g. 359°
  instead of -1°)
- Why sim and real steer gains can legitimately differ — not a tuning error, a numerical-stability
  necessity

## Existing code to reuse
`MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation()` — the concrete, already-shipped
example of exactly this concept: it overrides steer gains to `kP=70, kD=4.5` in sim (vs. real
`kP=100, kD=0.5`) purely for 200Hz numerical stability, guarded by `RobotBase.isReal()`. Read this
method before designing your own steer demo — it's the answer key.

## New simulation command / demo
`academy.steer.SteerPositionSim` — a CANcoder-backed position-control harness (reuses L1's
`TalonFXSimState` pattern plus a simulated absolute encoder). `academy.steer.SteerWrapTest`
(JUnit): command angle setpoints that cross the ±180° boundary, confirm the controller always
takes the short path.

## What to observe in Phoenix Tuner X
N/A (simulation). If real hardware becomes available before this lesson is revisited, this is the
first lesson with a genuine live-Tuner-X analogue — watching a real CANcoder-backed steer motor's
position PID response.

## What to observe in AdvantageScope
`Academy/Steer/CommandedAngle` vs `Academy/Steer/MeasuredAngle`, plus a derived
`Academy/Steer/WrapError` signal — confirm it never exceeds 180° in magnitude even when the raw
angle difference would.

## Common failure modes
- Naively subtracting angles (`target - measured`) without wrapping — produces spurious large
  errors and a Slot0 controller that spins the "long way around."
- Copy-pasting velocity-loop gains onto a position loop unchanged — position and velocity loops
  have different units and typically need very different gain magnitudes.
- Missing the sim-vs-real gain split entirely and being confused later (Lesson 6) why a module
  behaves differently than expected under `RobotBase.isReal()`.

## Small experiments to build intuition
1. Deliberately skip angle wrapping, command a setpoint just past ±180°, and watch the "long way
   around" spin — then add wrapping and confirm it takes the short path instead.
2. Push simulated steer kP toward the real-hardware value (100) at 200Hz and observe the numerical
   instability `MapleSimSwerveDrivetrain` was written to avoid — reproduce the problem it solves.
3. Command a rapid back-and-forth angle setpoint (a "shimmy") and see how kD affects overshoot
   at direction reversals, same D-term intuition from Lesson 2 applied to a new loop type.

## Success criteria
`SteerWrapTest` passes for setpoints on both sides of the ±180° boundary, and you can state in one
sentence why `MapleSimSwerveDrivetrain` uses different steer gains for sim vs. real. Move to
Lesson 6 once both hold.
