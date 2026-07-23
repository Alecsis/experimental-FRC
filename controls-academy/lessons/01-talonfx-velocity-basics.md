# Lesson 1: TalonFX Velocity Control Basics

## Learning objective
Understand how a `TalonFX` closed-loop velocity request actually flows: setpoint in →
`Slot0Configs` closed loop on the motor controller → measured velocity out. Build the minimal
harness every later lesson (2-6) will extend.

## Concepts introduced
- `TalonFXConfigurator` / `Slot0Configs` (kP/kI/kD/kS/kV/kA — kD/kI/kA all zero for this lesson)
- `VelocityVoltage` control request
- `TalonFXSimState` — how simulation injects a "measured" rotor velocity back into the device
- The scan-rate / status-signal refresh loop (`BaseStatusSignal.refreshAll`)

## Existing code to reuse
- `MapleSimSwerveDrivetrain.TalonFXMotorControllerSim.updateControlSignal()` — read (don't copy)
  as the reference for how this codebase already reads `TalonFXSimState.getMotorVoltageMeasure()`
  and feeds `SimulatedBattery.getBatteryVoltage()` back in.
- `TunerConstants.java`'s `driveGains` (`Slot0Configs`, line ~32) as a real-world example of what
  these constants look like once populated (`kP=0.1, kS=0.1, kV=0.124`, `kI=kD=kA=0` — the
  unedited Tuner X default, i.e. exactly this lesson's starting point).

## New simulation command / demo
`academy.motor.SingleMotorSim` — a bare `TalonFX` + `TalonFXSimState` with no gearbox, no
drivetrain, no CAN bus contention. `academy.motor.SingleMotorDemoTest` (JUnit): command a step
velocity setpoint (e.g. 0 → 20 rps), advance sim time in fixed ticks, log commanded vs. measured
velocity every tick via AdvantageKit.

## What to observe in Phoenix Tuner X
N/A for this lesson (pure simulation, no real CAN bus) — but open Tuner X's **Plot** tab against
a *mock* connection if available, to get comfortable with the velocity-vs-time plot UI before
Lesson 4 needs it against a live signal.

## What to observe in AdvantageScope
Two line-plot signals overlaid: `Academy/Motor/CommandedVelocity` vs `Academy/Motor/MeasuredVelocity`.
With `kP` alone (no kS/kV), expect a visible steady-state gap — the motor never quite reaches the
setpoint. That gap is this lesson's hook into Lesson 2.

## Common failure modes
- Forgetting to call `TalonFXSimState.setSupplyVoltage()` — the sim motor silently does nothing
  (`getMotorVoltage()` stays 0) if supply voltage is never set.
- Advancing sim time via `Timer`/wall clock instead of `SimHooks.stepTiming()` — produces
  nondeterministic step counts, making the step-response plot unreproducible run to run.
- Confusing rotor velocity (motor-side, before any gearing) with mechanism velocity — there's no
  gearbox in this lesson, so they're equal, but the distinction matters starting Lesson 6.

## Small experiments to build intuition
1. Set `kP=0` entirely — confirm the motor never moves (open-loop with zero command is zero).
2. Double `kP` — confirm time-to-90%-of-setpoint roughly halves, steady-state gap roughly halves.
3. Command a *negative* velocity step from a positive starting velocity — confirm symmetric
   behavior (no sign-handling bugs in the harness).

## Success criteria
`SingleMotorDemoTest` passes deterministically 3 runs in a row (same tick count each run), and
you can explain in one sentence *why* pure-`kP` control has a nonzero steady-state error at a
nonzero setpoint. Move to Lesson 2 once both hold.
