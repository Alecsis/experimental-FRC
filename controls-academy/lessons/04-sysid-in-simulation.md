# Lesson 4: SysId in Simulation

## Learning objective
Learn what SysId actually measures (it doesn't guess kS/kV/kA — it runs prescribed
quasistatic-ramp and dynamic-step tests and fits them) by running the *real* SysId pattern
against Lesson 1's toy motor, where the "true" kS/kV/kA are known because you set the sim
physics yourself — so you can grade the fit against ground truth.

## Concepts introduced
- Quasistatic ramp test (isolates kS/kV — near-zero acceleration) vs. dynamic step test
  (isolates kA — the added term once acceleration is nonzero)
- WPILib `SysIdRoutine` sequencing + CTRE `SignalLogger` CAN-synchronous capture — the same
  hybrid pattern this repo already runs in production
- `.hoot` → `.wpilog` conversion via Tuner X, and why CAN-synchronous logging matters (avoiding
  cross-signal timestamp skew that would corrupt the fit)

## Existing code to reuse
`CommandSwerveDrivetrain.java:109-161` — the three production `SysIdRoutine` instances
(translation/steer/rotation) are the **reference implementation**. Read them, don't copy them
wholesale; adapt the *pattern* (routine construction, `quasistatic`/`dynamic` command wiring,
`SignalLogger` start) to drive `academy.motor.SingleMotorSim` instead of the swerve drivetrain.
Also read `docs/SysId_Characterization_Checklist.md` for the documented mode-select-vs-run-modifier
gotcha (`leftBumper.onTrue` vs. held) before wiring your own bindings.

## New simulation command / demo
`academy.sysid.MotorSysIdTest` — wires a `SysIdRoutine` against `SingleMotorSim` with **known,
injected** kS/kV/kA in the sim physics itself (not the Slot0 config — the sim's true plant).
Run quasistatic-forward, quasistatic-reverse, dynamic-forward, dynamic-reverse. Export the
resulting log, run WPILib's own SysId analysis tool (or an equivalent open-source fit) on it, and
diff the fitted kS/kV/kA against the values you injected.

## What to observe in Phoenix Tuner X
Still simulation, but this is the first lesson where the *workflow* (not the signal) matters:
practice the `.hoot`→`.wpilog` conversion step in Tuner X even though the source is a sim log, so
the September real-hardware pass is mechanically familiar.

## What to observe in AdvantageScope
Voltage vs. velocity vs. acceleration for both test phases — the quasistatic phase should show a
near-linear voltage/velocity relationship (this *is* the kS/kV fit line, visually); the dynamic
phase's residual after subtracting the quasistatic fit is what isolates kA.

## Common failure modes
- Running dynamic-step tests too aggressively (too much commanded acceleration) such that
  current/voltage saturates mid-test — corrupts the kA fit exactly the way the checklist's
  mode-select gotcha corrupts a real run, just with a different root cause.
- Not isolating direction — forward and reverse fits can legitimately differ (asymmetric
  friction is real), collapsing them into one number hides that.
- Forgetting `SignalLogger.start()` before the routine runs — CTRE's own signals go uncaptured
  even if AdvantageKit's logger is running fine, since they're two independent logging paths.

## Small experiments to build intuition
1. Inject a *known* kS/kV/kA into `SingleMotorSim`, run the full 4-phase routine, and confirm the
   fitted values land within a small tolerance (e.g. 5%) of what you injected.
2. Intentionally shorten the quasistatic ramp duration — observe the kV fit degrade, building
   intuition for why SysId's default test durations aren't arbitrary.
3. Add simulated measurement noise and re-fit — see how much noise it takes before the fit
   meaningfully drifts, which calibrates how much you should trust a real noisy CAN bus's fit.

## Success criteria
Your fitted kS/kV/kA from `MotorSysIdTest` land within 5% of the values you injected into the sim
plant, and you can point to the exact log signal (voltage vs. velocity during quasistatic) that
produces the kV number. This is the last motor-level lesson — move to Lesson 5 (steer/position
control) once this holds.
