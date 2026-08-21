# Intake Jam-Detection Migration Design

Date: 2026-07-17
Status: Approved

## Context

`CLAUDE.md`'s Backlog has carried "Decide the NamedCommands migration" since the Superstructure v2 refactor (2026-07-17): `"Intake Start Sequence"` (registered in `RobotContainer.java`) still uses a legacy, mechanism-level `intake.intakeJamReverse()` Command instead of `superstructure.intakeCmd()`, specifically because jam-reverse stall protection lives inside that one Command and has no other home — swapping the registration would silently drop stall protection from autos.

This design gives jam detection a permanent home (1678-style, mechanism-level inside `Intake`), which removes the blocker and lets the NamedCommand migration finally happen. Investigating that migration surfaced two additional, real findings that are folded into this design rather than deferred:

1. A genuine agitator-direction bug: `"Intake Start Sequence"` commands `Shooter.Agitate.IN` during autonomous intaking; `Superstructure.INTAKING` (the teleop path) already correctly commands `Agitate.OUT`. Per mentor feedback from competition, `OUT` is the correct direction to prevent jams during intaking — `IN` is reserved for the shooting feed phase (`Superstructure.SHOOTING`). **This specific claim (which direction is correct) is cited from conversational mentor feedback, not a written source, and is not independently verifiable from code — flagged as an unverified hypothesis below, matching this project's existing convention for physical-hardware facts (see the CANcoder-offset Backlog item).**
2. A likely pre-existing bug in the current `"Intake Start Sequence"`: because it calls `Intake.setRoller()` directly (bypassing `Superstructure`) via one-shot (`runOnce`/`startEnd`-style) Command steps, and `Superstructure.periodic()`'s `OFF` case unconditionally calls `intake.setRoller(Roller.STOP)` every tick (confirmed against `CommandScheduler.run()`'s source — all subsystem `periodic()` calls run before any scheduled command's `execute()`, every tick, and `StartEndCommand`'s `execute()` is a verified no-op), the roller almost certainly cannot sustain an `INTAKE` command for more than one ~20ms tick during autos today. This is very likely the first real instance of the "two writers, same tick" gap already recorded as an accepted risk in `docs/claudex/architecture.md`. It predates this session and is fixed as a side effect of Section 3 below.

## Section 1 — Mechanism-level jam recovery in `Intake`

**Problem:** Jam detection/recovery (`isJammed()`, the eject-pulse-then-resume pattern) currently only exists inside the `intakeJamReverse()` Command. Any other caller of `Intake.setRoller(Roller.INTAKE)` — including `Superstructure.INTAKING`, which drives teleop's `intakeCmd()` today with zero jam awareness — gets no protection at all.

**Change:** `Intake.java` gains:
- `private boolean jamRecoveryActive = false;`
- `private double jamRecoveryStartTimestamp = 0.0;`
- `private static final double kJamRecoveryPulseSeconds = 0.3;` (unchanged value, carried over from `intakeJamReverse()`'s existing `Commands.waitSeconds(0.3)`)

`setRoller(Roller state)` becomes the single interception point for jam handling:

```java
public void setRoller(Roller state) {
  if (state != Roller.INTAKE) {
    // STOP or EJECT always wins immediately, aborting any in-progress recovery pulse.
    jamRecoveryActive = false;
    applyRoller(state);
    return;
  }

  if (!jamRecoveryActive && isJammed()) {
    jamRecoveryActive = true;
    jamRecoveryStartTimestamp = Timer.getFPGATimestamp();
  }

  if (jamRecoveryActive) {
    if (Timer.getFPGATimestamp() - jamRecoveryStartTimestamp < kJamRecoveryPulseSeconds) {
      applyRoller(Roller.EJECT); // recovery pulse in progress
      return;
    }
    jamRecoveryActive = false; // pulse elapsed, fall through to resume INTAKE
  }

  applyRoller(Roller.INTAKE);
}

private void applyRoller(Roller state) {
  if (state == Roller.STOP) {
    io.setRollerVoltage(0);
  } else {
    io.setRollerVelocity(RotationsPerSecond.of(state.getRPM() / 60.0).in(RadiansPerSecond));
  }
  currentRoller = state;
  Logger.recordOutput("Intake/TargetState", state);
  Logger.recordOutput("Intake/JamRecoveryActive", jamRecoveryActive);
}
```

`getRollerState()` (unchanged signature) continues to return `currentRoller`, which now reflects what was actually **applied** (e.g. `EJECT` mid-pulse), not the raw request — consistent with its existing contract from the Test B work (`docs/claudex/history.md`).

Because `Superstructure.periodic()` already calls `intake.setRoller(Roller.INTAKE)` every tick during `INTAKING`, this makes jam recovery automatic for that caller with zero changes needed in `Superstructure.java`. If a jam persists after a recovery pulse completes, the very next `INTAKE` request re-triggers `isJammed()` and starts a new pulse — matching `intakeJamReverse()`'s old `.repeatedly()` semantics.

## Section 2 — Cleanup

**Problem:** `intakeJamReverse()` (the Command) becomes redundant once jam protection is automatic inside `setRoller()`. `Intake.intake()` — the only other caller of `intakeJamReverse()` — is dead code; nothing in `RobotContainer`, `OperatorControls`, or `Superstructure` calls it.

**Change:**
- Delete `Intake.intakeJamReverse()`.
- Delete `Intake.intake()` (dead code, exists only to call the method being deleted).
- `isJammed()` stays — now used internally by `setRoller()`.
- `kJamStatorCurrentAmps`/`kJamVelocityThresholdRadPerSec` constants stay untouched.

## Section 3 — NamedCommands fix

**Problem:** `"Intake Start Sequence"` (`RobotContainer.java`) uses the legacy `Commands.parallel(intake.intakeJamReverse(), Commands.runOnce(() -> shooter.setAgitator(Agitate.IN)))`. Per Section 1/2, `intakeJamReverse()` no longer exists, and per the Context section's finding #1, `Agitate.IN` is the wrong direction for intaking.

**Change:**
- `"Intake Start Sequence"` registration becomes `superstructure.intakeCmd()`. This is the correct fix, not just a style unification: routing through `Superstructure` makes `Superstructure.periodic()` the single, consistent, every-tick writer to `intake.setRoller()` during `INTAKING` (avoiding the Context finding #2 bug), and picks up `INTAKING`'s existing, already-correct `Agitate.OUT` automatically — no separate agitator line needed in the registration at all.
- `"Intake Stop"` registration becomes `superstructure.stowCmd()`. A narrower `intake.stopRoller()`-only stop was considered and rejected: since `"Intake Start Sequence"` now sets `Superstructure`'s `mWantedState` to `INTAKING` (which persists independently of Command scheduling state), only a command that calls `requestStow()` — i.e. goes through `Superstructure` — actually stops the re-assertion. `superstructure.stowCmd()` also spins the shooter to idle RPM and moves the intake pivot down (accepted side effect, not previously present in the narrow `"Intake Stop"`).
- Delete the now-obsolete `RobotContainer.java` comments explaining why these NamedCommands stayed on legacy mechanism-level calls (the reason no longer applies).

## Section 4 — Testing

**Problem:** `IntakeIOSim`'s roller model (`DCMotorSim`, no load/obstruction) cannot produce a real stall — `rollerStatorCurrentAmps` never stays high while `rollerVelocityRadsPerSec` stays near zero without an external hook. `IntakeIOSim.updateInputs()` also reaches into the static field `RobotContainer.drivetrain` (line 111), so `Intake` cannot be constructed/tested independently of a full `Robot` boot the way a narrower unit test might otherwise attempt.

**Change:**
- New `src/test/java/frc/robot/subsystems/intake/IntakeJamRecoveryTest.java`, package `frc.robot.subsystems.intake` (needed for package-private access to the test hook below).
- Same harness shape as `RobotLifecycleTest`/`SuperstructureEjectingTest`: real `Robot.startCompetition()` loop on a daemon thread, `DriverStationSim`/`SimHooks` driving ticks, single `@Test` method (same `forkEvery=1`-per-class reasoning — `Intake` is a Singleton with no reset hook).
- `Intake.java` gains a package-private test-only hook, entirely self-contained (does not touch `IntakeIO`/`IntakeIOSim`):
  ```java
  private boolean testJamOverrideActive = false;
  private double testJamStatorCurrentAmps;
  private double testJamVelocityRadsPerSec;

  /** Test-only: forces isJammed()'s inputs, bypassing physics IntakeIOSim can't model. */
  void forceJamConditionForTest(double statorCurrentAmps, double velocityRadsPerSec) {
    testJamOverrideActive = true;
    testJamStatorCurrentAmps = statorCurrentAmps;
    testJamVelocityRadsPerSec = velocityRadsPerSec;
  }

  /** Test-only: stops overriding sensor readings, resumes real IntakeIOSim physics. */
  void clearJamOverrideForTest() {
    testJamOverrideActive = false;
  }
  ```
  Applied inside `periodic()`, immediately after `io.updateInputs(inputs)`:
  ```java
  io.updateInputs(inputs);
  if (testJamOverrideActive) {
    inputs.rollerStatorCurrentAmps = testJamStatorCurrentAmps;
    inputs.rollerVelocityRadsPerSec = testJamVelocityRadsPerSec;
  }
  Logger.processInputs("Intake", inputs);
  ```
- **Revised during implementation:** the test drives `INTAKING` through `Superstructure.requestIntake()`/`requestStow()` rather than calling `Intake.setRoller()` directly. A direct-call version was tried first and failed deterministically — `Superstructure` is constructed as part of the required `Robot` boot regardless, sits in its default `OFF` state for the whole test since nothing moves it, and `OFF`'s unconditional every-tick `intake.setRoller(Roller.STOP)` fights any direct call the test makes. Routing through `Superstructure` instead makes it the sole writer throughout, matching production and turning every assertion into a stable state.
- Test sequence: `requestIntake()`, assert baseline `Roller.INTAKE` (no jam); force jam sensor values, assert `Roller.EJECT` (pulse fired automatically via Superstructure's every-tick re-assertion); clear the jam and step past `kJamRecoveryPulseSeconds`, assert `Roller.INTAKE` again (resumed, stable); force a jam again, confirm a new pulse, then `requestStow()` mid-pulse and assert `Roller.STOP` immediately (proves the abort-immediately behavior from Section 1, triggered through the real caller path).
- Negative control required before considering this done, matching Test A/B rigor: temporarily flip one assertion, confirm a real `AssertionFailedError`, revert, reconfirm green.

## Out of scope

- `Superstructure.SHOOTING`'s `Agitate.IN` (feed phase) — unaffected by this design; it's a different use case (feeding the shooter, not intaking) than the direction question resolved in Section 3.
- Real-robot confirmation of the `IN`/`OUT` direction claim itself — tracked as a Backlog follow-up, not blocking this migration (the migration is correct regardless of which direction turns out to be right, since it just makes the auto path match the teleop path — if the teleop direction is ever found wrong, both paths get fixed together in one place).
