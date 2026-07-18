# RobotMotor: shared TalonFX construction/telemetry wrapper

**Status:** approved 2026-07-18, implementation in progress.
**Scope:** Architecture Rule 1 (Strict Hardware Isolation) only. Rule 3 (Superstructure state machine) is untouched by this change.

## Why

Session nine's Understand-Anything pass over `temp_reference/Lynk 2026` surfaced a structural contrast: Lynk's `LynkMotor`/`LynkSubsystem` wrap CTRE hardware directly inside subsystem classes (no IO-interface split), versus this repo's `<X>IO`/`<X>IOReal`/`<X>IOSim` split. That contrast is observational, not a recommendation (see `docs/claudex/architecture.md`) — but it prompted a mentor-directed design pass on whether any of Lynk's ergonomics are worth adopting *without* giving up what the IO split buys us.

## Boundary decision

A literal copy of Lynk's pattern (subsystems holding hardware directly) would delete the seam that `SKILLS/run_headless_sim.py`, `RobotLifecycleTest`, and `SuperstructureEjectingTest` all depend on, and would drop AdvantageKit's `@AutoLog`/replay capability in favor of Lynk's `DogLog`-based direct logging. That trade-off was rejected. Instead:

- `RobotMotor` (this doc's subject) lives at `utility/RobotMotor.java`, joining `HootReplayBridge.java` as one of the small number of sanctioned vendor-touching utility files outside `*IOReal.java` itself.
- It is only ever instantiated **from inside** `*IOReal.java` files — never from a subsystem class (`Intake.java`, `Shooter.java`), never from an `*IOSim.java` file.
- `IntakeIO`/`ShooterIO` interfaces, `IntakeIOSim`/`ShooterIOSim`, and every `@AutoLog` inputs struct are unchanged by this refactor.
- Rule 1's wording in `claude.md` gets a one-line amendment blessing `RobotMotor` as a shared construction utility — the isolation boundary itself (no vendor types outside `*IOReal.java`/sanctioned utilities) doesn't move.

## Scope exclusion: the agitator (`TalonFXS`)

`ShooterIOReal.agitator` is a `TalonFXS` (NEO 550 via Talon FXS, `MotorArrangementValue.NEO550_JST`, mentor-confirmed session eight) — a distinct Phoenix 6 class from `TalonFX`, with its own config type (`TalonFXSConfiguration`). `RobotMotor extends TalonFX` cannot wrap it, and it is the only `TalonFXS` in the codebase. Building a parallel `RobotMotorFXS` (or a composition-based shared base) for a single motor instance was rejected as premature — `ShooterIOReal.agitator` stays a raw `TalonFXS`, unchanged. Revisit if a second `TalonFXS` motor is ever added.

## `RobotMotor` API

```java
package frc.robot.utility;

public class RobotMotor extends TalonFX {
  public static class MotorConfig {
    public String name;
    public int canId;
    public TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    public SafetyTripConfig safetyTrip;                 // null = disabled
  }

  public static class SafetyTripConfig {
    public Current statorCurrentThreshold;
    public Time trippedFor = Second.of(0);
    public Runnable onTrip = () -> {};

    public static SafetyTripConfig of(Current threshold, Time debounce) {
      SafetyTripConfig cfg = new SafetyTripConfig();
      cfg.statorCurrentThreshold = threshold;
      cfg.trippedFor = debounce;
      return cfg;
    }
  }

  private final String name;
  private final SafetyTripConfig tripConfig;
  private final Debouncer tripDebouncer;

  public RobotMotor(MotorConfig config) {
    super(config.canId);   // rio bus, matches every existing *IOReal.java TalonFX(id) call -- no motor
                            // migrated by this refactor needs a non-default CAN bus
    this.name = config.name;
    getConfigurator().apply(config.talonConfig);
    this.tripConfig = config.safetyTrip;
    this.tripDebouncer = tripConfig == null ? null
        : new Debouncer(tripConfig.trippedFor.in(Second), Debouncer.DebounceType.kRising);
  }

  /** Call once per IOReal tick. Purely observational -- never calls setControl(). Real current
   *  protection is already the firmware StatorCurrentLimitEnable config every *IOReal.java motor
   *  sets; this is early/soft warning + telemetry only. Never substitutes for subsystem-owned jam
   *  logic (Intake.isJammed() stays exactly as tested in the sixth-session fix). */
  public boolean pollSafetyTrip() {
    if (tripConfig == null) return false;
    boolean overCurrent = getStatorCurrent().getValue().abs(Amps) > tripConfig.statorCurrentThreshold.in(Amps);
    boolean tripped = tripDebouncer.calculate(overCurrent);
    if (tripped) tripConfig.onTrip.run();
    return tripped;
  }
}
```

Default `onTrip` (wherever wired below): `Logger.recordOutput(name + "/SafetyTripped", true)` + `DriverStation.reportWarning(...)`. Warn, don't act — the same "warn without stopping" shape as the existing CANcoder boot barrier (`CommandSwerveDrivetrain.logCancoderBootReadings()`), not a novel pattern in this codebase.

**Deliberately not wired into `Intake`'s roller.** It already has tested, tuned jam-recovery (`kJamStatorCurrentAmps`, cooldown timer, the sixth-session bug fix documented in `docs/claudex/history.md`). A second independent over-current detector on the same signal is the same "two writers, same tick" failure class already documented in `docs/claudex/architecture.md`. `roller`'s `RobotMotor` gets `safetyTrip = null`.

`intakePivot`, `shootMotor`, `indexMotor` get `safetyTrip` wired — genuinely new visibility, no existing detector on those signals today.

Trip state logs via direct `Logger.recordOutput`, the same way `Intake.java` already logs `TargetState`/`JamRecoveryActive` — not through the replayable `@AutoLog` inputs struct, since trip state isn't meaningful in sim and doesn't need replay verification.

## Migration: `IntakeIOReal`

```java
// before
private final TalonFX intakePivot = new TalonFX(Constants.intakePivot);
public IntakeIOReal() { configurePivot(); configureRoller(); }
private void configurePivot() { ...; intakePivot.getConfigurator().apply(cfg); }

// after
private final RobotMotor intakePivot = new RobotMotor(pivotConfig());
private final RobotMotor roller = new RobotMotor(rollerConfig());
public IntakeIOReal() {}   // config now happens via field-initializer order
private static RobotMotor.MotorConfig pivotConfig() {
  var cfg = new RobotMotor.MotorConfig();
  cfg.name = "Intake/Pivot";
  cfg.canId = Constants.intakePivot;
  cfg.talonConfig = new TalonFXConfiguration()....;   // byte-identical config body, returned instead of applied
  cfg.safetyTrip = RobotMotor.SafetyTripConfig.of(Amps.of(100), Seconds.of(0.25));
  return cfg;
}
private static RobotMotor.MotorConfig rollerConfig() {
  var cfg = new RobotMotor.MotorConfig();
  cfg.name = "Intake/Roller";
  cfg.canId = Constants.roller;
  cfg.talonConfig = new TalonFXConfiguration()....;   // byte-identical config body
  cfg.safetyTrip = null;   // deliberately disabled -- see "Deliberately not wired" above
  return cfg;
}
```

`updateInputs()`, `setPivotVoltage()`, `setPivotPosition()`, `setPivotEncoderPosition()`, `setRollerVoltage()`, `setRollerVelocity()` — **zero changes**. `RobotMotor` *is* a `TalonFX`, so every existing `.setControl()`/`.getPosition()`/etc. call keeps compiling unmodified. `IntakeIO` interface and `IntakeIOInputs` are untouched.

## Migration: `ShooterIOReal`

Identical treatment for `shootMotor` and `indexMotor` (both `TalonFX`, both currently built via `configureShoot()`/`configureIndex()` → converted to `shootConfig()`/`indexConfig()` static builders, `safetyTrip` wired on both). `agitator` (`TalonFXS`) is unchanged — see scope exclusion above. `ShooterIO` interface and `ShooterIOInputs` are untouched.

## Refactor sequence

Ordered so the build is never left broken between steps:

1. Add `utility/RobotMotor.java` alone — new file, nothing else touched. Gate: `./gradlew compileJava`.
2. Migrate `IntakeIOReal.java` only. Gates: `./gradlew compileJava`, `python SKILLS/run_headless_sim.py`, `./gradlew test` (see verification limitation below).
3. Migrate `ShooterIOReal.java`'s `shootMotor`/`indexMotor` only (`agitator` untouched). Same gates as step 2.
4. One-line Rule 1 wording amendment in `claude.md` blessing `RobotMotor`, plus a decision entry in `docs/claudex/architecture.md` recording the `RobotMotor` exception and the agitator/`TalonFXS` gap.
5. Commit.

## Verification limitation (stated plainly, not glossed over)

`SKILLS/run_headless_sim.py` and both JUnit tests (`RobotLifecycleTest`, `SuperstructureEjectingTest`) run in `Constants.Mode.SIM`, which constructs `IntakeIOSim`/`ShooterIOSim` — **never** `IntakeIOReal`/`ShooterIOReal`. So for steps 2–3:

- `./gradlew compileJava` proves the migration builds.
- `run_headless_sim.py`/`./gradlew test` passing proves nothing regressed in the sim-mode paths (which are untouched by this refactor) — it does **not** exercise `RobotMotor` against real CAN hardware.
- `RobotMotor`'s actual behavior on real hardware (config application, `pollSafetyTrip()` against real stator current) is **UNVERIFIED** until next robot bring-up.

This is consistent with `docs/claudex/verification-loop.md`'s rule that a gate which can't exercise a code path is reported as such, never implied as passing more than it does.
