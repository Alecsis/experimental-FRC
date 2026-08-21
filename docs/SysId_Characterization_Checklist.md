# SysId Drivetrain Characterization — Infrastructure Audit & Mentor Checklist

**Audit date:** 2026-07-19. **Scope:** documentation only — no production code was modified to produce
this file. Written for a mentor/operator who has not seen the investigation that produced it.

**Why this exists:** `CLAUDE.md`'s "Corrected fix order" (sixteenth/seventeenth session) says the
bounded-controller clamp is done (commit `56ebba2`) and the next step is a real SysId
characterization of the drive motors, before retuning `Slot0` or the PathPlanner chassis PID gains.
This document establishes whether the code side of that step is actually ready to run on the robot.

**Bottom line up front:** the SysId infrastructure is code-complete and already wired to a
dedicated controller. Nothing needs to be written to run a translation characterization today.
What's missing is entirely on the physical-robot side (see §6).

---

## 1. Existing SysId implementation

All of it lives in **`src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`**. It is
the **unmodified CTRE Tuner X swerve generator output** (see the class javadoc at the top of the
file, and `docs/claudex/architecture.md`'s sanctioned exception for this file) — not custom code
written for this project, and not touched by any of the drivetrain work in the last several
sessions (the `sanitizeAutoSpeeds()` addition is entirely separate, later in the file).

- **Routine type:** WPILib's `edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine` (import at line 40),
  three separate instances — **not** a custom command, and not Phoenix's own routine class.
- **Data capture:** each routine's state-logging callback writes to `com.ctre.phoenix6.SignalLogger`
  (`SignalLogger.writeString(...)`), not to WPILib's own `SysIdRoutineLog`. This is the CTRE-recommended
  hybrid pattern — WPILib's `SysIdRoutine` for test sequencing/voltage ramping, Phoenix's `SignalLogger`
  for the actual high-rate CAN-synchronous data capture (see §3). Confirmed against CTRE's docs
  ([SysId Integration](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/wpilib-integration/sysid-integration/index.html)).

### Translation (drive motors) — `CommandSwerveDrivetrain.java:109-119`
```java
private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
    new SysIdRoutine.Config(
        null,          // default ramp rate, 1 V/s
        Volts.of(4),   // dynamic step voltage reduced from the 7V default to prevent brownout
        null,          // default timeout, 10s
        state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
    new SysIdRoutine.Mechanism(
        output -> setControl(m_translationCharacterization.withVolts(output)),
        null, this));
```
Applies `SwerveRequest.SysIdSwerveTranslation` — drives all four modules straight, wheels forward,
open-loop voltage. **This is the routine the "Next up" SysId action in `CLAUDE.md` is asking for.**

### Steer (azimuth motors) — `CommandSwerveDrivetrain.java:125-135`
Same shape, 7V dynamic step, applies `SwerveRequest.SysIdSwerveSteerGains`. Exists and is wired,
but is not the currently-blocking item per `CLAUDE.md`.

### Rotation — `CommandSwerveDrivetrain.java:144-161`
Applies `SwerveRequest.SysIdSwerveRotation` (chassis spin-in-place, for tuning
`FieldCentricFacingAngle`'s heading controller, not the PathPlanner chassis PID). **Units warning
already called out in the in-repo comment:** the config's ramp/step values are in rad/s²
and rad/s respectively, disguised as "volts" because `SysIdRoutine.Config` only understands
volts — `Volts.of(Math.PI)` here means "π rad/s", not 3.14 actual volts. Don't read these off a
multimeter.

### Routine selection — `CommandSwerveDrivetrain.java:163-180, 426-439`
A single mutable field, `m_sysIdRoutineToApply`, holds whichever routine is "active." Three
setters (`useTranslationSysId()`, `useSteerSysId()`, `useRotationSysId()`) swap it, and two runner
methods (`sysIdQuasistatic(direction)` / `sysIdDynamic(direction)`) always operate on whatever is
currently active — they don't take a routine argument.

### Wiring status
**Already connected.** `RobotContainer.java` constructs the drivetrain, binds all three
mode-select actions and all four run actions to a dedicated controller, and calls
`drivetrain.registerTelemetry(...)`. Nothing here needs new bindings.

**Not present anywhere in this repo:** a wheel-radius characterization routine (drive in a slow
circle, compare gyro delta-heading to summed wheel rotations). Not required for kS/kV/kA — see §5
— flagged only because it's the other common companion SysId-adjacent routine and a mentor
searching for it won't find it.

---

## 2. Operator controls

**Controller:** a fourth, dedicated `CommandXboxController` on **DriverStation port 2**
(`RobotContainer.java:57`, `sysid = new CommandXboxController(2)`). This is separate from the
driver controller (port 0), the operator control box (port 1, a `CommandGenericHID`), and the
sim-mirror controller (port 3) — see `OperatorControls.java:20-26`, which documents this
separation explicitly ("bench/tuning `sysid` controller stays in RobotContainer: it's
characterization tooling, not driver/operator gameplay controls"). **A physical Xbox-style
controller must be plugged into DS port 2**, or these bindings simply never fire — no error, no
warning, just silent no-op.

**No enable/disable requirement is enforced in code.** The routines run whenever their trigger
condition is true, in any robot mode the scheduler is running commands in (typically Test or
Teleop — see §4 for the recommended real-world mode).

### Bindings (`RobotContainer.java:152-161`)

| Action | Binding |
|---|---|
| Select **Translation** routine | `leftBumper` (fires on press) |
| Select **Steer** routine | `leftTrigger` (fires on press) |
| Select **Rotation** routine | `rightTrigger` (fires on press) |
| Quasistatic forward | `Y` + `leftBumper` held |
| Quasistatic reverse | `A` + `leftBumper` held |
| Dynamic forward | `B` + `leftBumper` held |
| Dynamic reverse | `X` + `leftBumper` held |

### Operational gotcha — read this before running Steer or Rotation tests

The mode-select for **Translation** is bound to `leftBumper.onTrue(...)`, and `leftBumper` is
*also* the required modifier held down for all four run actions (Y/A/B/X). `onTrue` fires once,
on the rising edge of the button press — so:

- If you press-and-hold `leftBumper` first, it fires `useTranslationSysId()` immediately, on that
  press. Any Y/A/B/X you tap while still holding it will run **Translation**, regardless of what
  you intended.
- To run a **Steer** or **Rotation** test: hold `leftBumper` down first (Translation gets
  selected, that's fine), *then*, while still holding `leftBumper`, tap `leftTrigger` or
  `rightTrigger` to re-select Steer/Rotation — their `onTrue` fires independently of
  `leftBumper`'s held state. *Then* tap the Y/A/B/X run button.
- Releasing `leftBumper` and re-pressing it resets the mode back to Translation.

Get this order wrong and you'll silently run the wrong routine — the drivetrain will move, data
will log under the right `SysIdXxx_State` key for whatever *is* selected, and nothing will error.
**Verify the intended mode by watching `SysIdTranslation_State` / `SysIdSteer_State` /
`SysIdRotation_State` on NetworkTables/SmartDashboard before trusting a run's data.**

---

## 3. Logging requirements

### What's captured, and how
Each routine's state callback writes `state.toString()` (`"none"`, `"quasistatic-forward"`,
`"quasistatic-reverse"`, `"dynamic-forward"`, `"dynamic-reverse"`) to a `SignalLogger` string
signal. The Rotation routine additionally logs `"Rotational_Rate"`. **Position, velocity, and
motor-output samples themselves are not manually logged anywhere in this code** — Phoenix's
`SignalLogger` automatically captures every enabled Phoenix device signal (drive motor position,
velocity, applied voltage, etc.) at high rate, synchronized to the CAN bus, independent of the
20ms main robot loop. This is the whole point of using it over AdvantageKit's log for this
purpose: no CAN-latency or Java-GC jitter in the data SysId's regression is built on.

### Where logs are saved
`SignalLogger.start()` is called once, unconditionally, in `Telemetry.java:37`, inside the
`Telemetry` constructor — and `RobotContainer` constructs one `Telemetry` unconditionally
(`RobotContainer.java:53`). **This means hoot logging is already running from robot boot, in every
mode (REAL/SIM/REPLAY) — no manual "start logging" step is needed before a SysId run.** There is
no matching `SignalLogger.stop()` anywhere in the codebase, so a single boot session produces one
continuous hoot log containing everything from boot through shutdown, not just the SysId window —
expect to find your test data mixed in with ordinary drive telemetry, and use the state-marker
signals to locate it.

Per CTRE's docs, the on-robot default write location is **a `logs` folder on the first USB flash
drive found, or `/home/lvuser/logs` on the roboRIO's internal storage if no USB drive is present.**
Nothing in this codebase calls `SignalLogger.setPath(...)` to override that default. One important
generation-specific behavior from the same docs: **on a roboRIO 1, automatic logging is *only*
active if a USB flash drive is present** — if this team's roboRIO is a Gen 1, a run without a USB
stick inserted will silently produce no hoot log at all. (`TunerConstants.java:75`'s
`new CANBus("Swerve", "./logs/example.hoot")` second argument is unrelated to this — it's the
Tuner X generator's default *simulation replay* source path, only consulted under
`Utils.isSimulation()`, not where SignalLogger writes on real hardware.)

### AdvantageKit (wpilog) vs. Phoenix (hoot) — these are two independent logs
`Robot.java:53-73` sets up AdvantageKit's own `WPILOGWriter` (to `media/sda1/logs` on REAL,
`logs` on SIM). **This wpilog does not substitute for the hoot log and is not what the SysId
analyzer reads.** It captures whatever this codebase explicitly calls `Logger.recordOutput(...)`
for (e.g. the `Drive/AppliedVoltsPerModule` / `Drive/StatorCurrentAmpsPerModule` telemetry added
in the sixteenth session) — useful for cross-checking current-limit behavior, but it does not
contain the raw per-loop position/velocity/output triples SysId's regression needs, and it isn't
synchronized the way Phoenix's own signal capture is. **Phoenix telemetry (the hoot log) is
required and is the only source that matters for this task.**

### Tool/process to compute kS/kV/kA
1. Run the quasistatic and dynamic tests (both directions, ≥3-5s of data minimum per CTRE's own
   guidance for quasistatic — the routine's default 10s timeout comfortably covers this).
2. Retrieve the hoot log via **Phoenix Tuner X**'s file explorer (left panel — lists and downloads
   logs off the connected device) or copy it directly off the USB drive/roboRIO.
3. Convert `.hoot` → `.wpilog` using Tuner X's **Log Extractor / Convert** tab (queue the file,
   pick an output directory and `.wpilog` as the output format, click Convert). Reference:
   [Extracting Signal Logs](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tools/log-extractor.html).
4. Compute gains. CTRE's docs describe two paths: Tuner X can print measured **kS and kV**
   directly to its console during/after the workflow; if **kA** is also wanted, feed the converted
   `.wpilog` into the standalone **WPILib SysId analyzer application** (`sysid`, shipped with the
   WPILib installation) for the full regression. Exact current-version UI steps should be
   confirmed against CTRE's live docs at test time — Tuner X's UI has changed release to release
   (the 2026 season notes mention new in-app hoot plotting/Log Analyzer features), and this audit
   should not be trusted over what's on screen.

---

## 4. Physical robot procedure (mentor-facing checklist)

### Before test
- [ ] Confirm a USB flash drive is inserted if the roboRIO is Gen 1 (see §3) — otherwise no hoot
      log will be produced. Confirm roboRIO generation if unknown (**requires physical robot**).
- [ ] Confirm a physical Xbox-style controller is plugged into **DriverStation port 2**. Verify in
      the DS controller tab that it's recognized as `sysid` (or whatever the DS labels it).
- [ ] Fully charged battery. SysId's dynamic tests apply a sudden voltage step (4V for translation,
      already reduced from the 7V default specifically to avoid brownout per the in-code comment)
      — a weak battery risks a brownout mid-test, corrupting that run's data.
- [ ] Clear, open floor space: quasistatic tests ramp voltage slowly and can carry the robot a
      meaningful distance before timeout/manual release; dynamic tests are a sudden step that
      will move the robot quickly. Budget several meters of clear runway in the direction of
      travel, both forward and reverse.
- [ ] Elevate the robot on blocks/a cart for an initial dry run if you want to sanity-check motor
      response before committing to a floor run (standard SysId caution, not specific to this
      codebase).
- [ ] Robot in a mode where the scheduler runs bound commands (Teleop or Test) and NOT e-stopped.
- [ ] Know the mode-selection gotcha in §2 before touching the controller.

### During test
Recommended order — translation is the priority per `CLAUDE.md`'s next-action item, but the
routine covers steer and rotation too if time allows:

1. **Translation quasistatic forward**, then **reverse**. Hold `leftBumper`, tap `Y`, let it run
   until it naturally times out or you release — CTRE recommends ~3-5s minimum of data. Repeat for
   `A` (reverse).
2. **Translation dynamic forward**, then **reverse** (`B`, then `X`, both with `leftBumper` held).
   These are short, sudden steps — a couple of seconds is normal.
3. Repeat for Steer (select via `leftTrigger` first) and Rotation (`rightTrigger` first) if
   characterizing those too.

**Expected behavior:** quasistatic runs look like a slow, smooth crawl that gradually speeds up;
dynamic runs look like a sudden lurch to a steady speed. **Signs of a bad run:** the robot stalls
against something before the test naturally ends (redo after clearing the runway); a brownout or
CAN dropout mid-test (check DS log for brownout/faults, discard that run); the wrong
`SysIdXxx_State` signal was active the whole time (the §2 gotcha) — discard and redo with the
correct mode confirmed on NetworkTables first.

### After test
- [ ] Pull the hoot log (Tuner X file explorer, or directly off the USB stick/roboRIO storage).
- [ ] Convert to `.wpilog` via Tuner X's Log Extractor.
- [ ] Compute kS/kV(/kA) — Tuner X console output for kS/kV, WPILib's standalone `sysid` analyzer
      app if kA is wanted too (see §3, step 4).
- [ ] Sanity-check the numbers before trusting them: kS should be a small positive voltage
      (roughly similar order of magnitude to the existing placeholder `kS=0.1` in
      `TunerConstants.java:33` — a wildly different value warrants a re-run, not immediate
      acceptance). kV should produce a sensible theoretical top speed when combined with 12V and
      the existing `kSpeedAt12Volts = 7.52 m/s` — if `12/kV` diverges a lot from 7.52, treat that
      as a flag to double check the run before applying it.
- [ ] Do **not** paste the new values into `TunerConstants.java` as part of this audit — that's
      the explicit next session's job per the user's "no tuning" instruction for this task. Record
      them somewhere durable (session notes / the Obsidian vault's PID Gains Registry, per
      `CLAUDE.md`'s Permanent Brain Sync section) so the next session doesn't have to redo the run.

---

## 5. Configuration targets — exactly where results go

| Value | Destination | Current value | Notes |
|---|---|---|---|
| Drive `kS`, `kV`, `kA` | `TunerConstants.java:32-34`, `driveGains` (`Slot0Configs`) | `kP=0.1, kI=0, kD=0, kS=0.1, kV=0.124, kA` unset | **This is the primary target.** These are the unedited CTRE Tuner X generator defaults, not a real characterization (per `CLAUDE.md`'s fourteenth-session finding) — this is what the whole exercise replaces. |
| Steer `kS`, `kV`, `kA` | `TunerConstants.java:26-29`, `steerGains` | `kP=100, kI=0, kD=0.5, kS=0.1, kV=2.5, kA=0` | Secondary; not the currently-blocking item, but the routine is ready if a future session wants it. |
| Drive `Slot0` `kP`/`kI`/`kD` | Same `driveGains` block | `kP=0.1` (generator default) | **Not** produced by SysId directly — SysId gives feedforward terms; `CLAUDE.md`'s fix order explicitly retunes `Slot0` *after* getting real kS/kV/kA, as a separate step. |

**PathPlanner's `RobotConfig` (`src/main/deploy/pathplanner/settings.json`) is a separate,
independent feedforward system and is *not* a destination for SysId's output.** It's loaded via
`RobotConfig.fromGUISettings()` (`CommandSwerveDrivetrain.java:353`) and computes its own physics
based feedforward internally from `robotMass` (43.545 kg), `driveMotorType` (`krakenX60`),
`driveCurrentLimit` (120A), and `wheelCOF` (0.8) — fed into
`.withWheelForceFeedforwardsX/Y(...)` at `CommandSwerveDrivetrain.java:362-363`. This is entirely
independent of `TunerConstants`' `Slot0` closed-loop gains, which drive each module's own onboard
velocity control loop every CAN cycle. **Don't conflate the two** — a mentor unfamiliar with this
split might expect SysId's output to also update `settings.json`; it doesn't, and shouldn't.

**Assumptions worth independently verifying while you're at it** (not required for the
characterization itself, but they directly scale its results):
- `kWheelRadius = 2 in` (`TunerConstants.java:87`) / `driveWheelRadius: 0.0508` (`settings.json:21`)
  — tire wear changes effective rolling radius and directly scales kV. A tape-measure or
  rollout-distance check is still on the `CLAUDE.md` Backlog as unmeasured.
- `wheelCOF = 0.8` (`settings.json:26`) — flagged repeatedly in `CLAUDE.md` as a placeholder
  every recent traction-related finding pivots on; SysId won't measure this (it's a PathPlanner
  input, not a `Slot0` gain), but it's adjacent enough that a bring-up session is a natural time
  to also get a real number for it.

---

## 6. Missing pieces — classification

| Item | Classification | Notes |
|---|---|---|
| WPILib `SysIdRoutine` objects (translation/steer/rotation) | **Ready now** | Fully constructed, correct config, in `CommandSwerveDrivetrain.java`. |
| Controller bindings to trigger them | **Ready now** | `RobotContainer.java`, port 2, all 7 actions bound. |
| Phoenix `SignalLogger` capture | **Ready now** | Starts automatically at boot (`Telemetry.java:37`), no manual step. |
| Actually running the 4-8 characterization passes | **Requires physical robot** | Sim characterization is circular here — maple-sim's own friction terms (`kDriveFrictionVoltage`/`kSteerFrictionVoltage`, `TunerConstants.java:98-99`) are themselves unmeasured placeholders, so a sim SysId run would just recover the numbers already fed into the sim, not real hardware behavior. |
| Confirming roboRIO generation + USB drive presence | **Requires physical robot** | Determines whether hoot logging even activates (Gen 1 requires a USB drive; see §3). |
| Confirming CTRE Pro license status | **Requires physical robot / account check** | `TunerConstants.java:49`'s existing comment already flags Fused/Sync CANcoder falling back without Pro licensing as an open unknown — the same licensing status also affects how much signal data exports for free from the Log Extractor per CTRE's docs. Worth resolving once, since it affects more than just this task. |
| Converting hoot → wpilog, computing kS/kV(/kA) | **Requires physical robot** (needs real data first) | Tooling itself (Tuner X, WPILib `sysid`) is external and already installed as part of the standard WPILib/Phoenix toolchain — no repo changes needed. |
| Pasting new kS/kV/kA into `TunerConstants.java` | **Requires tuning judgment — explicitly out of scope for this session** | One-line edit once real numbers exist; sanity-check against the existing placeholder before accepting (see §4). |
| Retuning drive `Slot0` kP/kI/kD off the new feedforward | **Requires tuning judgment** | `CLAUDE.md`'s fix order step 3, strictly after this step. |
| Wheel-radius characterization routine | **Not present, not required** | No such routine exists in this repo. Not needed for kS/kV/kA; only relevant if the team separately wants to cross-check the 2in wheel-radius assumption. Would be a **code change** if ever wanted — no action needed for the current task. |
| Any code/binding changes to the SysId infrastructure itself | **None identified** | This is the headline finding: the code side of "run a real SysId characterization" is already done. Everything remaining is physical-robot execution and external-tool usage. |
