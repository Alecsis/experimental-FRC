# Drive Slot0 Retuning Audit — Evidence, Impact, and Proposed Procedure

**Audit date:** 2026-07-28. **Scope:** investigation only — no `TunerConstants.java`, `CommandSwerveDrivetrain.java`,
or any other production file was modified to produce this document (`git status --short` clean before and after).
Builds directly on three prior investigation-only audits from 2026-07-19 (`docs/SysId_Characterization_Checklist.md`,
`docs/Drivetrain_Physical_Constants_Audit.md`, `docs/Path_Following_Tuning_Readiness_Audit.md`) — their findings are
cited, not re-derived, except where this audit found something those three missed. Confirmed via
`git log --since=2026-07-19 -- TunerConstants.java CommandSwerveDrivetrain.java settings.json Constants.java` that
**none of the audited files have changed since those three audits were written** — their findings are still current,
not stale.

**New evidence method used here that the prior three audits didn't:** this audit decompiled CTRE's actual shipped
`wpiapi-java-26.1.3-sources.jar` (found in the local Gradle cache, matching this repo's `vendordeps/Phoenix6-26.1.3.json`
exactly) to read `SwerveModuleConstants.java`, `SwerveModule.java`, and `SwerveRequest.java` directly, rather than
relying on public docs pages alone. Public docs (`v6.docs.ctr-electronics.com`, queried via context7 and WebSearch)
were also checked and confirm no season-2026 changelog entry touches Slot0/SysId/`ApplyRobotSpeeds`/gear-ratio handling
— the decompiled source is authoritative here, not a fallback.

---

## Bottom line up front

**The single highest-value finding of this audit, not present in any prior one:** autonomous path-following in this
codebase runs the drive motors in **`DriveRequestType.OpenLoopVoltage`** — confirmed from CTRE's own source, this mode
**does not consult the drive `Slot0` gains at all** (not `kP/kI/kD`, not `kS/kV/kA`). It uses only `kSpeedAt12Volts`
as a linear scalar. This means:

- **SysId-characterizing drive `kS`/`kV`/`kA` and retuning drive `Slot0` `kP`/`kI`/`kD` — the two steps `CLAUDE.md`'s
  existing fix order puts right after the already-shipped bounded-output clamp — currently have zero effect on
  autonomous path-tracking accuracy**, the exact problem (~6.9m peak lateral error on `LT Neutral`) that motivated the
  whole tuning investigation.
- Those two steps **do** matter for **teleop driving** and the **vision `trackTarget` command**, both of which run in
  `DriveRequestType.Velocity` (closed-loop, confirmed — see §2) and therefore do consult `Slot0` fully.
- This doesn't mean skip SysId — it means the existing playbook's step order needs one explicit decision inserted
  before it, and the *reason* to do SysId now shifts from "fix autonomous tracking" to "fix teleop/vision-tracking feel,
  and lay groundwork for a future closed-loop-auto decision." See §5 for the corrected priority order.

Everything below is either **confirmed** (read directly from this repo's code or CTRE's decompiled/documented source)
or explicitly labeled **hypothesis**/**unverified**, matching this project's own established audit conventions.

---

## 1. Current gains — placeholder/default inventory

| Location | Value | Status |
|---|---|---|
| `TunerConstants.java:32-34` `driveGains` (Slot0) | `kP=0.1, kI=0, kD=0, kS=0.1, kV=0.124, kA` unset (0) | **Confirmed placeholder.** Matches the CTRE Tuner X generator's own template almost exactly — context7's live-pulled current generator example shows the identical `kP=0.1, kI=0, kD=0, kV=0.124` (one difference: the current generator template ships `kS=0`, this repo has `kS=0.1` — a minor drift from the stock template, not obviously wrong, but worth knowing it's not the current generator default verbatim). |
| `TunerConstants.java:26-29` `steerGains` (Slot0) | `kP=100, kI=0, kD=0.5, kS=0.1, kV=2.5, kA=0` | **Confirmed placeholder**, same generator-template origin. Not the current session's priority (translation is), carried for completeness. |
| `TunerConstants.java:79` `kSpeedAt12Volts` | `7.52 m/s` | **Not a placeholder** — already independently confirmed against WCP's published SwerveX2S X3-tier free-speed range in the Physical Constants Audit (§1 of that doc). Real, measured-adjacent value. |
| `configureAutoBuilder()` chassis PID (`CommandSwerveDrivetrain.java:364-368`) | Translation `PIDConstants(5,0,0)`, Rotation `PIDConstants(3,0,0)` | **Confirmed placeholder**, never empirically tuned (Path-Following Audit §1). Unclamped — this is exactly why `sanitizeAutoSpeeds()` exists. |
| `kWheelRadius` / `driveWheelRadius` | 2 in (0.0508 m), duplicated in `settings.json` | **Confirmed placeholder-adjacent problem**: disagrees with WCP's stock SwerveX2S wheel (3.5in OD / 1.75in radius) by ~14% (Physical Constants Audit §4a). Directly scales every drive-`kV` computation below — **must be caliper-verified before trusting any new SysId `kV` for cross-checks against `kSpeedAt12Volts`.** |
| `wheelCOF = 0.8` | `settings.json`, `Constants.java` | **Confirmed unmeasured placeholder** (Physical Constants Audit §3). Not a `Slot0` value, but bounds how much of any newly-characterized `kA` can actually be used before wheel slip. |

---

## 2. Which control path uses which gains — the gap the prior audits flagged but couldn't close

The Path-Following Tuning Readiness Audit (§4) flagged this as **unverified**: *"`m_pathApplyRobotSpeeds` never has
`.withDriveRequestType()` called on it... This session could not confirm that default from CTRE's public API
documentation."* This audit closes that gap by reading CTRE's actual source.

**Confirmed from `SwerveRequest.java:885`** (decompiled `wpiapi-java-26.1.3-sources.jar`):
```java
public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
```
`ApplyRobotSpeeds`'s default `DriveRequestType` is `OpenLoopVoltage`. `configureAutoBuilder()`
(`CommandSwerveDrivetrain.java:359-363`) never overrides it. **Autonomous path-following is confirmed running in
open-loop voltage mode today.**

**Confirmed from `SwerveModuleConstants.java`'s own field documentation** (the `SpeedAt12Volts` field, lines 358-365):
> *"When using open-loop drive control, this specifies the measured speed at which the robot travels when driven with
> 12 volts. This is used to approximate the output for a desired velocity... **If using closed loop control, this
> value is ignored.**"*

Read together with the `DriveMotorGains` field doc (line 339-342): *"When using **closed-loop** control, the drive
motor uses the control output type specified by `DriveMotorClosedLoopOutput` and **any closed-loop
`DriveRequestType`**"* — i.e. `Slot0` (`kP/kI/kD/kS/kV/kA`) is documented as a closed-loop-only mechanism.
`OpenLoopVoltage` is, by CTRE's own naming and field docs, not one of the closed-loop request types that consult it.
**Net: under `OpenLoopVoltage`, only `kSpeedAt12Volts` matters; `Slot0` (all six drive gains) is inert.**

**Grepped every `DriveRequestType` usage in this repo** (`CommandSwerveDrivetrain.java`, `RobotContainer.java`) to
build the full picture:

| Control path | File:line | `DriveRequestType` | Consults `Slot0`? |
|---|---|---|---|
| Autonomous path-following (PathPlanner) | `CommandSwerveDrivetrain.java:98` (`m_pathApplyRobotSpeeds`, never overridden) | **OpenLoopVoltage** (default, unset) | **No** |
| `driveToPOI()` (auto-align-to-POI) | `CommandSwerveDrivetrain.java:449` | **OpenLoopVoltage** (explicit) | **No** |
| Teleop default drive command | `RobotContainer.java:52` | **Velocity** (explicit) | **Yes** |
| `trackTarget()` (vision hub/pass tracking, drive-while-aim) | `CommandSwerveDrivetrain.java:516,540` | **Velocity** (explicit) | **Yes** |

**Bonus finding — a stale, actively misleading in-code comment.** `RobotContainer.java:52`:
```java
.withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
```
The comment says "open-loop"; the code sets `Velocity`, which is CTRE's **closed-loop** request type (confirmed by
the enum's own javadoc at `SwerveModule.java:115-119`: *"Control the drive motor using a velocity **closed-loop**
request."*). Whoever reads only the comment would wrongly conclude teleop driving doesn't use `Slot0` at all — the
opposite of the truth. Cheap to fix, flagged here for completeness; not fixed by this audit per the "no constants
changes yet" instruction (this is a comment, not a gain, but leaving it alone matches the spirit of "evidence only").

---

## 3. Unit consistency — the rotor-vs-mechanism trap, confirmed from source

**Confirmed from `SwerveModuleConstants.java`'s own field docs**, quoted exactly:

- `DriveMotorGains` (line 341-342): *"These gains operate on **motor rotor rotations (before the gear ratio)**."*
- `SteerMotorGains` (line 333-334): *"These gains operate on **azimuth rotations (after the gear ratio)**."*

This asymmetry is structural, not incidental — confirmed by the `withDriveMotorInitialConfigs`/
`withSteerMotorInitialConfigs` javadocs listing exactly what gets overwritten on each motor:

- Drive (lines 413-414): `FeedbackConfigs.RotorToSensorRatio` / `SensorToMechanismRatio` are both overwritten to
  **`1.0`** — the drive TalonFX's onboard closed loop sees raw rotor rotations, full stop. `DriveMotorGearRatio` is
  used only for chassis-kinematics math (module → chassis speed conversion) elsewhere, never fed into the drive
  motor's own `FeedbackConfigs`.
- Steer (lines 950-951): `FeedbackConfigs.RotorToSensorRatio` is overwritten to **`SteerMotorGearRatio` (25.9)** — the
  steer TalonFX's onboard loop already sees post-gear-ratio azimuth rotations.

**Concrete consequence for this repo's `driveGains.kV = 0.124`:** that number is **volts per rotor-rotation-per-second**,
not per wheel-rotation-per-second and not linear m/s. Any SysId run's raw output should already be in this convention
(SysId characterizes whatever the mechanism reports, and `SysIdSwerveTranslation` drives the module through the same
rotor-referenced Slot0 path) — but it means **the existing sanity-check formula written into
`docs/SysId_Characterization_Checklist.md` §4 is unit-inconsistent as stated**, and this audit corrects it:

> *Checklist's stated check (imprecise):* "kV should produce a sensible theoretical top speed when combined with 12V
> and the existing `kSpeedAt12Volts = 7.52 m/s` — if `12/kV` diverges a lot from 7.52, treat that as a flag."

`12/kV` yields a value in **rotor-rotations-per-second**, not m/s — comparing it directly to `7.52 m/s` conflates two
different units by the drive gear ratio × wheel circumference conversion factor (≈0.0859 here), not 1. The **correct**
check, computed from this repo's own already-confirmed constants (`kDriveGearRatio = 3.7142857`,
`kWheelRadius = 0.0508 m` → circumference `C = 0.31920 m`):

```
theoretical_linear_speed_at_12V = (12 / kV_rotor) / kDriveGearRatio * C
```

Applying this to the **current placeholder** `kV = 0.124`:
```
(12 / 0.124) / 3.7142857 * 0.31920 = 96.77 rotor-rps / 3.7142857 = 26.06 wheel-rps * 0.31920 m = 8.32 m/s
```
**8.32 m/s vs. the real, confirmed `kSpeedAt12Volts = 7.52 m/s` — about 11% high.** That's actually a *sane* result:
the placeholder `kV` is a generic motor-constant-derived default with no friction/loading correction, so it should
predict a **higher** free speed than the real, loaded, measured top speed — and 11% is a plausible loading margin, not
a wild divergence. This is a genuine (if secondhand) sanity check that the placeholder's *units* are self-consistent
with a rotor-rps convention, even though its *value* is not yet this-robot-characterized. **Use the corrected formula
above, not the checklist's literal `12/kV` comparison, when validating a future real SysId `kV` result** — a real kV
plugged into the wrong formula could look wildly wrong (or wrongly plausible) purely from the missing gear-ratio term.

**Steer gains have no equivalent trap** — `kV=2.5` (`steerGains`) is already in post-gear-ratio azimuth-rps units,
directly comparable to azimuth angular rates without a conversion step.

**Both drive and steer are internally SI-consistent everywhere else already checked** — `sanitizeAutoSpeeds()` and
`LoggingHolonomicDriveController` both operate in m/s and rad/s throughout with no hidden conversions (re-confirmed
this session by re-reading `CommandSwerveDrivetrain.java:330-349`; matches the Path-Following Audit's own §4 finding).

---

## 4. Feedforward usage across the autonomous pipeline

Traced end-to-end again this session (`AutoBuilder.configure()` → `LoggingHolonomicDriveController` →
`configureAutoBuilder()`'s consumer lambda → `sanitizeAutoSpeeds()` → `ApplyRobotSpeeds` → CTRE module control),
cross-checked against the Path-Following Audit's §1/§4 findings, with §2's `OpenLoopVoltage` finding folded in:

**Two independent feedforward paths exist, and neither one currently touches `TunerConstants`' drive `Slot0`:**

1. **PathPlanner's own field-relative velocity feedforward**, summed with chassis PID feedback inside
   `PPHolonomicDriveController.calculateRobotRelativeSpeeds()` with no clamp between the two terms (confirmed against
   PathplannerLib 2026.1.2 source, Path-Following Audit §1). This produces the `ChassisSpeeds` that
   `sanitizeAutoSpeeds()` bounds. **This feedforward is chassis-kinematic (trajectory-nominal velocity), not a motor
   voltage/current feedforward — it never touches `Slot0` at all**, by design, regardless of `DriveRequestType`.
2. **PathPlanner's per-module wheel-force feedforward** (`feedforwards.robotRelativeForcesXNewtons()/Y()`), passed
   through to `ApplyRobotSpeeds.WithWheelForceFeedforwardsX/Y` (`CommandSwerveDrivetrain.java:362-363`). This is
   computed by PathPlanner from **`settings.json`'s own physics model** (`robotMass`, `driveMotorType`,
   `driveCurrentLimit`, `wheelCOF`) — confirmed independent of `TunerConstants`' `Slot0` (Physical Constants Audit §5,
   Path-Following Audit §1). **This is the only per-module feedforward that reaches the drive motors during
   autonomous today**, since path (1)'s output goes through `OpenLoopVoltage` (§2 above), which ignores `Slot0`
   entirely and only scales by `kSpeedAt12Volts`.

**One new question this audit could not close from Java source alone (flagged as unverified, not asserted):** whether
`WheelForceFeedforwardsX/Y` has any effect when `DriveRequestType = OpenLoopVoltage`. CTRE's actual force→voltage/
current conversion happens inside the native JNI layer (`SwerveJNI`), not visible in the decompiled Java source. The
field's own doc doesn't restrict it to closed-loop use, and `CommandSwerveDrivetrain.java`'s own comment (line 324)
already describes it as "applied as an added torque/current feedforward term on top of whichever closed-loop drive
control mode is active" — written when the codebase's prior authors also assumed a closed-loop context. **If
autonomous is confirmed running open-loop (§2, now settled), whether this wheel-force term does anything useful today
is worth a direct real- or sim-hardware check** (e.g. compare `Drive/AppliedVoltsPerModule` with the term zeroed vs.
not, on the same trajectory) before assuming it's already contributing — it may currently be a no-op in this specific
`OpenLoopVoltage` configuration.

**Net for "feedforward usage throughout the autonomous pipeline":** the pipeline has real, working feedforward
(items 1 and 2 above), but **none of it currently originates from `TunerConstants`' `Slot0` `kS/kV/kA`** — those sit
entirely unused for autonomous under the current `OpenLoopVoltage` configuration. A SysId-derived `kS/kV/kA` would
only enter the autonomous pipeline if a future change explicitly switches `m_pathApplyRobotSpeeds` to
`DriveRequestType.Velocity` — a real behavior change, not a constants-only edit, and squarely a decision for the
mentor, not something to default into.

---

## 5. Expected impact by action — what actually changes, and what doesn't

| Candidate action | Confirmed effect | Confirmed no-effect |
|---|---|---|
| SysId-characterize drive `kS/kV/kA`, apply to `TunerConstants.driveGains` | Improves **teleop** feel (smoother velocity tracking, more accurate stick-to-speed mapping) and **`trackTarget`** vision-tracking precision — both run `DriveRequestType.Velocity` | **Zero effect on autonomous path-tracking** or `driveToPOI` — both run `OpenLoopVoltage`, which ignores `Slot0` entirely (§2) |
| Retune drive `Slot0` `kP/kI/kD` off the new feedforward | Same as above — teleop + `trackTarget` only | Same — zero effect on autonomous or `driveToPOI` |
| Caliper-verify wheel radius, correct `kWheelRadius`/`driveWheelRadius` if the 14% WCP-stock discrepancy is real (Physical Constants Audit §4a) | **Affects everything** — odometry distance-per-rotation, `sanitizeAutoSpeeds()`'s desaturation ceiling, any future SysId `kV` cross-check math (§3), PathPlanner's wheel-force feedforward | — |
| Empirically re-verify `kSpeedAt12Volts` (currently plausible but not re-measured post-characterization) | **Directly changes autonomous and `driveToPOI` behavior today** — both scale purely off this constant under `OpenLoopVoltage` | — |
| Retune `PPHolonomicDriveController` chassis-level gains (translation `kP=5`, rotation `kP=3`) | **Affects autonomous tracking regardless of `DriveRequestType`** — this is upstream of the module-level control split, the dominant term in the already-documented ~6.9m lateral-error investigation (CLAUDE.md, sixteenth session: feedback was ~99% of commanded magnitude) | — |
| Switch `m_pathApplyRobotSpeeds` to `DriveRequestType.Velocity` (code change, not a constants edit — flagged, not recommended by default) | Would make drive `Slot0` (all six gains) newly relevant to autonomous for the first time; CTRE's own docs (§4, "Choosing Output Type") note voltage/duty-cycle closed-loop control **indirectly** controls acceleration and needs `kV` to hold speed, vs. torque-current modes controlling it directly — a real design choice, not a default | — |
| Fix the `RobotContainer.java:52` stale comment (§2) | Prevents a future reader from misjudging which control path uses `Slot0` | Zero functional effect — comment-only |

**This inverts part of `CLAUDE.md`'s existing "corrected fix order"** (SysId → drive `Slot0` → chassis PID), which was
written before this control-path split was confirmed. It doesn't invalidate the order for teleop/vision-tracking
quality, but it means **the order was never going to fix the autonomous-tracking problem it was originally written to
address** — that problem lives entirely in the chassis-level `PPHolonomicDriveController` gains and the physical
constants (wheel radius, `kSpeedAt12Volts`, `wheelCOF`), none of which are `Slot0`.

---

## 6. Proposed tuning procedure — reordered by confirmed impact, no gain values recommended

This supersedes the ordering (not the content) of the Path-Following Tuning Readiness Audit's §6 "September Tuning
Playbook" for the specific question of *when* SysId/`Slot0` work should happen relative to autonomous-tracking work.
That playbook's Steps 0, 4, 5, 6, 7 are unaffected and still apply as written; Steps 1-3 are reordered/re-scoped here.

**Step 0 (unchanged from the existing playbook):** Pre-flight physical verification — CANivore name, wheel-radius
caliper check, `Drive/CancoderBootReadRotations` vs. configured offsets. Do this regardless of which path below is
taken next; several other steps depend on wheel radius being right.

**Step A — Decision point (mentor sign-off required, not a default): does autonomous stay `OpenLoopVoltage` or move to
`Velocity`?** This determines whether the rest of this document's "teleop-only" impact column becomes "teleop +
autonomous." Two honest options, no recommendation forced here since it's a real trade-off:
- **Stay `OpenLoopVoltage` for auto** (no code change): autonomous tracking work should focus on chassis-level PID
  (Step B) and physical constants (wheel radius, `kSpeedAt12Volts`), not `Slot0`. SysId/`Slot0` retuning becomes a
  **teleop/vision-tracking-quality project**, decoupled from the autonomous-tracking investigation, and can proceed
  independently on its own priority.
- **Switch auto to `Velocity`** (a real code change, one line — `m_pathApplyRobotSpeeds.withDriveRequestType(...)` in
  `configureAutoBuilder()`): makes `Slot0` newly relevant to autonomous, at the cost of now needing the drive
  characterization to be *right* before autonomous behavior can be trusted, and reintroducing acceleration control as
  *indirect* (needs `kV` to hold speed) rather than direct — see the CTRE "Choosing Output Type" guidance quoted in
  §5. This is the path that would make `CLAUDE.md`'s original fix order (SysId → `Slot0` → chassis PID) actually
  apply to the autonomous-tracking problem as originally intended.

**Step B — Chassis-level PID retune (`PPHolonomicDriveController`, translation `kP=5`/rotation `kP=3`)** can proceed
**independently of Step A's outcome**, and is the most directly evidenced lever on the existing ~6.9m tracking error
regardless of which drive control mode is chosen. Matches the existing playbook's Step 5, just promoted earlier in
priority for autonomous work specifically, since it's the one lever confirmed to matter under *either* choice in
Step A.

**Step C — SysId characterization (translation)**, per `docs/SysId_Characterization_Checklist.md`'s existing,
code-complete, ready-to-run procedure — **unchanged mechanically**, but now scoped explicitly: do this because it
improves teleop and `trackTarget`, and/or because Step A chose `Velocity` for auto. If Step A stayed
`OpenLoopVoltage`, this step's acceptance criteria should be validated against **teleop driving feel and
`SwerveStates/Measured` during manual velocity commands**, not against autonomous path-tracking error — the latter
won't move.

**Step D — Retune drive `Slot0` `kP/kI/kD`** off the Step C feedforward, using the existing playbook's step-response
methodology (manual `SwerveRequest.Velocity` commands, watch `SwerveStates/Measured` vs `Setpoints` and
`Drive/StatorCurrentAmpsPerModule`). Same scoping caveat as Step C — validates teleop/`trackTarget` behavior primarily
unless Step A chose `Velocity` for auto.

**Step E onward:** rejoin the existing playbook's Steps 4 (steer, real-hardware re-verification), 6 (discretization
A/B), 7 (auto regression suite expansion) unchanged — those don't depend on this audit's findings.

**Also cheap, do whenever convenient (no dependency on the above):**
- Fix the `RobotContainer.java:52` stale "open-loop" comment (§2) — purely descriptive, zero functional risk.
- Directly test whether `WheelForceFeedforwardsX/Y` has any measurable effect under the current `OpenLoopVoltage`
  configuration (§4's unverified item) — cheap A/B, resolves a real unknown before it's assumed either way.
- The `RB Neutral` `goalEndState`/`idealStartingState` velocity mismatch (Path-Following Audit §1) — unrelated to
  gains, a `.path`-file authoring fix, independent of everything in this document.

---

## Summary of confirmed vs. hypothesis vs. unverified findings (this audit only)

**Confirmed (read directly from decompiled CTRE source or this repo's code):**
- `ApplyRobotSpeeds.DriveRequestType` defaults to `OpenLoopVoltage`; `configureAutoBuilder()` never overrides it —
  autonomous and `driveToPOI` run open-loop.
- Teleop and `trackTarget` explicitly request `DriveRequestType.Velocity` — closed-loop, `Slot0`-consulting.
- Drive `Slot0` gains operate on raw rotor rotations (`RotorToSensorRatio`/`SensorToMechanismRatio` hardcoded to 1.0
  for the drive motor); steer `Slot0` gains operate on post-gear-ratio azimuth rotations.
- `OpenLoopVoltage` is documented as ignoring `Slot0` and using `SpeedAt12Volts` instead; `Slot0` is documented as a
  closed-loop-only mechanism.
- The existing SysId checklist's `12/kV` vs. `kSpeedAt12Volts` sanity check is unit-inconsistent; the corrected
  formula requires dividing by `kDriveGearRatio` and multiplying by wheel circumference.
- `RobotContainer.java:52`'s comment ("open-loop") contradicts its own code (`DriveRequestType.Velocity`).
- No vendordep version has changed, and no 2026-season CTRE changelog entry touches Slot0/SysId/`ApplyRobotSpeeds`/
  gear-ratio handling, since the three prior 2026-07-19 audits — their findings are current, not stale.

**Hypothesis (plausible, not proven this session):**
- None of this audit's own findings are hypotheses — the control-path/unit findings above are all sourced directly.
  (Prior audits' own hypothesis-labeled items — missing discretization's real-world impact, `RB Neutral`'s tracking
  effect, maple-sim's battery-model fidelity — are unchanged and not re-litigated here.)

**Unverified (flagged, not asserted either way):**
- Whether `WheelForceFeedforwardsX/Y` has any effect under `OpenLoopVoltage` — CTRE's force→voltage/current
  conversion happens in native JNI code, not visible from the decompiled Java source.
