# Autonomous Drive Architecture Recommendation — OpenLoopVoltage vs. Velocity

**Audit date:** 2026-07-28. **Scope:** investigation and synthesis only — no production file was modified
(`git status --short` clean before and after, confirmed at the end of this document). Builds directly on
`docs/Autonomous_Control_Path_Audit.md` (complete, line-cited call graph proving the current control path) and
`docs/Drive_Slot0_Retuning_Audit.md` (gain inventory and unit-consistency findings) — those facts are cited, not
re-derived. New work in this document: a fair, evidence-based comparison of the two architectures, and research into
what CTRE and other teams' shipped code actually do.

---

## 1. OpenLoopVoltage vs. Velocity — compared on the six requested dimensions

Each row is graded on **mechanism**, not preference — what actually differs, confirmed either from CTRE's own
decompiled source/docs (per the prior two audits) or from direct control-theory reasoning that follows from those
confirmed mechanics. Where a claim can't be fully confirmed from source, it's marked accordingly.

| Dimension | `OpenLoopVoltage` | `Velocity` (closed-loop) | Basis |
|---|---|---|---|
| **Trajectory accuracy** | Commands voltage as a fixed linear function of desired speed via `kSpeedAt12Volts` only (confirmed, `docs/Drive_Slot0_Retuning_Audit.md` §2, fact-cited in the Control Path Audit #16). No feedback if the resulting shaft speed differs from what was commanded. | The onboard `Slot0` loop measures actual rotor velocity every control cycle and corrects toward the commanded value (`kP/kI/kD` on velocity error, `kV/kS/kA` as feedforward) — confirmed mechanism from `SwerveModuleConstants.java`'s own `DriveMotorGains` doc (Control Path Audit fact #15). | Direct from CTRE source. Neither mode changes the **chassis-level** command (`PPHolonomicDriveController`'s unclamped `kP=5`/`kP=3` sum, Control Path Audit fact #6-7) — this row is about how faithfully that chassis-level command gets executed at the module, not about the chassis-level command's own correctness. |
| **Disturbance rejection** | None, structurally — no sensor feedback loop exists in this control path (confirmed absence, not inferred: `SpeedAt12Volts`'s field doc says open-loop "approximates" output, no correction mechanism is documented or present). | Corrects **shaft-level torque disturbances** — carpet friction changes, minor defense-contact loading, incline — by increasing/decreasing commanded voltage to hold measured rotor velocity at target. **Does not correct ground-relative wheel slip** — see next row, this is a real and important distinction this audit did not see stated elsewhere. | Confirmed mechanism (Slot0 closed loop) + reasoned consequence (a motor encoder cannot distinguish "wheel gripping and rotating at commanded speed" from "wheel slipping and rotating at commanded speed" — both look identical from the rotor's own sensor). |
| **Wheel slip** | No feedback either way — commands the same voltage regardless of whether the wheel is gripping or spinning freely. | **Also does not detect or correct ground-relative slip** — a slipping wheel's rotor still reports whatever speed the motor is actually spinning at, which can match the commanded target even while the wheel isn't translating the chassis. Neither mode is a traction-control system. | The actual anti-slip mechanism in this codebase is `kSlipCurrent = 120A` (`TunerConstants.java:54`), a stator current limit applied to the drive motor unconditionally, independent of `DriveRequestType` (confirmed: `SwerveModuleConstants.java`'s `SlipCurrent` field is a separate config path from `DriveMotorGains`, applied via `CurrentLimitsConfigs` per the same class's `withDriveMotorInitialConfigs` javadoc, already read in the prior audit). **This dimension does not differentiate the two modes** — a finding worth stating plainly since it's easy to assume closed-loop "fixes slip." |
| **Battery voltage variation** | The `OpenLoopVoltage` name and CTRE's general "Voltage" output-type behavior (confirmed elsewhere in Phoenix 6's docs: *"Voltage control output takes into account the supply voltage to ensure its voltage output remains consistent,"* from the "Choosing Output Type" doc already cited in the prior audit) suggest the **requested voltage** is likely bus-voltage-compensated even in open-loop mode — **this specific claim is not independently confirmed for the open-loop path from source** (the actual computation is native/JNI, per Control Path Audit §5b) and is stated here as a reasoned inference, not a fact. Regardless of that inference, there is no correction for the **resulting speed** falling short under sag — only for the voltage delivery being consistent. | Same physical ceiling applies (can't exceed available bus voltage), but the closed loop will use more of whatever headroom remains to hold the commanded speed, and — critically — the shortfall becomes **observable** in `SwerveStates/Measured` vs `Setpoints` divergence and closed-loop error signals in a way that's structurally the same as before under open-loop (the encoder reports real speed regardless of control mode) but is now something the *controller itself* reacts to. | Partially confirmed, partially reasoned — flagged honestly rather than overstated. |
| **Tuning complexity** | Requires only `kSpeedAt12Volts` — already a plausible, real-hardware-range value in this repo (Physical Constants Audit §1). **Lower tuning surface, lower risk of a bad tune actively hurting behavior.** | Requires real SysId characterization (`kS/kV/kA`, currently generator-template placeholders per Drive Slot0 Retuning Audit §1) **and** `Slot0` `kP/kI/kD` retuning, done in the correct order (feedforward before feedback) to avoid a badly-tuned closed loop overshooting/oscillating — a real, avoidable regression if sequenced wrong. **Higher tuning surface, higher risk if rushed, higher ceiling if done right.** | Direct consequence of what each mode's field docs say it consults (Control Path Audit facts #15-16). |
| **FRC match robustness** | Predictable, doesn't degrade unexpectedly from a bad tune (there's very little to mistune) — but also doesn't adapt to in-match conditions (carpet wear over a match, battery discharge, defense contact). | Higher theoretical robustness to in-match physical variation **if and only if** correctly characterized and tuned — an incorrectly tuned closed loop can be *less* robust than a conservative open-loop default (oscillation, current pinning against `kSlipCurrent`, overshoot). Robustness is conditional, not automatic. | Reasoned from the above rows; matches CTRE's own general framing in the "Choosing Output Type" doc, which presents closed-loop control as a capability to be earned through correct characterization, not a free upgrade. |

**Note on what does NOT differ between the two modes, confirmed from source:** the chassis-level `PPHolonomicDriveController` PID (`kP=5`/`kP=3`, unclamped) and PathPlanner's per-module wheel-force feedforward computation are both upstream of this choice and unaffected by it (Control Path Audit §4). Whichever mode is chosen, the already-confirmed dominant lever on the existing ~6.9m tracking error (chassis PID) is untouched by this decision alone.

---

## 2. How CTRE and other teams' shipped code actually use `ApplyRobotSpeeds`

**Method:** read the actual `configureAutoBuilder()`-equivalent wiring in every reference codebase available locally
(`temp_reference/`, `C:\Users\xdm\frc-steal-from-the-best\`) that uses PathPlanner and/or CTRE's swerve API, plus
targeted web research. Public docs alone don't state an explicit recommendation (confirmed by direct WebSearch this
session — no CTRE changelog or docs page states a preferred `DriveRequestType` for autonomous); **shipped source code
is the strongest available evidence for "intended/accepted practice," and is what this section relies on.**

| Source | Autonomous/precision-tracking `DriveRequestType` | Teleop `DriveRequestType` | Citation |
|---|---|---|---|
| **CTRE's own official example** (`Phoenix6-Examples`, vendored locally as `temp_reference/Phenoix 6 API Examples/java/SwerveWithPathPlanner/.../CommandSwerveDrivetrain.java`) — the direct template this project's own `CommandSwerveDrivetrain.java` derives from | **Not set — defaults to `OpenLoopVoltage`**, same as this repo. **But unlike this repo, it calls `ChassisSpeeds.discretize(speeds, 0.020)`** before passing speeds to `m_pathApplyRobotSpeeds` | (not present in this file) | Lines 211-215: `m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020)).withWheelForceFeedforwardsX(...)...` |
| **Team 254 (2025, "The Cheesy Poofs")**, `temp_reference/Team 254 Code/.../DriveSubsystem.java` | **Explicitly `Velocity`** — a dedicated `pathplannerAutoRequest` field, live-used in the PathPlanner-following control loop | `OpenLoopVoltage` (their `stopRequest`, an idle/stop state, not a driving request — no teleop drive request was found using `Velocity` or `OpenLoopVoltage` in the reviewed file) | Lines 51-57 (`stopRequest`/`pathplannerAutoRequest` declarations), lines 117-129 (`pathplannerAutoRequest` used live: `setControl(pathplannerAutoRequest.withSpeeds(applyDeadband(speeds)).withWheelForceFeedforwardsX(...)...)`) |
| **Team 1678**, `temp_reference/Team 1678 Code/.../DriveConstants.java` | `Velocity` for `autoPIDRequest`, `choreoRequest`, and precision-aiming requests (`aimingRequest`, `hitTower`) | `OpenLoopVoltage` for `teleopRequest`, `HEADING_LOCK_REQUEST` | Lines 143-158, 193-197, 284-303 |
| **Team 6328 ("Mechanical Advantage")**, `temp_reference/Team 6328 Code/.../Drive.java`, `Module.java` | Doesn't use CTRE's swerve API's `DriveRequestType` enum at all — their `runVelocity()` (used for **both** teleop and autonomous, no open-loop path exists anywhere in this file) sends every module a closed-loop velocity target **with an explicit characterized feedforward model** (`ffModel.calculate(speedRadPerSec)`) | Same method, same closed-loop path — 6328 does not differentiate teleop vs. auto by control mode | `Drive.java:189-209` (`runVelocity`, includes `ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs)` at line 191), `Module.java:86-99` (`runSetpoint`, sets `driveVelocityRadPerSec` + `driveFeedforward`) |
| **WCP's own 2026 concept code**, `frc-steal-from-the-best/wcp-2026-concept/` | No PathPlanner/`AutoBuilder` wiring found in this repo — no evidence either way | `OpenLoopVoltage` for teleop drive commands (`ManualDriveCommand`, `AimAndDriveCommand`) | `ManualDriveCommand.java:43,50`, `AimAndDriveCommand.java:32` |
| **Community teaching resource** (Team 5712 "Gray Matter" workshop, `frc5712.com/swerve-calibration`, web-fetched this session) | Not autonomous-specific — recommends switching teleop `DriveRequestType` from `OpenLoopVoltage` to `Velocity` "for more precise speed tracking" | (this is the switch being recommended) | WebFetch of the page's "Update DriveRequestType (Teleop OpMode)" section |

**Pattern, stated plainly:** every reference codebase checked that customizes this setting away from the raw CTRE
template default chooses `Velocity` for autonomous or precision-tracking contexts, and reserves `OpenLoopVoltage` (if
used at all) for teleop, idle, or stop states. Zero counterexamples were found in the sample (no team's autonomous
path was found deliberately using `OpenLoopVoltage`). The sample is not large (3 teams with direct evidence, plus one
vendor concept repo and one community resource with partial/indirect evidence) — stated as a real limit on this
finding's strength, not glossed over.

**CTRE's own official example matches this repo's current choice exactly for `DriveRequestType`** — both leave it at
the unmodified default. This is meaningful: it means this repo's current architecture is not a mistake or an
oversight relative to CTRE's own reference, it's the literal unmodified template. **But CTRE's own example also
includes `ChassisSpeeds.discretize()`, which this repo's `configureAutoBuilder()` (`CommandSwerveDrivetrain.java:359-
363`) does not** — confirming, from a second independent source (CTRE's own example, in addition to 6328's
independent implementation), the discretization gap already flagged in `docs/Path_Following_Tuning_Readiness_Audit.md`
§4. This repo diverges from CTRE's own reference in the one place elite teams and CTRE agree, and matches it in the
one place elite teams diverge from it.

---

## 3. Does this project's architecture match CTRE's intended design?

**Partially, and not for the reason it might look like.** The `DriveRequestType` default match against CTRE's own
example (§2) could be read as "this repo deliberately follows CTRE's intended design" — but the same comparison shows
this repo is *missing* the one line (`ChassisSpeeds.discretize()`) that sits directly adjacent to it in that same
reference file. That combination — matching an unmodified default while missing an adjacent, also-unmodified-from-the-
template line that ships in the *same* reference example — is much more consistent with **this repo having inherited
the generator template's autonomous wiring largely as-is**, rather than the `OpenLoopVoltage` choice having been a
deliberate, evaluated architecture decision. Nothing in this repo's own code comments, `CLAUDE.md`, or session history
(checked before writing this section) documents `OpenLoopVoltage` as an intentional choice for autonomous — it reads
as template inertia, not a decision, which matters for how much weight to give it as "working as intended."

CTRE's own documented philosophy (the "Choosing Output Type" passage already cited in the prior audit, discussing
closed-loop output types generally) treats closed-loop, feedback-driven control as the more capable and generally
preferred approach **once properly characterized** — CTRE doesn't say open-loop is "correct" for autonomous, only
that it's a valid, simpler starting point requiring less setup. The swerve generator shipping `OpenLoopVoltage` as the
`ApplyRobotSpeeds` default reads as "safe zero-configuration starting point for a brand-new project," consistent with
every other generator default in `TunerConstants.java` being an unedited placeholder (Drive Slot0 Retuning Audit §1)
— not as a competitive recommendation.

---

## 4. If staying `OpenLoopVoltage`

**Tuning order** (unchanged from `docs/Autonomous_Control_Path_Audit.md` §6a, restated here for completeness):
1. Physical pre-flight (CANivore name, wheel-radius caliper check).
2. Resolve the wheel-force-feedforward-under-open-loop experiment (Control Path Audit §5a).
3. Retune `PPHolonomicDriveController` chassis PID (`kP=5`/`kP=3`) — confirmed dominant, unconditional lever.
4. Re-verify `kSpeedAt12Volts` empirically.
5. SysId/`Slot0` work proceeds as an independent teleop/`trackTarget`-quality project, not gated on autonomous.
6. Discretization, auto regression suite expansion.

**Highest-impact remaining improvements, ranked by this audit's new evidence:**
1. **Add `ChassisSpeeds.discretize(speeds, <loop period>)`** to `configureAutoBuilder()`'s lambda, matching both
   CTRE's own official example and Team 6328's independent implementation exactly. This is now the single
   best-evidenced, lowest-risk improvement identified across all three audits — two independent, unrelated sources
   (a vendor and a top-tier software team) converge on doing this, and it's currently entirely absent from this repo.
2. Chassis-level PID retune (unchanged from prior audit — still the dominant confirmed lever).
3. `kSpeedAt12Volts`/wheel-radius re-verification.
4. Resolve the open-loop wheel-force-feedforward question.

---

## 5. If switching to `Velocity`

### Exact code changes required

1. **`CommandSwerveDrivetrain.java`, `configureAutoBuilder()`'s output lambda (lines 359-363):** add
   `.withDriveRequestType(DriveRequestType.Velocity)` to the `m_pathApplyRobotSpeeds` builder chain — one line,
   directly mirroring Team 254's confirmed pattern (`pathplannerAutoRequest`, §2). `DriveRequestType` is already
   imported in this file (`CommandSwerveDrivetrain.java:14`, confirmed — used elsewhere for `driveToPOI`/`trackTarget`).
2. **Precondition, not simultaneous:** `TunerConstants.driveGains` (`kS/kV/kA`) must be SysId-characterized and
   `kP/kI/kD` retuned *before* step 1 is applied — switching the control mode against the current unedited generator
   placeholders (`kP=0.1, kS=0.1, kV=0.124, kA` unset) would run autonomous through a closed loop with no real
   characterization behind it, a genuine regression risk (see below).
3. Optional, but recommended regardless of this decision per §4 item 1: add `ChassisSpeeds.discretize()` at the same
   call site.

**No other files require changes for the switch itself** — `sanitizeAutoSpeeds()`, `LoggingHolonomicDriveController`,
`AutoBuilder.configure()`'s call signature, and PathPlanner's `settings.json` are all confirmed independent of
`DriveRequestType` (Control Path Audit §4, §5).

### Risks

- **Sequencing risk (the main one):** applying step 1 before step 2 runs autonomous through an uncharacterized closed
  loop — plausible failure modes include oscillation, overshoot, or current pinning against `kSlipCurrent=120A`
  during otherwise-routine tracking, none of which are possible under the current open-loop path (§1's "tuning
  complexity" row).
- **Regression-suite invalidation:** existing auto regression goldens (`LtNeutralAutoRegressionTest`) were established
  against open-loop behavior; switching modes is a genuine dynamics change and would require re-baselining, not
  reuse of the existing golden as a pass/fail bar.
- **Doesn't address the confirmed-dominant lever:** per §1's summary note, the chassis-level `PPHolonomicDriveController`
  PID is unaffected by this switch — expecting this change alone to fix the ~6.9m tracking error would be
  unsupported by the evidence; that error's dominant confirmed cause sits upstream of this decision.
- **Process risk, not technical:** this is a real behavior change (not a constants edit), and per this project's own
  established workflow (`CLAUDE.md`'s Karpathy Guidelines, and the pattern already used for e.g. the VT-input-relay
  migration — design → plan → execute → live validation before merge) it should go through the same design/plan/
  live-validation cadence rather than being applied directly, even though the code change itself is small.

### Expected benefit

Evidence-supported, stated at the confidence level the evidence actually supports: **a plausible improvement to
module-level shaft-speed tracking fidelity under torque disturbances** (defense contact loading, carpet friction
variation, moderate battery sag within available headroom) — consistent with why every reference team with evidence
in §2 makes this same choice for their own autonomous/precision paths. **Not** a confirmed fix for ground-relative
wheel slip (§1's wheel-slip row — orthogonal, governed by `kSlipCurrent` either way) and **not** a substitute for the
chassis-level PID retune, which remains the higher-leverage, independent, already-confirmed item regardless of this
decision.

### Regression tests that must pass

Per this project's own established verification-loop mandate (`CLAUDE.md`'s "Advanced Agent Verification Loop"):
- `./gradlew compileJava` (gate 1, mandatory).
- `python SKILLS/run_headless_sim.py` (gate 2, mandatory).
- `./gradlew test` (gate 2.5, mandatory for any behavior claim) — specifically:
  - `LtNeutralAutoRegressionTest` (and any other established auto regression tests) — expect the existing golden to
    need re-baselining, not to pass unchanged, per the risk noted above.
  - `CommandSwerveDrivetrainSanitizeSpeedsTest` — `sanitizeAutoSpeeds()` itself is untouched by this change, but it
    sits immediately downstream of the modified lambda and should be re-confirmed passing, not assumed.
  - `SimSpawnPoseOwnershipTest` — unrelated to this change, but part of the full suite; a regression here would
    indicate an unrelated problem, not this change, and should be investigated as such rather than dismissed.
- `python SKILLS/parse_akit_log.py` (gate 3) against a real `.wpilog` from a post-change sim or hardware run, to
  independently verify the behavior claim rather than trusting the JUnit suite's green result alone.

---

## Recommendation

**Switch to `Velocity`, conditional on completing SysId characterization first (§5's step 2 before step 1).**

This is not a "prefer closed-loop in general" preference — it's the conclusion the evidence in §1 and §2 actually
supports: every reference implementation with direct evidence (Team 254, Team 1678, Team 6328) independently arrives
at closed-loop, feedback-driven control for autonomous/precision-tracking contexts, while reserving open-loop for
teleop or idle states; CTRE's own general documentation frames closed-loop as the more capable mode once earned
through characterization, not as something to avoid; and this repo's current match to CTRE's open-loop *default*
reads as inherited template state (§3) rather than an evaluated decision, weakening any argument that
`OpenLoopVoltage` was chosen deliberately for a good reason specific to this robot.

The recommendation is explicitly conditional and sequenced, not unconditional, because the evidence in §1 (tuning
complexity, regression risk) and §5 (sequencing risk) is just as real as the evidence favoring the switch — an
uncharacterized closed loop is a plausible regression, not a hypothetical one. **This recommendation is not a claim
that switching alone will fix the currently-documented autonomous tracking error** — §1's closing note and §5's
"expected benefit" both state plainly that the confirmed-dominant lever (`PPHolonomicDriveController`'s unclamped
chassis PID) is independent of this choice and should be pursued regardless of which `DriveRequestType` is active.

**What would change this recommendation:** if the §5a wheel-force-feedforward experiment (from
`docs/Autonomous_Control_Path_Audit.md`) shows PathPlanner's precomputed forces already meaningfully compensate under
the current open-loop path, that would narrow the gap this recommendation is based on — worth running before
committing engineering time to the switch, since it's cheap (a telemetry A/B, no committed code change) relative to a
full SysId-characterize-then-switch effort.

---

## Verification

`git status --short`: clean before this document was written, and confirmed clean of any tracked-file changes after —
only this new file is untracked. No `TunerConstants.java`, `CommandSwerveDrivetrain.java`, `CLAUDE.md`, or any other
production/tracked file was modified. No constants were changed. No commit was made.
