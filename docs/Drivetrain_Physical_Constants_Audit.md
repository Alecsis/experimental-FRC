# Drivetrain Architecture Audit — Physical Constants for Real-World Bring-Up

**Audit date:** 2026-07-19. **Scope:** investigation only — no files were modified to produce this
report. Reviewed `TunerConstants.java`, `CommandSwerveDrivetrain.java`, `RobotContainer.java`,
`Constants.java`, AutoBuilder/`RobotConfig` wiring, and `pathplanner/settings.json`; cross-referenced
against WCP's published SwerveX2S documentation and this repo's own prior-session history in
`CLAUDE.md`.

**Headline finding:** most values are internally consistent between `TunerConstants.java` and
`pathplanner/settings.json`, and several (gear ratios, motor type, steer ratio) check out cleanly
against WCP's public docs. But there are two concrete problems worth mentor attention before
bring-up: a wheel-radius value that doesn't match WCP's stock module, and stray duplicate
PathPlanner config files sitting inside the deploy tree (§4).

---

## 1. Verified values

Cross-referenced and confirmed correct/consistent from at least two independent sources (code +
vendor docs, or code + mentor-confirmed measurement per `CLAUDE.md`).

| Value | Where | Value | Verified against |
|---|---|---|---|
| Drive gear ratio | `TunerConstants.java:85` (`kDriveGearRatio`) | `3.7142857142857144` (26/7) | Matches WCP SwerveX2S "X3" tier top pinion. WCP's published X3 free-speed range is ~22.3–24.7 ft/s for Kraken X60; this repo's resulting `kSpeedAt12Volts = 7.52 m/s` = 24.67 ft/s sits right at the top of that range, consistent with the fastest X3 pinion option. |
| Steer gear ratio | `TunerConstants.java:86` (`kSteerGearRatio`) | `25.9` | Matches WCP's published SwerveX2S steer ratio exactly (25.9:1, constant across all X2S drive-ratio tiers per WCP docs). |
| Max speed at 12V | `TunerConstants.java:79` (`kSpeedAt12Volts`) | `7.52 m/s` | Consistent with the drive ratio above; also duplicated correctly into `settings.json:23` (`maxDriveSpeed: 7.52`) — see §2. |
| Drive motor / steer motor type | `TunerConstants.java:44,46` | `TalonFX_Integrated` (Kraken X60 both) | Matches `CLAUDE.md`'s twelfth-session mentor confirmation: "entire drivetrain, shooter, indexer, and pivot motors are Kraken X60." Also matches `settings.json:24` (`driveMotorType: "krakenX60"`). |
| Robot mass, bumper footprint | `Constants.java:68-70`, `settings.json:2-3,18` | 43.545 kg, 0.686m × 0.813m | Internally consistent across both files — **but see §4 for a contradiction about whether this was ever actually measured.** |
| Module positions (wheelbase/trackwidth) | `TunerConstants.java:138-172` vs `settings.json:27-34` | X=±0.2921m (11.5in), Y=±0.4064m (16in) all four corners | Exact match — 0.2921m = 11.500in and 0.4064m = 16.000in to the reported precision, both sides. Derived wheelbase 23in / trackwidth 32in. |
| `robotTrackwidth` | `settings.json:20` | `0.8128` | = 2 × 0.4064 (module Y-spacing above), consistent with the module positions it's supposed to summarize. |
| Front-left / back-right CANcoder offset both `0.066650390625` | `TunerConstants.java:134,167` | — | Looks like a copy-paste bug at first glance but is **not** — already investigated and closed in `CLAUDE.md`'s Backlog: "mentor confirmed the existing offsets... are correct as-is; not a calibration paste" (2026-07-18). Re-flagging this without that context would waste a future session's time — noted here so this audit doesn't look like it missed something. |
| CAN ID isolation | `TunerConstants.java` (11-13,21-23,31-33,41-43, Pigeon=1) vs `Constants.java` (2,3,4,5,6) | No collisions | Drivetrain devices + Pigeon live on the `"Swerve"` named bus (`TunerConstants.java:75`, via `DrivetrainConstants.withCANBusName(...)`); Intake/Shooter motors (`Constants.java:32-38`) are constructed via `RobotMotor(config.canId)`, which explicitly documents using the default `TalonFX(int)` constructor → **rio** bus (`RobotMotor.java:51`). Different physical buses, so even the shared-numeral overlap risk (none exists here, but would be harmless) is moot. |
| PathPlanner `RobotConfig` source | `CommandSwerveDrivetrain.java:353` | `RobotConfig.fromGUISettings()` | Loads `settings.json` at runtime — confirmed this is the single source of truth PathPlanner uses; `Constants.java`'s drivetrain-sim constants are a separately-maintained mirror for MapleSim, not a second copy PathPlanner reads (see §2). |

---

## 2. Duplicated values that must stay synchronized

These physical constants are declared **independently in two or more places** with no code-level
link between them. Nothing enforces agreement — a future edit to one side and not the other is a
silent correctness bug, not a compile error.

| Physical quantity | Copy 1 | Copy 2 | Copy 3 |
|---|---|---|---|
| Wheel radius | `TunerConstants.java:87` `kWheelRadius = Inches.of(2)` | `settings.json:21` `driveWheelRadius: 0.0508` (= 2.000in) | — |
| Drive gear ratio | `TunerConstants.java:85` `3.7142857142857144` | `settings.json:22` `driveGearing: 3.714286` (rounded to 6 places) | — |
| Max drive speed | `TunerConstants.java:79` `7.52 m/s` | `settings.json:23` `maxDriveSpeed: 7.52` | — |
| Slip current / drive current limit | `TunerConstants.java:54` `kSlipCurrent = Amps.of(120)` | `settings.json:25` `driveCurrentLimit: 120.0` | — |
| Robot mass | `Constants.java:68` `43.545` | `settings.json:18` `robotMass: 43.545` | — |
| Bumper/robot footprint | `Constants.java:69-70` `0.686 × 0.813` | `settings.json:2-3` `robotLength/robotWidth: 0.686/0.813` | — |
| Wheel COF | `Constants.java:74` `kWheelCOF = 0.8` | `settings.json:26` `wheelCOF: 0.8` | — |
| Module X/Y positions | `TunerConstants.java:138-172` (4 modules) | `settings.json:27-34` (`flModuleX/Y` etc.) | — |

None of these are wired together programmatically (e.g. neither file reads the other), so **every
one of these pairs is a manual-sync hazard**: currently all agree, but nothing prevents drift.
`Constants.java`'s own comment (lines 64-66) already acknowledges this for the mass/footprint/COF
group ("kept in sync with pathplanner/settings.json... Mass/footprint below match settings.json's
[fields]") — worth extending that awareness to the `TunerConstants.java` pairs above too, which
carry no equivalent comment.

**Near-miss, not a real hazard:** `robotTrackwidth: 0.8128` (`settings.json:20`) is very close to
but not identical to `robotWidth: 0.813` (`settings.json:3`, the *bumper* width) — 0.8128m =
32.000in exactly vs 0.813m ≈ 32.008in. These describe different things (module spacing vs. bumper
outer edge) so they're not supposed to be equal, just close, which they are. Flagged only so a
future reader doesn't mistake the ~0.3mm gap for an error.

---

## 3. Placeholder estimates requiring physical-robot measurement

Carried over/confirmed from `CLAUDE.md`'s own Backlog, cross-checked against the code comments
that mark them:

| Value | Location | Status |
|---|---|---|
| Wheel coefficient of friction (`wheelCOF` / `kWheelCOF = 0.8`) | `settings.json:26`, `Constants.java:74` | Explicitly commented as unmeasured in both places: `Constants.java:71-73` says "Neither this nor settings.json's wheelCOF is a measured value... TODO: validate against the real wheel/carpet pairing." Every recent traction-related finding in `CLAUDE.md` (the sixteenth-session current-limit/traction analysis) pivots on this number — highest-value item to actually measure. |
| `robotMOI = 4.1` | `settings.json:19` | No corresponding measurement or derivation documented anywhere in `Constants.java` or `CLAUDE.md`. MapleSim has no equivalent field (`CLAUDE.md`: "robotMOI remains PathPlanner-only... inertia is derived internally from mass+geometry" on the sim side), so this number is consumed by PathPlanner's trajectory generator only, and is unverified against the real chassis. |
| Robot mass / bumper footprint measurement provenance | `Constants.java:66-70` | **See §4 — this is not just "still a placeholder," it's actively contradictory with `CLAUDE.md`'s own history.** |
| Drive `Slot0` `kS`/`kV`/`kA` (`TunerConstants.java:32-34`) | Unedited CTRE Tuner X generator defaults (`kS=0.1, kV=0.124, kA` unset) | Not a placeholder in the "needs code review" sense — already the subject of the separate SysId audit (`docs/SysId_Characterization_Checklist.md`). Included here only for completeness against the "verify every physical constant" ask. |
| Coupling ratio (`kCoupleRatio = 3.57`, `TunerConstants.java:82`) | Generator/vendor-nominal value | CTRE's own documented procedure for this constant is an **empirical, per-module measurement** (lock the drive wheel, rotate the azimuth exactly 3 full turns by hand, read the drive motor's reported rotation count, divide) — it is not purely a vendor spec sheet lookup, because manufacturing tolerance can shift it module to module. `3.57` matches the commonly-cited WCP SwerveX2S nominal, but this repo has no record of the lock-and-rotate check having been performed on this specific robot. Cheap to verify at bring-up. |

---

## 4. Inconsistencies and maintenance hazards

### 4a. Wheel radius does not match WCP's stock SwerveX2S wheel — needs a caliper check

`TunerConstants.java:87` (`kWheelRadius = Inches.of(2)`) and `settings.json:21`
(`driveWheelRadius: 0.0508`) both specify a **4-inch-diameter (2in radius)** wheel. WCP's own store
listing for the SwerveX2S's stock molded wheel hub
([WCP Swerve X2S Molded Wheel Hub](https://wcproducts.com/products/wcp-1621)) specifies
**3.5" OD × 1.75" WD** — a 3.5-inch-diameter (1.75in radius) wheel, confirmed independently by a
WCP blog post describing the same wheel as "3.5 inch diameter... 1.75 inch wide."

That's a **~14% radius discrepancy** (2.0in coded vs. 1.75in stock), and wheel radius is a
direct multiplier on every distance-derived quantity this drivetrain computes: odometry
distance-per-motor-rotation, the drive `kV`/speed relationship, and PathPlanner's wheel-force
feedforward. A 14% error here would show up as odometry consistently over- or under-reporting
distance traveled relative to reality, at exactly the kind of magnitude that's easy to misdiagnose
as a tuning problem instead of a wrong constant.

This is **not automatically a bug** — running a larger 4in wheel/tread on a WCP X2S hub (instead of
WCP's own stock 3.5in molded wheel) is a common team modification, and this repo's twelfth-session
mentor confirmation only covers "WCP SwerveX2S at the X3 gearing tier," not the specific wheel
installed. But nothing in this repo documents that substitution having been made deliberately,
which is exactly the gap CAD/vendor docs can't close on their own. **Action: measure the actual
installed wheel diameter with calipers before trusting `kWheelRadius`/`driveWheelRadius` for
anything precision-sensitive** (odometry-based auto tuning, the SysId work described in the
companion audit, etc).

### 4b. In-code comment claims a real measurement that `CLAUDE.md`'s own history says didn't happen

`Constants.java:64-67`:
```java
// Drivetrain simulation (maple-sim) physical constants -- kept in sync with
// pathplanner/settings.json so PathPlanner's trajectory generation and MapleSim's physics
// simulate the same robot. Mass/footprint below match settings.json's robotMass/robotLength/
// robotWidth (mentor-confirmed against a real scale/tape measurement, 2026-07-18).
```

This reads as a claim that the 43.545kg mass and 0.686×0.813m bumper footprint were confirmed
against **an actual scale and tape measure** on 2026-07-18. But `CLAUDE.md`'s own Backlog section,
describing that same date's session, says the opposite:

> "Update (twelfth session, 2026-07-18): `Constants.java` and `pathplanner/settings.json` are now
> internally *consistent* with each other... **but consistency is not measurement.**"

and lists "measure the real robot's mass... against reality" as still open, deferred because there
was "no live robot access while running sim-only" that session. **These two statements directly
contradict each other about whether a physical measurement occurred.** A future session (or a
mentor skimming just the code comment, not the full `CLAUDE.md` history) could reasonably walk away
believing this number is measurement-verified when the project's own authoritative changelog says
it explicitly is not. **Recommend resolving this discrepancy** — either the comment is wrong and
should be corrected to match `CLAUDE.md` (most likely, given `CLAUDE.md` is the more detailed,
session-dated record), or `CLAUDE.md`'s Backlog is stale and the measurement genuinely happened
since, in which case the Backlog item should be closed. Left as-is, whichever one is correct is now
ambiguous from the repo alone.

### 4c. Stray, stale, deeply-nested duplicate PathPlanner config files inside `autos/`

While reviewing `pathplanner/settings.json` and the deploy directory, found this unexpected
structure inside `src/main/deploy/pathplanner/autos/` (which should contain only `.auto` files):

```
src/main/deploy/pathplanner/autos/
├── *.auto                                          (13 real auto files, expected)
├── deploy/pathplanner/navgrid.json                 (stray duplicate, STALE)
├── deploy/pathplanner/deploy/pathplanner/navgrid.json   (stray duplicate, STALE, identical to the above)
└── docs/superpowers/specs/                         (stray, empty directory — no files in it)
```

The two nested `navgrid.json` copies are **not** identical to the real one at
`src/main/deploy/pathplanner/navgrid.json` — a `diff` shows the real file has additional obstacle
cells marked in the field grid (rows 13-14) that the nested copies lack, meaning the real file has
been edited more recently than these stray copies were made. The `docs/superpowers/specs/` path is
especially telling — that's this repo's own planning-doc directory structure
(`docs/superpowers/specs/`, matching the real one at the top-level `docs/superpowers/specs/`)
showing up nested three-deep inside a PathPlanner autos folder, which has nothing to do with
PathPlanner. This looks like the residue of some tool or script having recursively copied part of
the repo into the wrong working directory at some point, rather than anything PathPlanner itself
would create.

**Why this matters for drivetrain bring-up specifically:** `src/main/deploy/` is copied wholesale
to the roboRIO by the WPILib Gradle deploy task, so this cruft ships to the robot on every deploy.
More importantly, if anyone (mentor or student) ever opens one of the nested stale `navgrid.json`
files by mistake instead of the real one — e.g. via a fuzzy file search matching "navgrid" — edits
made there will silently do nothing, since PathPlanner's desktop app only reads the top-level
`deploy/pathplanner/navgrid.json`. Not a functional risk to the robot today (PathPlanner's autos
loader appears to ignore non-`.auto` entries in that folder), but it's dead weight and a real
"which file is the real one" trap for a future session. **Recommend deleting
`src/main/deploy/pathplanner/autos/deploy/` and `src/main/deploy/pathplanner/autos/docs/`** — left
untouched here per this task's investigation-only scope.

### 4d. `CANBus` name requires a physical CANivore, unverified

`TunerConstants.java:75`: `public static final CANBus kCANBus = new CANBus("Swerve", "./logs/example.hoot");`
names the drivetrain's CAN bus `"Swerve"` — this must correspond to a CANivore device on the real
robot that has been named `"Swerve"` in Phoenix Tuner X (CANivore names are set per-device, not
derived from this string). Nothing in this repo's audit trail confirms that naming has been done on
the physical CANivore, as opposed to the robot instead using the default `"rio"` bus name for
everything (which would make every drivetrain device fail to enumerate at boot, since they're all
configured for the `"Swerve"` bus name). **Requires a physical-robot check at bring-up**, ideally
the very first thing verified — a bus-name mismatch here would present as "the whole drivetrain is
dead," which is a scary and non-obvious first bring-up symptom if this is the actual cause.

---

## 5. Summary table — action items by category

| Category | Item |
|---|---|
| **Ready / verified — no action** | Drive gear ratio, steer gear ratio, max speed, motor types, module X/Y positions, CAN bus isolation, CANcoder offset "duplicate" (previously investigated), RobotConfig source. |
| **Physical measurement needed** | Wheel radius (§4a — most urgent, disagrees with vendor stock spec), wheel COF, robot MOI, coupling ratio empirical check, mass/footprint (re-verify given §4b's contradiction). |
| **Documentation fix needed** | Resolve the `Constants.java` comment vs. `CLAUDE.md` Backlog contradiction about mass/footprint measurement provenance (§4b). |
| **Repo cleanup needed** | Remove stray nested `autos/deploy/` and `autos/docs/` directories (§4c) — not done here, investigation-only. |
| **Bring-up verification needed** | Confirm the physical CANivore is named `"Swerve"` in Phoenix Tuner X before first power-on (§4d). |
| **Sync-hazard awareness** | The eight duplicated-value pairs in §2 have no code-level link; any future edit to one side needs a matching edit on the other, by convention only. |
