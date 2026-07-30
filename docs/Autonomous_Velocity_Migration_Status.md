# Autonomous Velocity Migration Status

Verified 2026-07-30.

## Current decision

The migration remains hardware-gated. Autonomous production path following still uses `DriveRequestType.OpenLoopVoltage`; no production `Velocity` request switch has been made or authorized.

## Completed evidence

- Phase 0 discretization and bounded auto-speed preparation: `309a89b`.
- Phase 1 simulation-only SysId workflow validation: `7518244`, `2ac441a`, and `1991bd3`.
- Forward and reverse simulation fits demonstrate that the analysis pipeline works, but they are placeholders only. They are not valid real-robot characterization data.

## Required before the next migration phase

1. On the physical robot, characterize drive `Slot0` with the real mechanism, battery, and traction conditions.
2. Review the measured `kS`/`kV`/`kA` values and decide whether to update `TunerConstants.driveGains`.
3. Only after that evidence and explicit approval, implement and validate the production `DriveRequestType.Velocity` switch.

Do not paste the simulation-derived values into `TunerConstants.driveGains`, and do not treat a green simulation test as proof that the real robot is ready for closed-loop autonomous control.

## Verification snapshot

The final verification run on 2026-07-30 passed `compileJava`, the 12-second headless simulation launch, and the full JUnit suite: **65 tests, 0 failures, 0 errors, 0 skipped**. A preceding full-suite run reproduced the known intermittent `ResetPoseHeadingSimTest` heading race; its standalone rerun passed, followed by a clean full-suite rerun. This does not remove the underlying nondeterminism from the open-questions list.
