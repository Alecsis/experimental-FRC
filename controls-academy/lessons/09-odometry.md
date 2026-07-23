# Lesson 9: Odometry

## Learning objective
Integrate Lesson 8's module states over time into a pose estimate using wheel encoders + gyro
alone, and directly observe why wheel-only odometry drifts — the problem Lesson 10's pose
estimation exists to fix.

## Concepts introduced
- `SwerveDriveOdometry` (module positions + gyro angle → `Pose2d`)
- Why odometry is *relative*-only (accumulates error, has no absolute-position correction)
- Sources of real-world drift this lesson can only partially simulate: wheel slip, encoder
  quantization, gyro drift — and which ones matter most

## Existing code to reuse
None directly (this repo's odometry lives inside CTRE's generated `SwerveDrivetrain` base class,
not hand-rolled — so there's no production odometry code to read here the way there was for
SysId/kinematics). This lesson is closer to first-principles than most others.
**Required reading first:** `docs/claudex/design/sim-timing-determinism.md` — its finding that
CTRE's native odometry thread runs on a real-time clock independent of `SimHooks` (causing
0.1-0.3m run-to-run noise even with identical code) means any drift you measure in this lesson's
demo must come from a deterministic tick-stepped harness, not a live sim loop, or your "drift"
measurement is actually clock jitter.

## New simulation command / demo
`academy.odometry.OdometryDriftTest` (JUnit, deterministic tick-stepped): drive Lesson 8's chassis
in a closed loop (e.g. a square path back to the start), integrate odometry the whole way, and
measure the final position error against the known-true starting pose — with zero wheel slip
injected first (should be near-zero drift), then with a small injected slip term (visible drift).

## What to observe in Phoenix Tuner X
N/A (simulation).

## What to observe in AdvantageScope
Overlay the "true" simulated pose against the odometry-estimated pose on a 2D field-view plot —
with the closed-loop square path, a good run should end near-coincident; the slip-injected run
should show visible endpoint separation.

## Common failure modes
- Measuring drift over a live (non-deterministic) sim loop and blaming your integration math for
  noise that's actually the real-time-clock artifact documented in `sim-timing-determinism.md`.
- Forgetting gyro angle entirely and doing odometry from wheel encoders alone — technically
  possible (differential-drive style) but throws away information a swerve chassis has and
  produces much worse heading drift.
- Not resetting/zeroing the gyro at a known start heading before comparing against a "true" pose
  — an offset heading reference makes every subsequent comparison meaningless.

## Small experiments to build intuition
1. Run the closed-loop square path with zero slip — confirm near-zero final drift (validates the
   harness itself, not just the concept).
2. Introduce a small constant wheel-slip factor and re-run — measure how final drift scales with
   path length (odometry error compounds, it doesn't average out).
3. Introduce a small constant gyro bias instead of wheel slip — compare the *shape* of the
   resulting drift (heading-driven drift looks different from translation-driven drift on the
   field-view plot) to build intuition for diagnosing which sensor is at fault later.

## Success criteria
`OdometryDriftTest`'s zero-slip case shows near-zero drift (confirming your harness is correct),
and you can visually distinguish a wheel-slip-drift trace from a gyro-bias-drift trace on the
AdvantageScope field view. Move to Lesson 10 once both hold.
