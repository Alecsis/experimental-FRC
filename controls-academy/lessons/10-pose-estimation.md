# Lesson 10: Pose Estimation

## Learning objective
Fuse a simulated vision measurement into Lesson 9's odometry using `SwerveDrivePoseEstimator`,
and — deliberately — inject realistic vision noise/latency, directly addressing a documented gap
in the production codebase (`VisionIOSim` returns a "perfect" measurement every cycle, so the
real trust filter never gets exercised in sim).

## Concepts introduced
- `SwerveDrivePoseEstimator` (odometry + timestamped vision measurements, Kalman-style fusion)
- Standard deviations as a trust knob (`stateStdDevs`/`visionMeasurementStdDevs`) — not a
  correctness dial, a *confidence* dial
- Measurement latency / timestamping — why vision updates need `addVisionMeasurement(pose,
  timestampSeconds, stdDevs)` rather than being applied at "now"
- Jump-gate style rejection of clearly-bad vision measurements (the concept behind this repo's
  production vision trust filter)

## Existing code to reuse
`src/main/java/frc/robot/subsystems/vision/VisionIOSim.java` is explicitly the **counterexample**
here, not a template: it returns a zero-noise, zero-latency "perfect" measurement every cycle
(confirmed in the 18th-session Autonomous Architecture Audit as gap #2 — the production trust
filter is real but never exercised in sim). This lesson's job is to build the noisy/latent/
occasionally-wrong vision sim that production is missing, as a standalone academy exercise.

## New simulation command / demo
`academy.odometry.NoisyVisionSim` — generates timestamped pose measurements with configurable
Gaussian noise, latency, and an occasional deliberately-wrong outlier. `academy.odometry.
PoseEstimatorFusionTest` (JUnit): drive Lesson 9's drift scenario, fuse in noisy vision at a few
different std-dev settings, and compare fused-pose error against odometry-only error.

## What to observe in Phoenix Tuner X
N/A (simulation).

## What to observe in AdvantageScope
Three overlaid traces: true pose, odometry-only estimate, and fused estimate. Confirm fused
tracks closer to true than odometry-only over a long run, and — with the outlier-injection case
— confirm a badly-set (too-trusting) std-dev config lets one bad vision frame yank the fused pose,
while a properly jump-gated config rejects it.

## Common failure modes
- Setting vision std-devs too low ("trust vision completely") — one outlier frame teleports the
  fused pose; this is the literal failure mode the production trust filter exists to prevent.
- Setting vision std-devs too high ("barely trust it") — fusion converges to odometry-only
  behavior, silently wasting the vision system entirely.
- Applying vision measurements without proper timestamping — using "now" instead of the
  measurement's actual capture time introduces a lag-dependent bias, worse at higher latency.

## Small experiments to build intuition
1. Run fusion with well-tuned std-devs against a run with occasional outliers — confirm the
   fused pose ignores or heavily discounts single bad frames (this is the jump-gate concept, made
   concrete instead of theoretical).
2. Sweep vision std-dev from very low to very high across identical noisy-but-not-outlier data —
   find the qualitative crossover from "reacts too much to noise" to "ignores vision entirely."
3. Deliberately mis-timestamp vision measurements (apply them "late") and observe the resulting
   fused-pose lag, distinguishing it from the noise-driven jitter of experiment 2.

## Success criteria
`PoseEstimatorFusionTest` shows fused error measurably lower than odometry-only error on the
noisy-vision case, and separately demonstrates outlier rejection at a reasonable std-dev setting.
You should be able to explain why `VisionIOSim`'s "perfect" measurement is a real testing gap, not
just a simplification. Move to Lesson 11 once both hold.
