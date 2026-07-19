# Sim Timing Determinism — What's Fixed, What Isn't, and Where Each Kind of Test Belongs

## Why this exists

A prior investigation found 0.20–0.37 m RMS lateral-error variance across repeated trials of
*identical* nominal code in sim, tracing to CTRE Phoenix 6's native `SwerveDrivetrain.OdometryThread`
running on its own real-time-clocked thread (`Utils.getCurrentTimeSeconds()` → native JNI), separate
from WPILib's `SimHooks`-controlled clock that `maple-sim`'s own physics `Notifier` and JUnit test
harnesses use. This session added a low-risk, sim-only mitigation
(`CommandSwerveDrivetrain.startSimThread()`: raises the update frequency of each module's
drive/steer position+velocity, CANcoder absolute position, and Pigeon2 yaw/angular-velocity signals
to 250 Hz under `Utils.isSimulation()`, per CTRE's own documented "High Fidelity CAN Bus Simulation"
guidance) and measured it: same auto (`LB Neutral`), 3 trials each, mitigation off vs on.

| | RMS lateral error per trial | Spread |
|---|---|---|
| Mitigation off | 0.112, 0.289, 0.186 m | ~0.177 m |
| Mitigation on | 0.087, 0.161, 0.067 m | ~0.094 m |

Both the mean and the spread roughly halved. Encouraging — but n=3 per condition is small, and this
does **not** establish statistical significance. Treat it as "the mitigation helps, directionally,"
not as a determinism proof.

## 1. Full bit-for-bit deterministic pose simulation is not expected

The signal-frequency change narrows the window where stale simulated-CAN data could be read, but it
does not and cannot touch CTRE's native `OdometryThread` itself — no public API exposes a way to
replace or synchronously step it. That thread's timestamps come from a native clock independent of
`SimHooks`. As long as we use CTRE's real (sanctioned-exception) `SwerveDrivetrain`/`Phoenix 6` stack
in sim rather than a from-scratch drivetrain, some run-to-run timing variance in continuous pose
numerics should be expected. Don't chase eliminating it further without changing that architecture,
which is explicitly out of scope.

## 2. Continuous PID benchmarking → deterministic unit tests with mocked pose/setpoint suppliers

Any test that measures a *continuous* numeric quantity sensitive to exact timing (RMS tracking
error, PID gain comparisons, disturbance-response magnitude) should stub the pose/setpoint suppliers
directly and bypass `SwerveDrivetrain`/maple-sim entirely, per the trajectory-error-instrumentation
spec's original Gate 2.5. This sidesteps the native-thread timing domain altogether and is the only
way to get genuinely reproducible numbers for gain comparisons.

## 3. Full MapleSim → route completion, collision, and integration testing

Full sim (real `CommandSwerveDrivetrain`, real `AutoBuilder`-built autos, real MapleSim physics) is
the right tool for coarse pass/fail questions that tolerate this level of jitter: does an auto
finish, does it get physically blocked by a field obstacle, does a state machine transition
correctly end-to-end. The Bump-collider validation earlier this session (0.0 m vs 4.78 m of X
progress, same auto, collider on vs off) is the model case — a binary/coarse outcome, unaffected by
±0.1 m of continuous tracking noise.
