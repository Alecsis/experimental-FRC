// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.fail;

import org.wpilib.driverstation.DriverStation;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Subsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * A WALL-CLOCK-BOUNDED wrapper around the paused-simulation stepping that every Robot-booting test
 * in this package drives.
 *
 * <p><b>Why this exists.</b> {@code SimHooks.stepTiming()} advances the paused HAL clock and blocks
 * until the notifier alarms scheduled inside that window have been serviced. The 20 ms alarm belongs
 * to the robot's own {@code startCompetition()} loop, running on a background thread the test class
 * started. If that thread dies -- an uncaught exception out of any {@code xxxInit()} kills it
 * silently, because a bare {@code new Thread(robot::startCompetition)} has no handler -- then
 * nothing will ever service that alarm again and {@code stepTiming()} blocks in native code
 * FOREVER.
 *
 * <p>{@code @Timeout} does not bound that. JUnit's default timeout thread mode is SAME_THREAD, so
 * the only tool it has is {@code Thread.interrupt()} on the test thread, and interrupting a thread
 * parked inside a JNI call is a no-op: the flag is set, the native wait ignores it, the method never
 * returns, and the timeout can only be reported once the call comes back -- which it does not. The
 * forked test JVM then never exits and Gradle waits on it indefinitely. That is the failure mode
 * this class removes.
 *
 * <p><b>How the bound works.</b> Every {@code stepTiming()} call runs on a single reusable DAEMON
 * thread and is awaited with a hard ceiling. Exceeding the ceiling is a test FAILURE carrying the
 * captured robot-loop stack trace and a snapshot of scheduler/DriverStation/mechanism state -- not a
 * hang. The daemon thread is deliberately expendable: if it is wedged in native code it can never be
 * unwedged, but a daemon thread does not keep the JVM alive, so the fork still terminates and the
 * suite still finishes. Once a step has blown its ceiling the clock is latched dead and every later
 * step fails IMMEDIATELY, so one wedged loop costs one ceiling, not one per remaining step.
 *
 * <p>The ceiling is a watchdog, not a tolerance. A healthy {@code stepTiming()} call in this project
 * returns in single-digit milliseconds; {@link #maxObservedStepMillis()} is reported at shutdown so
 * that claim stays measured rather than assumed.
 */
public final class SimRobotLoop {
  /**
   * Wall-clock ceiling for ONE {@code SimHooks.stepTiming()} call. Three orders of magnitude above
   * the observed per-call cost, so it cannot fire on a slow-but-healthy machine, while still turning
   * a wedged robot loop into a bounded failure.
   */
  private static final long kStepCeilingMillis = 10_000;

  /** Wall-clock ceiling for {@code SimHooks.waitForProgramStart()}, which is likewise unbounded. */
  private static final long kProgramStartCeilingMillis = 60_000;

  /** Grace period after {@code endCompetition()} for a wedged step to unwind, before giving up. */
  private static final long kUnwedgeGraceMillis = 2_000;

  private static ExecutorService stepper;
  private static Robot robot;
  private static Thread robotThread;

  /** The throwable that killed the robot loop thread, if it died. */
  private static volatile Throwable robotLoopFailure;

  /** Non-null once the simulation clock is known unusable; every later step fails fast on it. */
  private static volatile String deadReason;

  /** Guards {@link #shutdown()} against a second pass; see its comment. */
  private static boolean shutdownComplete;

  private static Supplier<String> diagnostics = () -> "(no test-specific diagnostics registered)";
  private static long maxStepMillis;
  private static long stepCount;

  private SimRobotLoop() {
  }

  /* --------------------------------------------------------------------------- lifecycle */

  /**
   * Starts {@code robot.startCompetition()} on a named daemon thread WITH an uncaught-exception
   * handler, so a loop death is captured and reportable instead of vanishing into stderr.
   *
   * @return the started thread, so callers can keep their existing field
   */
  public static Thread start(Robot bootedRobot, String threadName) {
    reset();
    robot = bootedRobot;
    robotThread = new Thread(bootedRobot::startCompetition, threadName);
    robotThread.setDaemon(true);
    robotThread.setUncaughtExceptionHandler((t, e) -> robotLoopFailure = e);
    robotThread.start();
    return robotThread;
  }

  /** Bounded {@code SimHooks.waitForProgramStart()} -- it too waits on the robot loop. */
  public static void awaitProgramStart() {
    runBounded(SimHooks::waitForProgramStart, kProgramStartCeilingMillis,
        "SimHooks.waitForProgramStart()");
  }

  /** Registers a supplier of test-specific state, appended to every bounded-wait failure message. */
  public static void registerDiagnostics(Supplier<String> supplier) {
    diagnostics = supplier;
  }

  /**
   * Shuts the robot loop down and releases the simulation/odometry threads.
   *
   * <p>Ordering matters: the enabled-off settle step is SKIPPED when the clock is already dead, so
   * {@code endCompetition()} and {@code close()} still run. Those two are what stop the MapleSim
   * notifier and the Phoenix odometry thread -- non-daemon threads that would otherwise keep the
   * forked JVM alive after the test methods have finished, reproducing the very hang this class
   * exists to prevent, just one stage later.
   */
  public static void shutdown() {
    // Idempotent: a test that needs the robot stopped mid-method (to flush and read its wpilog, say)
    // calls this itself, and the @AfterAll/@AfterEach hook still calls it afterwards. A second
    // robot.close() would close the static drivetrain twice.
    if (shutdownComplete) {
      return;
    }
    shutdownComplete = true;

    AssertionError deferred = null;
    try {
      if (deadReason == null && robotThread != null && robotThread.isAlive()) {
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        stepQuietly(0.2);
      }
      if (robot != null) {
        robot.endCompetition();
      }
      if (robotThread != null) {
        robotThread.join(1000);
        if (robotThread.isAlive()) {
          deferred = new AssertionError(
              "startCompetition() loop should have exited within 1s of endCompetition()" + report());
        }
      }
      if (robot != null) {
        robot.close();
      }
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    } finally {
      SimHooks.resumeTiming();
      if (stepper != null) {
        stepper.shutdownNow();
        // Null it out rather than reusing it: a class that boots a Robot per @BeforeEach would
        // otherwise submit its next step to an already-shut-down executor and get a
        // RejectedExecutionException instead of a bounded step.
        stepper = null;
      }
    }

    // A loop that died from an exception leaves isAlive() == false, which the old teardown assertion
    // read as a clean exit. Surface it instead: a silent robot-loop death is never an expected way
    // for one of these tests to end.
    if (robotLoopFailure != null) {
      throw new AssertionError(
          "the robot's startCompetition() loop died with an uncaught exception" + report());
    }
    if (deferred != null) {
      throw deferred;
    }
  }

  /* ------------------------------------------------------------------------ bounded waits */

  /** Advances the paused simulation clock by {@code seconds}, bounded in WALL-CLOCK time. */
  public static void step(double seconds) {
    if (deadReason != null) {
      fail("the simulation clock is already dead (" + deadReason + ")" + report());
    }
    long t0 = System.nanoTime();
    runBounded(() -> SimHooks.stepTiming(seconds), kStepCeilingMillis,
        "SimHooks.stepTiming(" + seconds + ")");
    long millis = (System.nanoTime() - t0) / 1_000_000L;
    stepCount++;
    maxStepMillis = Math.max(maxStepMillis, millis);
  }

  /**
   * Steps until {@code condition} holds, bounded BOTH in simulated time ({@code timeoutSeconds}
   * worth of 20 ms ticks) and, per tick, in wall-clock time by {@link #step(double)}. On expiry it
   * fails with {@code what} plus the full state snapshot, never silently returns.
   */
  public static void stepUntil(BooleanSupplier condition, double timeoutSeconds, String what) {
    int maxTicks = (int) Math.ceil(timeoutSeconds / 0.02);
    for (int tick = 0; tick < maxTicks; tick++) {
      if (condition.getAsBoolean()) {
        return;
      }
      step(0.02);
    }
    if (!condition.getAsBoolean()) {
      fail("timed out after " + timeoutSeconds + "s of simulated time (" + maxTicks + " ticks): "
          + what + report());
    }
  }

  /** Bounded step used by teardown, where a further failure would only mask the real one. */
  private static void stepQuietly(double seconds) {
    try {
      step(seconds);
    } catch (AssertionError ignored) {
      // deadReason / robotLoopFailure already carry the diagnosis; shutdown must keep going.
    }
  }

  /* ---------------------------------------------------------------------------- internals */

  private static void runBounded(Runnable body, long ceilingMillis, String label) {
    if (stepper == null) {
      stepper = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "SimRobotLoop-stepper");
        // Expendable on purpose: a thread wedged inside a JNI wait can never be unwedged, and a
        // daemon thread does not stop the forked JVM from exiting once the tests are done.
        t.setDaemon(true);
        return t;
      });
    }
    Future<?> future = stepper.submit(body);
    try {
      future.get(ceilingMillis, TimeUnit.MILLISECONDS);
    } catch (TimeoutException e) {
      // The alarm this call is waiting on belongs to the robot loop. Stopping the loop's notifier is
      // the only lever the test thread still has; if it does not free the call, nothing will.
      if (robot != null) {
        try {
          robot.endCompetition();
        } catch (RuntimeException ignored) {
          // best effort only -- the failure below is the report that matters
        }
      }
      try {
        future.get(kUnwedgeGraceMillis, TimeUnit.MILLISECONDS);
      } catch (Exception ignored) {
        // still wedged; the daemon thread is abandoned deliberately
      }
      deadReason = label + " did not return within " + ceilingMillis + " ms";
      fail(deadReason + report());
    } catch (ExecutionException e) {
      fail(label + " threw " + e.getCause() + report(), e.getCause());
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      fail("interrupted while running " + label + report());
    }
  }

  private static void reset() {
    robot = null;
    robotThread = null;
    robotLoopFailure = null;
    deadReason = null;
    shutdownComplete = false;
    maxStepMillis = 0;
    stepCount = 0;
  }

  /** Largest wall-clock cost of a single step so far, in ms -- evidence for the ceiling above. */
  public static long maxObservedStepMillis() {
    return maxStepMillis;
  }

  /* -------------------------------------------------------------------------- diagnostics */

  /** The state snapshot appended to every bounded-wait failure. */
  private static String report() {
    StringBuilder b = new StringBuilder("\n  --- simulation loop state ---");
    b.append("\n  robot loop thread : ")
        .append(robotThread == null ? "never started"
            : robotThread.isAlive() ? "ALIVE" : "DEAD (startCompetition() returned or threw)");
    b.append("\n  sim clock         : ").append(stepCount).append(" steps, worst step ")
        .append(maxStepMillis).append(" ms wall")
        .append(deadReason == null ? "" : ", DEAD: " + deadReason);
    b.append("\n  DriverStation     : enabled=").append(DriverStationSim.getEnabled())
        .append(" autonomous=").append(DriverStationSim.getAutonomous())
        .append(" test=").append(DriverStationSim.getTest())
        .append(" dsAttached=").append(DriverStationSim.getDsAttached());
    b.append("\n  ").append(safeDiagnostics());
    if (robotLoopFailure != null) {
      b.append("\n  --- the robot loop thread died with ---\n").append(stackTrace(robotLoopFailure));
    }
    return b.toString();
  }

  private static String safeDiagnostics() {
    try {
      return diagnostics.get();
    } catch (RuntimeException e) {
      return "(diagnostics supplier threw " + e + ")";
    }
  }

  /**
   * The shared operator-control-board snapshot: everything a stuck operator test needs to explain
   * itself. Test classes pass this to {@link #registerDiagnostics(Supplier)}.
   */
  public static String operatorBoardState(int controlBoxPort, int buttonCount) {
    Superstructure superstructure = Superstructure.getInstance(null);
    Intake intake = Intake.getInstance();
    Shooter shooter = Shooter.getInstance();
    StringBuilder b = new StringBuilder("--- operator board state ---");
    b.append("\n  Superstructure    : system=").append(superstructure.getSystemState())
        .append(" owner=").append(name(CommandScheduler.getInstance().requiring(superstructure)))
        .append(" default=").append(name(defaultOf(superstructure)));
    b.append("\n  Intake            : roller=").append(intake.getRollerState())
        .append(" owner=").append(name(CommandScheduler.getInstance().requiring(intake)))
        .append(" default=").append(name(defaultOf(intake)));
    b.append("\n  Shooter           : indexer=").append(shooter.getIndexerState())
        .append(" agitator=").append(shooter.getAgitateState())
        .append(" targetRPM=").append(shooter.getTargetRPM());
    b.append("\n  control box ").append(controlBoxPort).append("    : ");
    for (int button = 1; button <= buttonCount; button++) {
      b.append(button).append('=')
          .append(DriverStation.getStickButton(controlBoxPort, button) ? "DOWN " : "up   ");
    }
    return b.toString();
  }

  private static Command defaultOf(Subsystem subsystem) {
    return CommandScheduler.getInstance().getDefaultCommand(subsystem);
  }

  private static String name(Command command) {
    return command == null ? "<none>" : command.getName();
  }

  private static String stackTrace(Throwable t) {
    StringWriter sw = new StringWriter();
    t.printStackTrace(new PrintWriter(sw));
    return sw.toString();
  }
}
