// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import frc.robot.subsystems.intake.Intake;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/**
 * The maple-sim / dyn4j world must have exactly ONE owning thread, and toggling the intake must not
 * be able to corrupt it.
 *
 * <p><b>The defect this pins down.</b> {@code IntakeSimulation extends dyn4j's BodyFixture}, so
 * {@code startIntake()}/{@code stopIntake()} literally add and remove a fixture on the drivetrain's
 * physics body. Those run from {@code Intake.periodic()} on the robot loop. The arena used to be
 * stepped by a separate 5 ms {@code Notifier}, so the physics step walked the broadphase and contact
 * structures while the robot loop mutated them, throwing {@link
 * java.util.ConcurrentModificationException} out of whichever side happened to be iterating --
 * measured at 3 of 7 runs of {@code OperatorBoxMaintainedSwitchLifecycleTest}. Losing that race on
 * the robot thread kills the robot program; losing it on the Notifier kills the physics thread, and
 * the world then silently stops advancing while the robot code keeps running and logging. The second
 * is the dangerous one, which is why this test asserts the world is still MOVING, not merely that
 * nothing threw.
 *
 * <p><b>How ownership is proven.</b> Not by inspection: a {@link SimulatedArena.Simulatable} is
 * registered with the arena, and maple-sim invokes it from inside {@code simulationSubTick} -- i.e.
 * on whichever thread is actually stepping physics. Recording {@code Thread.currentThread()} there
 * and asserting it is the robot competition thread is direct evidence that stepping and mechanism
 * mutation share one thread. The same callback counts sub-ticks, which is what detects a dead or
 * stalled stepper.
 *
 * <p>Toggling is driven through the real control board (button 6, the maintained intake switch)
 * rather than by poking {@code Intake} directly, so the transitions travel the production path:
 * board -> operator policy -> Superstructure -> {@code Intake.setRoller} -> {@code IntakeIOSim} ->
 * {@code IntakeSimulation}. Ticks are deterministic; there are no sleeps, and every step is
 * wall-clock bounded by {@link SimRobotLoop}.
 */
class MapleSimIntakeToggleRaceTest {
  private static final int kControlBoxPort = 1;
  private static final int kSimMirrorPort = 3;
  private static final int kIntakeToggle = 6;

  /** Board states to drive. Each transition must actually reach the roller to count. */
  private static final int kTargetTransitions = 200;
  /** Scheduler ticks held per board state: policy executes on one tick, periodic applies the next. */
  private static final int kTicksPerState = 3;

  private static Robot robot;
  private static Thread robotThread;
  private static Intake intake;

  /** Name of the thread maple-sim actually steps physics on, captured from inside the step. */
  private static final AtomicReference<String> steppingThreadName = new AtomicReference<>();
  /** Sub-ticks maple-sim has executed. Frozen counter == dead or stalled physics. */
  private static final AtomicInteger subTicks = new AtomicInteger();

  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    DriverStationSim.setJoystickButtonCount(kControlBoxPort, 12);
    DriverStationSim.setJoystickButtonCount(kSimMirrorPort, 12);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    robot = new Robot();
    robotThread = SimRobotLoop.start(robot, "MapleSimIntakeToggleRaceTest-competition");
    SimRobotLoop.awaitProgramStart();

    intake = Intake.getInstance();
    SimRobotLoop.registerDiagnostics(
        () -> SimRobotLoop.operatorBoardState(kControlBoxPort, 6)
            + "\n  physics          : subTicks=" + subTicks.get()
            + " steppingThread=" + steppingThreadName.get());

    // Runs inside SimulatedArena.simulationSubTick(), on the stepping thread, with the arena
    // monitor held. addCustomSimulation() is itself synchronized, so registering here is safe.
    SimulatedArena.getInstance().addCustomSimulation(subTickNum -> {
      steppingThreadName.set(Thread.currentThread().getName());
      subTicks.incrementAndGet();
    });
  }

  @AfterAll
  static void teardown() {
    setToggle(false);
    SimRobotLoop.shutdown();
  }

  /* ------------------------------------------------------------------------------- helpers */

  private static void step(double seconds) {
    SimRobotLoop.step(seconds);
  }

  private static void setToggle(boolean pressed) {
    DriverStationSim.setJoystickButton(kControlBoxPort, kIntakeToggle, pressed);
    DriverStationSim.notifyNewData();
  }

  private static void enableTeleop() {
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(0.1);
  }

  /* ---------------------------------------------------------------------------------- test */

  @Test
  @Timeout(180)
  void togglingTheIntakeAgainstALiveWorldNeitherCorruptsNorStopsIt() {
    setToggle(false);
    enableTeleop();

    // Let teleop's intake.homing() finish so it is not holding the pivot while we hammer the roller.
    SimRobotLoop.stepUntil(
        () -> org.wpilib.command2.CommandScheduler.getInstance().requiring(intake) == null,
        4.0, "intake.homing() should complete and release Intake before the stress loop");

    assertTrue(subTicks.get() > 0,
        "precondition: maple-sim must actually be stepping before the stress loop -- a zero "
            + "sub-tick count here would make every later assertion vacuous");
    String stepper = steppingThreadName.get();
    assertEquals(robotThread.getName(), stepper,
        "SINGLE-THREAD OWNERSHIP: maple-sim must step the dyn4j world on the robot competition "
            + "thread, the same thread Intake.periodic() mutates it from. A different name here "
            + "means the arena is being stepped off-thread again and the race is back");

    // ---- stress: alternate the maintained intake switch against a live physics world.
    int observedRollerTransitions = 0;
    Intake.Roller previousRoller = intake.getRollerState();
    int subTicksAtLoopStart = subTicks.get();

    for (int i = 0; i < kTargetTransitions; i++) {
      setToggle(i % 2 == 0);
      for (int tick = 0; tick < kTicksPerState; tick++) {
        step(0.02);
        Intake.Roller now = intake.getRollerState();
        if (now != previousRoller) {
          observedRollerTransitions++;
          previousRoller = now;
        }
      }
      assertTrue(robotThread.isAlive(),
          "the robot loop died during intake toggling -- see the captured stack trace. This is the "
              + "robot-thread side of the dyn4j race (removeFixture while the step walks contacts)");
    }

    // ---- the toggling was real, not a no-op loop.
    assertTrue(observedRollerTransitions >= kTargetTransitions / 2,
        "the stress loop must actually have driven the roller back and forth; only "
            + observedRollerTransitions + " roller transitions were observed across "
            + kTargetTransitions + " board toggles, so this run proved nothing about the race");

    // ---- the world is still advancing. A dead stepper freezes this counter while the robot loop
    // keeps running, which is exactly the silent failure mode the old Notifier had.
    int subTicksDuringLoop = subTicks.get() - subTicksAtLoopStart;
    assertTrue(subTicksDuringLoop >= kTargetTransitions * kTicksPerState,
        "maple-sim must have kept stepping throughout: expected at least one sub-tick per "
            + "scheduler tick but saw " + subTicksDuringLoop + " over "
            + (kTargetTransitions * kTicksPerState) + " ticks. A frozen counter means the physics "
            + "world stopped advancing -- silent in every other assertion");

    // ---- and it never migrated off the robot loop mid-run.
    assertEquals(robotThread.getName(), steppingThreadName.get(),
        "the stepping thread must still be the robot competition thread at the end of the run");

    assertTrue(robotThread.isAlive(), "the robot loop must still be running after the stress loop");
    setToggle(false);
    step(0.1);
  }
}
