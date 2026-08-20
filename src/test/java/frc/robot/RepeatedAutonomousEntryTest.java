// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.datalog.DataLogReader;
import org.wpilib.datalog.DataLogRecord;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.Timeout;

/**
 * Entering autonomous MORE THAN ONCE in a single robot program run must not kill the robot program.
 *
 * <p>The defect this pins down: {@code RobotContainer.getAutonomousCommand()} is
 * {@code autoChooser.getSelected()}, which hands back the SAME {@code Command} instance on every
 * call, and {@code Command.finallyDo()} registers its receiver as composed.
 * {@code autonomousInit()} used to wrap the selection on every entry, so the second entry threw
 *
 * <pre>IllegalArgumentException: Commands that have been composed may not be added to another
 * composition or scheduled individually!</pre>
 *
 * out of {@code autonomousInit()} -&gt; {@code loopFunc()} -&gt; {@code startCompetition()}, ending
 * the robot loop thread for the rest of the power cycle. On a real robot that is reachable any time
 * autonomous is run twice without restarting robot code (practice mode, pit testing). In these
 * tests it also wedged the harness outright: with the loop thread gone nothing services the 20 ms
 * notifier alarm, so {@code SimHooks.stepTiming()} blocks in native code forever -- see
 * {@link SimRobotLoop}, whose wall-clock bound is what turns that into a reported failure instead.
 *
 * <p>The fix caches the raw selection by identity and re-wraps only when the selection changes.
 *
 * <p><b>How "the auto scheduled again" is observed.</b> Through {@code CommandScheduler}'s
 * initialize/finish hooks, filtered to commands with NO requirements. The chooser's default
 * selection is a trivial {@code Commands.none()}, and its telemetry wrapper inherits that empty
 * requirement set, so within these autonomous windows the wrapper is the only requirement-free
 * command scheduled -- everything else running (the operator policy, the drivetrain default, intake
 * homing) owns a subsystem. Capturing the INSTANCE, not just a count, is what makes this test prove
 * the fix rather than merely the absence of a crash: the same wrapper object must be re-scheduled,
 * which is exactly "cached, not re-wrapped".
 *
 * <p><b>Why the wpilog cannot count the entries.</b> AdvantageKit records one value per entry per
 * cycle, and {@code Commands.none()} is armed ({@code Auto/Running} true) and finished
 * ({@code Auto/Running} false) inside the SAME robot cycle -- {@code autonomousInit()} and the
 * {@code CommandScheduler.run()} that ends the command both happen in one {@code loopFunc()}. Only
 * the cycle's final value is kept, and it is identical for both entries, so write-on-change
 * collapses the series to a single sample. That was measured against this test's own log, not
 * assumed. The log is therefore used for what it CAN prove: that the wrapped path ran at all rather
 * than {@code autonomousInit()}'s null-selection branch, and that no run's ending was misattributed.
 *
 * <p>ONE Robot per JVM, per build.gradle's {@code forkEvery = 1} note and
 * {@code RobotContainer.close()}'s static-drivetrain warning. The methods are ordered because the
 * second one writes the very telemetry entries the first one reads out of the wpilog, so it must
 * not run until that read is done.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class RepeatedAutonomousEntryTest {
  private static final String kEndReasonEntryName = "/RealOutputs/Auto/EndReason";
  private static final String kEndedInterruptedEntryName = "/RealOutputs/Auto/EndedInterrupted";
  private static final String kBooleanType = "boolean";
  private static final String kStringType = "string";

  private static Robot robot;
  private static Thread robotThread;

  /**
   * Every requirement-free command the scheduler initialized/finished, in order. Written from the
   * ROBOT thread by the scheduler hooks and read from the test thread, hence synchronized.
   */
  private static final List<Command> autoWrapperInits =
      Collections.synchronizedList(new ArrayList<>());
  private static final List<Command> autoWrapperFinishes =
      Collections.synchronizedList(new ArrayList<>());

  @BeforeAll
  static void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    robot = new Robot();
    robotThread = SimRobotLoop.start(robot, "RepeatedAutonomousEntryTest-competition");
    SimRobotLoop.awaitProgramStart();

    // Registered while the clock is paused and the robot is disabled, per this repo's rule about
    // not mutating scheduler state from a test thread while the real-time loop is running.
    CommandScheduler.getInstance().onCommandInitialize(command -> {
      if (command.getRequirements().isEmpty()) {
        autoWrapperInits.add(command);
      }
    });
    CommandScheduler.getInstance().onCommandFinish(command -> {
      if (command.getRequirements().isEmpty()) {
        autoWrapperFinishes.add(command);
      }
    });
  }

  @AfterAll
  static void tearDown() {
    SimRobotLoop.shutdown();
  }

  /* ------------------------------------------------------------------------------- helpers */

  private static void step(double seconds) {
    SimRobotLoop.step(seconds);
  }

  private static void enterAutonomous() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(0.1);
  }

  private static void enterTeleop() {
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(0.1);
  }

  private static void disable() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    step(0.1);
  }

  private static String hookState() {
    return "\n  requirement-free initializes: " + describe(autoWrapperInits)
        + "\n  requirement-free finishes   : " + describe(autoWrapperFinishes);
  }

  private static String describe(List<Command> commands) {
    synchronized (commands) {
      StringBuilder text = new StringBuilder("[");
      for (int i = 0; i < commands.size(); i++) {
        Command command = commands.get(i);
        text.append(i == 0 ? "" : ", ").append(command.getName()).append('@')
            .append(Integer.toHexString(System.identityHashCode(command)));
      }
      return text.append(']').toString();
    }
  }

  /* ---------------------------------------------------------------------------------- tests */

  @Test
  @Order(1)
  @Timeout(90)
  void autonomousCanBeEnteredTwiceWithTheSameChooserSelection() throws IOException {
    assertTrue(robotThread.isAlive(),
        "precondition: the robot loop is running after waitForProgramStart()");
    assertEquals(0, autoWrapperInits.size(),
        "precondition: no autonomous command has been scheduled yet" + hookState());

    // ---- autonomous entry #1: the wrapper is built, scheduled, and runs.
    enterAutonomous();
    assertTrue(robotThread.isAlive(),
        "the first autonomous entry must not disturb the robot loop");
    assertEquals(1, autoWrapperInits.size(),
        "the first autonomous entry must schedule exactly one wrapped auto" + hookState());
    assertEquals(1, autoWrapperFinishes.size(),
        "and it must run to completion, which is what fires the wrapper's telemetry finallyDo"
            + hookState());
    Command firstRunWrapper = autoWrapperInits.get(0);

    // ---- the mode transitions a real match makes between two autonomous runs.
    enterTeleop();
    assertTrue(robotThread.isAlive(), "teleop must not disturb the robot loop");
    disable();
    assertTrue(robotThread.isAlive(), "disabling must not disturb the robot loop");
    assertEquals(1, autoWrapperInits.size(),
        "neither teleop nor disable may schedule an autonomous command" + hookState());

    // ---- autonomous entry #2, SAME chooser selection. This is the regression.
    //
    // Nothing here can catch the exception: it was thrown on the ROBOT thread, not this one, so the
    // only way to observe it is that the loop stops existing. SimRobotLoop captures the throwable
    // from that thread and prints it in the failure message, and its bounded step is what stops a
    // dead loop from wedging stepTiming() in native code instead of failing.
    enterAutonomous();
    assertTrue(robotThread.isAlive(),
        "a SECOND autonomous entry with the same chooser selection must not kill the robot loop -- "
            + "re-wrapping an already-composed Command throws IllegalArgumentException straight "
            + "out of autonomousInit(), which ends startCompetition() for the whole power cycle");

    assertEquals(2, autoWrapperInits.size(),
        "the second autonomous entry must schedule the auto again -- surviving the entry is not "
            + "enough, the auto has to actually run" + hookState());
    assertSame(firstRunWrapper, autoWrapperInits.get(1),
        "and it must be the SAME wrapper instance, re-scheduled. A different instance would mean "
            + "autonomousInit() built a second wrapper around the same already-composed selection, "
            + "which is precisely what throws" + hookState());
    assertEquals(2, autoWrapperFinishes.size(),
        "the reused wrapper must run to completion again, so its telemetry finallyDo fires once "
            + "per autonomous run rather than only on the first" + hookState());

    // Leave autonomous, then stop the robot so AdvantageKit flushes the wpilog read below.
    // tearDown()'s shutdown() is idempotent.
    disable();
    SimRobotLoop.shutdown();

    // ---- telemetry: the wrapped path really ran, and no run's ending was misattributed.
    Path wpilog = findNewestWpilog();
    assertTrue(wpilog != null, "no .wpilog under logs/ after the run -- SIM mode should write one");

    List<String> endReason = readStringSeries(wpilog, kEndReasonEntryName);
    assertTrue(endReason.contains("NATURAL_COMPLETION"),
        "Auto/EndReason is written only by the telemetry wrapper's finallyDo, so its presence is "
            + "what separates 'the wrapped auto ran' from autonomousInit()'s null-selection branch, "
            + "which would make every assertion above vacuous. Series: " + endReason);
    assertTrue(endReason.stream().allMatch("NATURAL_COMPLETION"::equals),
        "both runs ended on their own, so no other end reason may appear -- a stale "
            + "pendingCancelReason leaking across the re-arm would show up right here. Series: "
            + endReason);

    List<Boolean> endedInterrupted = readBooleanSeries(wpilog, kEndedInterruptedEntryName);
    assertTrue(!endedInterrupted.contains(Boolean.TRUE),
        "neither autonomous run was cut off, so Auto/EndedInterrupted must never have gone true -- "
            + "a true here would mean re-arming misattributed one run's ending to the other. "
            + "Series: " + endedInterrupted);
  }

  /**
   * The rule the identity cache exists to respect, pinned down directly.
   *
   * <p>The end-to-end half above cannot vary the chooser selection: {@code autoChooser} is private
   * to {@code RobotContainer} with no test hook, and driving it through its NetworkTables
   * {@code selected} topic would make the assertion depend on real-time listener propagation inside
   * a paused-clock test. So the "a different selection gets its own wrapper" half is asserted
   * against WPILib's composition rule itself, which is the entire reason {@code autonomousInit()}
   * compares by identity rather than wrapping once and caching forever: two distinct selections each
   * wrap cleanly, and re-wrapping ONE selection is what throws.
   */
  @Test
  @Order(2)
  @Timeout(30)
  void distinctSelectionsEachWrapCleanlyAndOnlyRewrappingOneThrows() {
    Command selectionA = Commands.none();
    Command selectionB = Commands.none();

    Command wrappedA = Robot.wrapAutonomousForTelemetry(selectionA);
    Command wrappedB = Robot.wrapAutonomousForTelemetry(selectionB);
    assertNotSame(wrappedA, wrappedB,
        "a different selection must get its own wrapper, never the previous selection's");

    assertThrows(IllegalArgumentException.class,
        () -> Robot.wrapAutonomousForTelemetry(selectionA),
        "re-wrapping an already-wrapped selection is what used to happen on every repeat "
            + "autonomous entry, and it throws -- the cache in autonomousInit() exists precisely "
            + "because this is not survivable on the robot thread");
  }

  /* -------------------------------------------------------------------------- wpilog reading */

  /**
   * Reads every sample of one boolean AdvantageKit entry, via WPILib's own
   * {@link DataLogReader}/{@link DataLogRecord} -- the same approach
   * {@code RobotAutoTerminationTelemetryTest} and {@code WpilogTrajectoryErrorReader} use.
   */
  private static List<Boolean> readBooleanSeries(Path wpilogFile, String entryName)
      throws IOException {
    List<Boolean> samples = new ArrayList<>();
    forEachSampleOf(wpilogFile, entryName, kBooleanType, record -> samples.add(record.getBoolean()));
    return samples;
  }

  /** Same shape as {@link #readBooleanSeries}, for a string-typed entry (Auto/EndReason). */
  private static List<String> readStringSeries(Path wpilogFile, String entryName)
      throws IOException {
    List<String> samples = new ArrayList<>();
    forEachSampleOf(wpilogFile, entryName, kStringType, record -> samples.add(record.getString()));
    return samples;
  }

  private static void forEachSampleOf(Path wpilogFile, String entryName, String entryType,
      Consumer<DataLogRecord> sink) throws IOException {
    DataLogReader reader = new DataLogReader(wpilogFile.toString());
    if (!reader.isValid()) {
      throw new IOException("Not a valid WPILOG file: " + wpilogFile);
    }

    Map<Integer, String> activeNames = new HashMap<>();
    Map<Integer, String> activeTypes = new HashMap<>();

    for (DataLogRecord record : reader) {
      if (record.isStart()) {
        DataLogRecord.StartRecordData start = record.getStartData();
        activeNames.put(start.entry, start.name);
        activeTypes.put(start.entry, start.type);
        continue;
      }
      if (record.isFinish() || record.isSetMetadata()) {
        continue;
      }
      if (entryName.equals(activeNames.get(record.getEntry()))
          && entryType.equals(activeTypes.get(record.getEntry()))) {
        sink.accept(record);
      }
    }
  }

  /** Newest-by-mtime, matching AutoRegressionTestBase's own findNewestWpilog() convention. */
  private static Path findNewestWpilog() throws IOException {
    Path logsDir = Paths.get("logs");
    if (!Files.isDirectory(logsDir)) {
      return null;
    }
    try (Stream<Path> entries = Files.list(logsDir)) {
      return entries
          .filter(path -> path.toString().endsWith(".wpilog"))
          .max(Comparator.comparingLong(path -> path.toFile().lastModified()))
          .orElse(null);
    }
  }
}
