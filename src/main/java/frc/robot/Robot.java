// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.HubActiveState;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  /**
   * The RAW chooser selection that {@link #m_autonomousCommand} currently wraps, compared by
   * identity. See {@link #autonomousInit()} for why re-wrapping the same instance is fatal.
   */
  private Command m_wrappedAutoSelection;

  private final RobotContainer m_robotContainer;
  private final HubActiveState m_hubInstance = HubActiveState.getInstance();
  private final Vision m_vision;

  /**
   * Why the autonomous command's most recent end was attributed the way it was, logged as
   * {@code Auto/EndReason}. Distinct from {@code Auto/EndedInterrupted} (which only says
   * whether it was interrupted, not why) -- added so a wpilog can tell a driver-initiated mode
   * change apart from a test-mode cancel apart from (once one exists) a future recovery-layer
   * cancellation, none of which are otherwise distinguishable after the fact.
   */
  enum AutoEndReason {
    NATURAL_COMPLETION, TELEOP_INTERRUPTION, TEST_CANCELLATION, RECOVERY_CANCELLATION, UNKNOWN_INTERRUPTION
  }

  // Package-private (not private) so RobotAutoTerminationTelemetryTest -- same package -- can
  // assert teleopInit()/testInit() tag it correctly, without a reflection hack or a test-only
  // production method. Set immediately before each known cancellation call site; wrapAutonomous
  // ForTelemetry() reads it only when a wrapped command actually ends interrupted, then resets it,
  // so a later, unrelated interruption never inherits a stale reason.
  static AutoEndReason pendingCancelReason = AutoEndReason.UNKNOWN_INTERRUPTION;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Simulation runs intentionally have no physical controller attached. Keep WPILib from
    // emitting the repeated "Joystick Button ... not available" warning while preserving the
    // warning on real hardware, where a disconnected driver controller is actionable.
    if (Constants.currentMode != Constants.Mode.REAL) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter("media/sda1/logs"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT and to a local .wpilog for the verification loop
        Logger.addDataReceiver(new WPILOGWriter("logs"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_vision = Vision.getInstance(m_robotContainer.drivetrain);

  }
  

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_hubInstance.periodic();
    m_robotContainer.periodic();
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    CommandScheduler.getInstance().run();
    // After the scheduler, not before: this loop's PathPlanner setpoint (if any) has just been
    // produced, so the tracker pairs it with a same-instant pose read instead of a stale one.
    // See RobotContainer.trajectoryTrackerPeriodic().
    m_robotContainer.trajectoryTrackerPeriodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_vision.setIMUMode(1);
  }

  @Override
  public void disabledPeriodic() {
    m_vision.setIMUMode(1);

  }

  // This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
@Override
public void robotInit() {
}


  @Override
public void autonomousInit() {
    // Was previously left in disabledPeriodic's IMUMode(1) (EXTERNAL_SEED) for the whole match --
    // auto never switched to fusion mode, so MegaTag2 solves rotated field coordinates against a
    // stale IMU seed and produces odometry snaps the first time a tag comes into view.
    m_vision.setIMUMode(4);
    m_vision.setIMUAssistAlpha(0.001);
    Command selected = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (selected != null) {
      // The chooser hands back the SAME Command instance on every entry into autonomous, and
      // finallyDo() registers its receiver as composed. Wrapping that instance a second time
      // therefore throws IllegalArgumentException ("Commands that have been composed may not be
      // added to another composition or scheduled individually!") straight out of autonomousInit(),
      // out of loopFunc(), and out of startCompetition() -- killing the robot program for the rest
      // of the power cycle. That is reachable any time autonomous is entered twice without
      // restarting robot code (practice mode, pit testing), and it is what wedged
      // OperatorTeleopExitIsolationTest. Wrap once per DISTINCT selection instead, comparing by
      // identity so that changing the chooser still gets its own wrapper.
      if (selected != m_wrappedAutoSelection) {
        m_wrappedAutoSelection = selected;
        m_autonomousCommand = wrapAutonomousForTelemetry(selected);
      } else {
        // Same selection, new run: the wrapper is already built and already composed, so only the
        // per-run telemetry needs re-arming. wrapAutonomousForTelemetry() does exactly this at its
        // top on the first entry, which is what keeps first-entry behavior bit-identical.
        armAutonomousTelemetry();
      }
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    } else {
      Logger.recordOutput("Auto/Running", false);
      Logger.recordOutput("Auto/EndedInterrupted", false);
    }
  }

  /**
   * Marks the start of an autonomous run in telemetry: {@code Auto/Running} true, and
   * {@code Auto/EndedInterrupted} back to false so the previous run's ending cannot be misread as
   * this one's. Split out of {@link #wrapAutonomousForTelemetry(Command)} so a repeat entry with an
   * already-wrapped selection re-arms the same two entries without re-composing the Command.
   */
  private static void armAutonomousTelemetry() {
    Logger.recordOutput("Auto/Running", true);
    Logger.recordOutput("Auto/EndedInterrupted", false);
  }

  /**
   * Wraps the autonomous command so its completion is observable after the fact from a wpilog --
   * before this, nothing distinguished "auto finished on its own" from "auto got cut off" (a
   * teleopInit() cancel(), a disable-triggered scheduler cancel, or anything else), found during
   * the autonomous reliability audit. {@code Auto/Running} flips true right before scheduling and
   * false once the command ends; {@code Auto/EndedInterrupted} records which kind of ending it was.
   * Package-private and static so {@code RobotAutoTerminationTelemetryTest} can exercise both
   * outcomes directly without needing a real auto-chooser selection (PathPlanner's default chooser
   * selection is a trivial {@code Commands.none()} that finishes before there is anything to
   * observe -- see {@code AutoRegressionTestBase}'s own comment on why it bypasses the chooser too).
   */
  static Command wrapAutonomousForTelemetry(Command autoCommand) {
    armAutonomousTelemetry();
    return autoCommand.finallyDo(interrupted -> {
      Logger.recordOutput("Auto/Running", false);
      Logger.recordOutput("Auto/EndedInterrupted", interrupted);
      AutoEndReason reason = interrupted ? pendingCancelReason : AutoEndReason.NATURAL_COMPLETION;
      Logger.recordOutput("Auto/EndReason", reason.name());
      pendingCancelReason = AutoEndReason.UNKNOWN_INTERRUPTION;
    });
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    pendingCancelReason = AutoEndReason.TELEOP_INTERRUPTION;
    m_vision.setIMUMode(4);
    m_vision.setIMUAssistAlpha(0.001);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_vision.getPoseResetEstimate().ifPresent(m_robotContainer.drivetrain::resetPose);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /**
   * Runs on every exit from teleop, before the next mode's init. Used to neutralize latched
   * operator intent -- see {@link RobotContainer#teleopExit()} for why the Superstructure default
   * command's own interruption cleanup does not cover this transition on its own.
   */
  @Override
  public void teleopExit() {
    m_robotContainer.teleopExit();
  }

  @Override
  public void testInit() {
    pendingCancelReason = AutoEndReason.TEST_CANCELLATION;
    m_vision.setIMUMode(4);
    m_vision.setIMUAssistAlpha(0.001);
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /**
   * Advances the maple-sim physics world, once per robot loop, in simulation only.
   *
   * <p>This is the location maple-sim's {@code SimulatedArena.simulationPeriodic()} javadoc
   * mandates: <em>"This method should be called ONCE in {@code TimedRobot#simulationPeriodic()}"</em>.
   * The library ships no thread of its own, so whoever calls it owns the dyn4j world. Driving it
   * here -- after {@code robotPeriodic()} has run the CommandScheduler and applied this loop's
   * motor outputs -- means mechanism simulations that mutate the world (notably
   * {@code IntakeSimulation.startIntake()/stopIntake()}, which add and remove a physics fixture)
   * run on the SAME thread as the step that walks it, so they cannot race it.
   *
   * <p>{@code IterativeRobotBase} only calls this in simulation, so real-robot behavior is
   * untouched.
   */
  @Override
  public void simulationPeriodic() {
    m_robotContainer.drivetrain.updateSimulation();
  }

  /** Stops simulation-owned background resources before AdvantageKit and HAL shutdown. */
  @Override
  public void close() {
    if (m_robotContainer != null) {
      m_robotContainer.close();
    }
    super.close();
  }
}
