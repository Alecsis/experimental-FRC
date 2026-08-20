// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.plant.LinearSystemId;
import org.wpilib.units.measure.Distance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * IntakeIO implementation backed by WPILib physics simulation. No vendor hardware
 * dependencies -- suitable for desktop/sim-only testing and replay.
 */
public class IntakeIOSim implements IntakeIO {
  // Confirmed real hardware: both pivot and roller are Kraken X60 (TalonFX), full CTRE drivetrain.
  private static final DCMotor pivotGearbox = DCMotor.getKrakenX60(1);
  private static final DCMotor rollerGearbox = DCMotor.getKrakenX60(1);

  private static final double kSimLoopPeriodSecs = 0.02;
  private static final double kStartingAngleDeg = -10.0; // matches Intake.PivotState.DOWN

  private static final double kPivotKP = 70.0;
  private static final double kPivotKD = 0.0;
  private static final double kRollerKP = 0.03;

  private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
      pivotGearbox,
      Constants.kIntakePivotReduction,
      Constants.kIntakePivotMassKg
          * Constants.kIntakePivotLengthMeters
          * Constants.kIntakePivotLengthMeters
          / 3.0,
      Constants.kIntakePivotLengthMeters,
      Degrees.of(Constants.kIntakePivotMinAngleDeg).in(Radians),
      Degrees.of(Constants.kIntakePivotMaxAngleDeg).in(Radians),
      true,
      Degrees.of(kStartingAngleDeg).in(Radians));

  private final DCMotorSim rollerSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          rollerGearbox, Constants.kIntakeRollerMOI, Constants.kIntakeRollerReduction),
      rollerGearbox);

  private double pivotAppliedVolts = 0.0;
  private boolean pivotClosedLoop = false;
  private double pivotPositionSetpointRads = 0.0;

  private double rollerAppliedVolts = 0.0;
  private boolean rollerClosedLoop = false;
  private double rollerVelocitySetpointRadPerSec = 0.0;

  /**
   * Volatile because it is the one field of this class read from a thread other than the robot
   * loop: {@link #addGamePieceForTest()} runs on a JUnit thread. The robot loop constructs this
   * lazily in {@link #updateInputs}, which no longer holds the arena monitor, so the monitor
   * {@code addGamePieceForTest()} still takes no longer supplies a happens-before edge back to that
   * construction. {@code volatile} supplies it directly. Not a performance concern -- one reference
   * read per 20 ms tick.
   */
  private volatile IntakeSimulation intakeSimulation;

  /** Robot-loop-only: written by setRoller*(), read by updateInputs(), both on the robot thread. */
  private boolean intakeRunning = false;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      pivotAppliedVolts = 0.0;
      rollerAppliedVolts = 0.0;
    } else {
      if (pivotClosedLoop) {
        pivotAppliedVolts = MathUtil.clamp(
            (pivotPositionSetpointRads - pivotSim.getAngleRads()) * kPivotKP
                + (0.0 - pivotSim.getVelocityRadPerSec()) * kPivotKD,
            -12.0, 12.0);
      }
      if (rollerClosedLoop) {
        rollerAppliedVolts = MathUtil.clamp(
            (rollerVelocitySetpointRadPerSec - rollerSim.getAngularVelocityRadPerSec()) * kRollerKP,
            -12.0, 12.0);
      }
    }

    pivotSim.setInputVoltage(pivotAppliedVolts);
    pivotSim.update(kSimLoopPeriodSecs);

    rollerSim.setInputVoltage(rollerAppliedVolts);
    rollerSim.update(kSimLoopPeriodSecs);

    inputs.pivotConnected = true;
    inputs.pivotPositionRads = pivotSim.getAngleRads();
    inputs.pivotVelocityRadsPerSec = pivotSim.getVelocityRadPerSec();
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotStatorCurrentAmps = pivotSim.getCurrentDrawAmps();
    inputs.pivotSupplyCurrentAmps = pivotSim.getCurrentDrawAmps();
    inputs.pivotTempCelsius = 0.0;

    inputs.rollerConnected = true;
    inputs.rollerPositionRads = rollerSim.getAngularPositionRad();
    inputs.rollerVelocityRadsPerSec = rollerSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerStatorCurrentAmps = rollerSim.getCurrentDrawAmps();
    inputs.rollerSupplyCurrentAmps = rollerSim.getCurrentDrawAmps();
    inputs.rollerTempCelsius = 0.0;

    // The drive sim reference is read BEFORE any SimulatedArena.getInstance() call on purpose:
    // getInstance() CONSTRUCTS the default arena if none has been installed yet, and the default is
    // exactly the one whose hub/ramp collider MapleSimSwerveDrivetrain deliberately overrides away.
    // A non-null drive sim proves that override already ran, so reaching for the arena is safe.
    AbstractDriveTrainSimulation driveSim =
        intakeSimulation == null ? RobotContainer.drivetrain.getMapleSimDrive() : null;

    if (intakeSimulation != null || driveSim != null) {
      // These calls mutate the shared dyn4j world -- IntakeSimulation extends dyn4j's BodyFixture,
      // so startIntake()/stopIntake() add and remove a fixture on the drivetrain's physics body,
      // touching the broadphase and contact structures a physics step walks. They used to be
      // wrapped in `synchronized (SimulatedArena.getInstance())`, because the arena was stepped by
      // a separate 5 ms Notifier and the two threads genuinely raced: unguarded, that threw
      // ConcurrentModificationException out of whichever side happened to be iterating, measured at
      // 2 of 6 runs of OperatorBoxMaintainedSwitchLifecycleTest.
      //
      // The monitor is gone because the second thread is gone. This method has exactly one call
      // chain -- Intake.periodic() (SubsystemBase) <- CommandScheduler.run() <- Robot.robotPeriodic()
      // -- and the arena is stepped from Robot.simulationPeriodic(), later in that same loopFunc()
      // pass. Both therefore run on the robot competition thread, sequentially, and no lock can
      // order a thread against itself. Nothing else in the repository steps or mutates the arena
      // while it is running; MapleSimIntakeToggleRaceTest pins the stepping thread's identity.
      //
      // REAL hardware is unaffected either way: IntakeIOReal is a different IntakeIO implementation
      // and this class is never constructed on a roboRIO.
      if (intakeSimulation == null) {
        Distance width = Meters.of(Constants.kIntakeSimWidthMeters);
        intakeSimulation = IntakeSimulation.InTheFrameIntake(
            "Fuel", driveSim, width, IntakeSimulation.IntakeSide.FRONT, Constants.kIntakeSimCapacity);
      }

      if (intakeRunning && !intakeSimulation.isRunning()) {
        intakeSimulation.startIntake();
      } else if (!intakeRunning && intakeSimulation.isRunning()) {
        intakeSimulation.stopIntake();
      }

      inputs.hasGamePiece = intakeSimulation.getGamePiecesAmount() > 0;
    } else {
      inputs.hasGamePiece = false;
    }
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotClosedLoop = false;
    pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setPivotPosition(double positionRads) {
    pivotClosedLoop = true;
    pivotPositionSetpointRads = positionRads;
  }

  @Override
  public void setPivotEncoderPosition(double positionRads) {
    // No-op: the physics sim already tracks the arm's true position directly, so there is
    // no separate encoder reference to re-zero the way there is on real hardware.
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerClosedLoop = false;
    rollerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    intakeRunning = false;
  }

  @Override
  public void setRollerVelocity(double velocityRadPerSec) {
    rollerClosedLoop = true;
    rollerVelocitySetpointRadPerSec = velocityRadPerSec;
    intakeRunning = velocityRadPerSec > 0;
  }

  /** Test-only: adds one game piece directly to the lazily-constructed IntakeSimulation,
   *  bypassing field/collision physics. Returns false if IntakeSimulation hasn't been constructed
   *  yet (RobotContainer.drivetrain's MapleSim drive wasn't available on any prior
   *  updateInputs() tick). Package-private -- only for JUnit tests in this package. */
  boolean addGamePieceForTest() {
    if (intakeSimulation == null) {
      return false;
    }
    // DELIBERATELY RETAINED after the updateInputs() monitor above was dropped, and not by
    // oversight -- the two are not the same case. updateInputs() runs on the robot competition
    // thread, the same thread that steps the arena, so its lock could not order anything. This
    // method is the one world mutation in the repository that is BY CONSTRUCTION off that thread:
    // it is a test hook, invoked from a JUnit thread.
    //
    // What makes it safe today is the caller, not this lock: IntakeGamePieceSignalTest calls it
    // between SimHooks.stepTiming() calls, i.e. with the clock paused and the robot loop parked in
    // waitForNotifierAlarm, so no physics step is in flight (the same paused-mutation rule
    // CLAUDE.md states, and that PathDisturbanceSimTestBase's resetPose() seed relies on).
    // Nothing enforces that rule, though: a future test that called this after resumeTiming()
    // would race the physics step, and this monitor -- the same one
    // SimulatedArena.simulationPeriodic() is declared `synchronized` on -- is the only thing that
    // would still serialize it. It costs one uncontended acquire per test, on no production path.
    synchronized (SimulatedArena.getInstance()) {
      return intakeSimulation.addGamePieceToIntake();
    }
  }
}
