// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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

  private IntakeSimulation intakeSimulation;
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
      // maple-sim mutates the shared dyn4j world from TWO threads here: this one (the robot loop,
      // via Intake.periodic()) and the 5 ms sim Notifier that runs SimulatedArena.simulationPeriodic().
      // Every world-mutating entry point the library owns -- simulationPeriodic(), addGamePiece(),
      // removeGamePiece(), addIntakeSimulation(), addDriveTrainSimulation() -- is `synchronized` on
      // the arena instance. That monitor IS the library's threading contract.
      //
      // IntakeSimulation.startIntake()/stopIntake() do NOT take it for us, and they are not
      // incidental: IntakeSimulation extends dyn4j's BodyFixture, so those two calls add and remove
      // a fixture on the drivetrain's physics body, mutating the broadphase and contact structures
      // the Notifier thread is walking. Unguarded, that threw ConcurrentModificationException from
      // whichever side happened to be iterating -- either out of Intake.periodic() (killing the
      // robot program for the rest of the session) or out of the Notifier (killing the physics
      // thread, so the world silently stopped advancing while the robot code kept running and
      // reporting). Measured at 2 of 6 runs of OperatorBoxMaintainedSwitchLifecycleTest.
      //
      // Taking the same monitor closes both directions. The cost is bounded: the robot thread waits
      // at most one physics step, and simulationPeriodic() runs a single sub-tick per 5 ms period
      // (MapleSimSwerveDrivetrain's overrideSimulationTimings(simPeriod, 1)).
      synchronized (SimulatedArena.getInstance()) {
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
      }
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
    // Called from a JUnit test thread, i.e. a THIRD thread reaching into the same world. Same
    // monitor, same reason as updateInputs() above; addGamePieceToIntake() mutates the piece list
    // the physics step walks.
    synchronized (SimulatedArena.getInstance()) {
      return intakeSimulation.addGamePieceToIntake();
    }
  }
}
