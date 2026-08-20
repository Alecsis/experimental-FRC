// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility.simulation;

import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Voltage;
import org.wpilib.framework.RobotBase;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Injects maple-sim rigid-body physics into a CTRE swerve drivetrain, replacing CTRE's
 * physics-free SimSwerveDrivetrain. Structurally adapted from Team 254's public 2025 codebase
 * (temp_reference/Team 254 Code) for this project's TalonFX + CANcoder + Pigeon2 swerve setup.
 */
public class MapleSimSwerveDrivetrain {
  private final Pigeon2SimState pigeonSim;
  public final SwerveDriveSimulation mapleSimDrive;

  /**
   * @param simPeriod the OUTER stepping period -- how much simulated time one {@link #update()}
   *     advances. This is the robot loop period, because {@code update()} is driven from
   *     {@code Robot.simulationPeriodic()} rather than from a background Notifier.
   * @param simSubTicksPerPeriod how many physics integration steps maple-sim takes inside that
   *     window. {@code simPeriod / simSubTicksPerPeriod} is the integration timestep, which must
   *     stay small for the simulated steer gains to remain numerically stable.
   */
  public MapleSimSwerveDrivetrain(
      Time simPeriod,
      int simSubTicksPerPeriod,
      Mass robotMassWithBumpers,
      Distance bumperLengthX,
      Distance bumperWidthY,
      DCMotor driveMotorModel,
      DCMotor steerMotorModel,
      double wheelCOF,
      Translation2d[] moduleLocations,
      Pigeon2 pigeon,
      SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
      SwerveModuleConstants<?, ?, ?>... moduleConstants) {
    this.pigeonSim = pigeon.getSimState();

    DriveTrainSimulationConfig simulationConfig = DriveTrainSimulationConfig.Default()
        .withRobotMass(robotMassWithBumpers)
        .withBumperSize(bumperLengthX, bumperWidthY)
        .withGyro(COTS.ofPigeon2())
        .withCustomModuleTranslations(moduleLocations)
        .withSwerveModule(new SwerveModuleSimulationConfig(
            driveMotorModel,
            steerMotorModel,
            moduleConstants[0].DriveMotorGearRatio,
            moduleConstants[0].SteerMotorGearRatio,
            Volts.of(moduleConstants[0].DriveFrictionVoltage),
            Volts.of(moduleConstants[0].SteerFrictionVoltage),
            Meters.of(moduleConstants[0].WheelRadius),
            KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
            wheelCOF));
    mapleSimDrive = new SwerveDriveSimulation(simulationConfig, new Pose2d());

    SwerveModuleSimulation[] moduleSimulations = mapleSimDrive.getModules();
    for (int i = 0; i < moduleSimulations.length; i++) {
      moduleSimulations[i].useDriveMotorController(
          new TalonFXMotorControllerSim(modules[i].getDriveMotor()));
      moduleSimulations[i].useSteerMotorController(
          new TalonFXMotorControllerWithRemoteCanCoderSim(
              modules[i].getSteerMotor(), modules[i].getEncoder()));
    }

    // Sim-only: the default Arena2026Rebuilt() treats the hub+ramp/bump region as a solid,
    // impassable 2D collider (dyn4j has no height axis, so it can't model a drivable ramp).
    // AddRampCollider=false drops that collider so PathPlanner autos crossing the Bump can be
    // developed/tested in sim.
    SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
    SimulatedArena.overrideSimulationTimings(simPeriod, simSubTicksPerPeriod);
    SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
  }

  /**
   * Pushes the physics body's current heading straight into the Pigeon sim state, without advancing
   * physics.
   *
   * <p>{@link #update()} is otherwise the only writer of the Pigeon's RAW yaw, and it runs on the
   * sim notifier thread. A caller that teleports the body therefore leaves the Pigeon reading the
   * STALE pre-teleport heading for a tick or two, and CTRE's odometry -- which integrates gyro
   * DELTAS -- then folds that catch-up step in as if the robot had really rotated. Calling this
   * immediately after a teleport collapses that delta to zero.
   */
  public void syncGyroToSimulationPose() {
    pigeonSim.setRawYaw(mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
  }

  /**
   * Advances the physics simulation one robot period and injects the results into the CTRE sim
   * devices.
   *
   * <p><b>Must be called from the robot loop</b> ({@code Robot.simulationPeriodic()}), never from a
   * background thread. maple-sim ships no thread of its own and documents this as the caller's
   * responsibility; stepping the dyn4j world off-thread races every robot-periodic mutation of it.
   */
  public void update() {
    SimulatedArena.getInstance().simulationPeriodic();
    pigeonSim.setRawYaw(mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
    pigeonSim.setAngularVelocityZ(
        RadiansPerSecond.of(
            mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative().omegaRadiansPerSecond));
  }

  public static class TalonFXMotorControllerSim implements SimulatedMotorController {
    private final TalonFXSimState talonFXSimState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
      this.talonFXSimState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonFXSimState.setRawRotorPosition(encoderAngle);
      talonFXSimState.setRotorVelocity(encoderVelocity);
      talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      return talonFXSimState.getMotorVoltageMeasure();
    }
  }

  public static class TalonFXMotorControllerWithRemoteCanCoderSim extends TalonFXMotorControllerSim {
    private final CANcoderSimState remoteCancoderSimState;

    public TalonFXMotorControllerWithRemoteCanCoderSim(TalonFX talonFX, CANcoder cancoder) {
      super(talonFX);
      this.remoteCancoderSimState = cancoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      remoteCancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      remoteCancoderSimState.setRawPosition(mechanismAngle);
      remoteCancoderSimState.setVelocity(mechanismVelocity);
      return super.updateControlSignal(mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }

  /** Regulates all module constants for simulation. No-ops on real hardware. */
  public static SwerveModuleConstants<?, ?, ?>[] regulateModuleConstantsForSimulation(
      SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
    for (SwerveModuleConstants<?, ?, ?> moduleConstant : moduleConstants) {
      regulateModuleConstantForSimulation(moduleConstant);
    }
    return moduleConstants;
  }

  private static void regulateModuleConstantForSimulation(SwerveModuleConstants<?, ?, ?> moduleConstants) {
    if (RobotBase.isReal()) {
      return;
    }
    moduleConstants
        .withEncoderOffset(0)
        .withDriveMotorInverted(false)
        .withSteerMotorInverted(false)
        .withEncoderInverted(false)
        .withSteerMotorGains(moduleConstants.SteerMotorGains.withKP(70).withKD(4.5))
        .withDriveFrictionVoltage(Volts.of(0.1))
        .withSteerFrictionVoltage(Volts.of(0.15))
        .withSteerInertia(KilogramSquareMeters.of(0.05));
  }
}
