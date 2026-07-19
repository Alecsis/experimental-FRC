// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility.simulation;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

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

  public MapleSimSwerveDrivetrain(
      Time simPeriod,
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
    SimulatedArena.overrideSimulationTimings(simPeriod, 1);
    SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
  }

  /** Advances the physics simulation one tick and injects the results into the CTRE sim devices. */
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
