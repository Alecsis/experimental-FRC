// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Roller;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

/**
 * Singleton Superstructure. Coordinates Shooter + Intake + Vision + drivetrain state together for
 * complex multi-subsystem sequences (shooting). Individual subsystems only handle their own
 * mechanism control; cross-subsystem orchestration lives here.
 */
public class Superstructure extends SubsystemBase {
  private static Superstructure instance;

  /** Creates (on first call) or returns the Superstructure Singleton. */
  public static Superstructure getInstance(CommandSwerveDrivetrain drivetrain) {
    if (instance == null) {
      instance = new Superstructure(drivetrain);
    }
    return instance;
  }

  private final Shooter shooter = Shooter.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Vision vision;

  /** Creates a new Superstructure. Use {@link #getInstance(CommandSwerveDrivetrain)} instead of constructing directly. */
  private Superstructure(CommandSwerveDrivetrain drivetrain) {
    this.vision = Vision.getInstance(drivetrain);
  }

  public Command shootCmd() {
    return Commands.parallel(
        intake.agitatePivot(),
        Commands.sequence(
            Commands.run(() -> {
              if (shooter.shooterTuningModeEnable) {
                shooter.targetRPMShooter(shooter.getTargetRPM());
              } else {
                var rpmOpt = vision.calculateRPM();
                rpmOpt.ifPresent(shooter::targetRPMShooter);
              }
            }).until(() -> shooter.shooterAtSpeed(shooter.getTargetRPM())),
            Commands.waitSeconds(0.1),
            Commands.run(() -> {
              shooter.indexControl(Shooter.indexing.INDEX);
              shooter.setAgitator(Shooter.Agitate.IN);

              if (shooter.shooterTuningModeEnable) {
                shooter.targetRPMShooter(shooter.getTargetRPM());
              } else {
                var rpmOpt = vision.calculateRPM();
                rpmOpt.ifPresent(shooter::targetRPMShooter);
                rpmOpt.ifPresent(rpm -> Logger.recordOutput("Shooter/TargetRPM", rpm));
              }
            }, shooter)))
        .finallyDo(() -> {
          intake.setRoller(Roller.STOP);
          shooter.indexControl(Shooter.indexing.STOP);
          shooter.setAgitator(Shooter.Agitate.STOP);
          shooter.targetRPMShooter(700);
          intake.goTo(Intake.PivotState.DOWN);
        });
  }

  public Command shootingSequence(double timeoutSeconds) {
    return Commands.parallel(
        intake.agitatePivot(),
        Commands.sequence(
            Commands.run(() -> {
              var rpmOpt = vision.calculateRPM();
              rpmOpt.ifPresent(shooter::targetRPMShooter);
              rpmOpt.ifPresent(rpm -> Logger.recordOutput("Shooter/Auto_TargetRPM", rpm));
            }, shooter).until(() -> shooter.shooterAtSpeed(shooter.getTargetRPM())).withTimeout(1),
            Commands.waitSeconds(0.3),
            Commands.run(() -> {
              shooter.setAgitator(Shooter.Agitate.IN);
              shooter.indexControl(Shooter.indexing.INDEX);
              var rpmOpt = vision.calculateRPM();
              rpmOpt.ifPresent(shooter::targetRPMShooter);
              rpmOpt.ifPresent(rpm -> Logger.recordOutput("Shooter/Auto_TargetRPM", rpm));
            }, shooter).withTimeout(timeoutSeconds)))
        .finallyDo(() -> {
          shooter.indexControl(Shooter.indexing.STOP);
          shooter.setAgitator(Shooter.Agitate.STOP);
          intake.goTo(Intake.PivotState.DOWN);
          shooter.targetRPMShooter(700);
        });
  }
}
