// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.utility.RoboMath;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shooting_Sequence extends SequentialCommandGroup {

  /** Creates a new ShootCmd. */
  public Shooting_Sequence(Shooter shooter, Vision vision, Intake intake, CommandSwerveDrivetrain swerve,
      double timeout) {
    addCommands(
        Commands.parallel(
            intake.agitatePivot(),
            Commands.sequence(
                Commands.run(() -> {
                  var rpmOpt = RoboMath.calculateRPMFromVision(vision, swerve.getState().Pose);
                  rpmOpt.ifPresent(shooter::targetRPMShooter);
                  rpmOpt.ifPresent(rpm -> Logger.recordOutput("Shooter/Auto_TargetRPM", rpm));

                }, shooter).until(() -> shooter.shooterAtSpeed(shooter.getTargetRPM())).withTimeout(1),
                Commands.waitSeconds(0.3),
                Commands.run(() -> {
                  shooter.setAgitator(Shooter.Agitate.IN);
                  shooter.indexControl(Shooter.indexing.INDEX);
                  var rpmOpt = RoboMath.calculateRPMFromVision(vision, swerve.getState().Pose);
                  rpmOpt.ifPresent(shooter::targetRPMShooter);
                  rpmOpt.ifPresent(rpm -> Logger.recordOutput("Shooter/Auto_TargetRPM", rpm));

                }, shooter).withTimeout(timeout)))
            .finallyDo(() -> {
              shooter.indexControl(Shooter.indexing.STOP);
              shooter.setAgitator(Shooter.Agitate.STOP);
              intake.goTo(Intake.PivotState.DOWN);
              shooter.targetRPMShooter(700);
            }));
  }
}
