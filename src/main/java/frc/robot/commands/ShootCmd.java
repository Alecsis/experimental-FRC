// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.intake.Intake.Roller;
import frc.robot.utility.RoboMath;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCmd extends SequentialCommandGroup {

  /** Creates a new ShootCmd. */
  public ShootCmd(Shooter shooter, Vision vision, CommandSwerveDrivetrain swerve, Intake intake) {
    addCommands(
        Commands.parallel(
            intake.agitatePivot(),
            Commands.sequence(
                Commands.run(() -> {
                  if (shooter.shooterTuningModeEnable) {
                    shooter.targetRPMShooter(shooter.getTargetRPM());

                  } else {
                    var rpmOpt = RoboMath.calculateRPMFromVision(vision, swerve.getState().Pose);
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
                    var rpmOpt = RoboMath.calculateRPMFromVision(vision, swerve.getState().Pose);
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
            }));
  }
}