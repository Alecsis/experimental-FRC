// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoAlignPOI;
import frc.robot.commands.Intake_Start;
import frc.robot.commands.Intake_Stop;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.Target;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PoseHelpers;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Roller;
import frc.robot.subsystems.intake.Intake.PivotState;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.indexing;
import frc.robot.subsystems.shooter.Shooter.Agitate;
import frc.robot.commands.AutoAlignPOI;
import frc.robot.commands.Shooting_Sequence;
import frc.robot.subsystems.Vision;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.RoboMath;

@SuppressWarnings("unused")

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                       // second
                                                                                       // max angular velocity
        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
        private final Telemetry logger = new Telemetry(MaxSpeed);
        private final ProfiledPIDController hubPID = new ProfiledPIDController(
                        5.0, 0.0, 0.0,
                        new TrapezoidProfile.Constraints(MaxAngularRate, MaxAngularRate * 2));
        private final CommandXboxController joystick = new CommandXboxController(0);
        private final CommandGenericHID controlBox = new CommandGenericHID(1);
        private final CommandXboxController sysid = new CommandXboxController(2);
        public static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public static Intake intake = Intake.getInstance();
        public static Shooter shooter = Shooter.getInstance();
        private final Limelight limelight = new Limelight("limelight-bow", "limelight-intake");
        private final AutoAlignPOI aPOI = new AutoAlignPOI(drivetrain, POI.Hub);
        private final PoseHelpers poseHelpers = new PoseHelpers(drivetrain);
        private final Vision vision = new Vision(limelight, drivetrain);
        private boolean ctrlBtn;
        private final Intake_Start cmd_Intake_Start = new Intake_Start(intake);
        private final Intake_Stop cmd_Intake_Stop = new Intake_Stop(intake);
        private final Shooting_Sequence cmd_Normal_Shooting = new Shooting_Sequence(shooter, vision, intake, drivetrain,
                        5.0);
        private final Shooting_Sequence cmd_Quick_Shooting = new Shooting_Sequence(shooter, vision, intake, drivetrain,
                        3.0);
        private final ShootCmd cmd_ShootCmd = new ShootCmd(shooter, vision, drivetrain, intake);
        private SendableChooser<Command> autoChooser;

        public RobotContainer() {
                RobotModeTriggers.teleop().onTrue(intake.homing()); // if commented out its temp removed for testing
                                                                    // since intake is not chained
                NamedCommands.registerCommand(
                                "Home Intake", intake.homing());
                NamedCommands.registerCommand("Orbit",
                                new Target(drivetrain, poseHelpers::getHubPosition, 0, () -> 0, () -> 0, true));
                NamedCommands.registerCommand(
                                "Shooting Sequence", cmd_Normal_Shooting);
                NamedCommands.registerCommand(
                                "Quick Shooting", cmd_Quick_Shooting);
                NamedCommands.registerCommand(
                                "Intake Start Sequence", Commands.parallel(
                                                intake.intakeJamReverse(),
                                                Commands.runOnce(() -> shooter.setAgitator(Agitate.IN))));
                NamedCommands.registerCommand(
                                "Intake Stop", cmd_Intake_Stop);
                autoChooser = AutoBuilder.buildAutoChooser();
                configureBindings();
                dashboard();
                SmartDashboard.putNumber("offset", 0);

        }

        private void dashboard() {
                SmartDashboard.putData(shooter);
                SmartDashboard.putData("Index Run", new InstantCommand(() -> shooter.indexControl(indexing.INDEX)));
                SmartDashboard.putData("Index Stop", new InstantCommand(() -> shooter.indexControl(indexing.STOP)));
                SmartDashboard.putData("Index Eject", new InstantCommand(() -> shooter.indexControl(indexing.EJECT)));
                SmartDashboard.putData("Stop Shooter", new InstantCommand(() -> shooter.targetRPMShooter(0)));
                SmartDashboard.putData("Agitator", new InstantCommand(() -> shooter.setAgitator(Agitate.IN)));
                SmartDashboard.putData("Agitator Stop", new InstantCommand(() -> shooter.setAgitator(Agitate.STOP)));
                SmartDashboard.putData("Set Pivot Up", new InstantCommand(() -> intake.goTo(Intake.PivotState.STOW)));
                SmartDashboard.putData("Set Pivot Down", new InstantCommand(() -> intake.goTo(Intake.PivotState.DOWN)));
                SmartDashboard.putData("Intake", new InstantCommand(() -> intake.setRoller(Roller.INTAKE)));
                SmartDashboard.putData("Intake Stop", new InstantCommand(() -> intake.setRoller(Roller.STOP)));
                SmartDashboard.putData("Auto Chooser", autoChooser);

                SmartDashboard.putNumber("Current Speed Up/Down", -joystick.getLeftY() * MaxSpeed);
                SmartDashboard.putNumber("Current Speed Right/Left", -joystick.getLeftX() * MaxSpeed);
                SmartDashboard.putNumber("Current Angle Speed", -joystick.getRightX() * MaxAngularRate);

        }

        private void configureBindings() {
                limelight.setDefaultCommand(updatePose());
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive
                                                                                                                   // forward
                                                                                                                   // with
                                                                                                                   // negative
                                                                                                                   // Y
                                                                                                                   // (forward)
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with
                                                                                                // negative X (left)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
                                                                                                            // counterclockwise
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // X (left)
                                ));
                joystick.rightBumper().whileTrue(
                                new Target(drivetrain, poseHelpers::getHubPosition, MaxSpeed, joystick::getLeftX,
                                                joystick::getLeftY, false));
                joystick.rightTrigger().whileTrue(
                                new Target(drivetrain, poseHelpers::getPassTargetPosition, MaxSpeed,
                                                joystick::getLeftX, joystick::getLeftY, false));
                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                // joystick.b().whileTrue(drivetrain.applyRequest(() ->
                // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
                // -joystick.getLeftX()))
                // ));

                // }

                // run quasistatic sysid routine on button hold, with forward and reverse directions then do dynamic
                // drivetrain
                sysid.leftBumper().onTrue(Commands.runOnce(drivetrain::useTranslationSysId));
                sysid.leftTrigger().onTrue(Commands.runOnce(drivetrain::useSteerSysId));
                sysid.rightTrigger().onTrue(Commands.runOnce(drivetrain::useRotationSysId));

                sysid.y().and(sysid.leftBumper()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                sysid.a().and(sysid.leftBumper()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                sysid.b().and(sysid.leftBumper()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
                sysid.x().and(sysid.leftBumper()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
                // Reset the field-centric headiazng on left bumper press.
                joystick.leftBumper().onTrue(Commands.runOnce(() -> {
                        drivetrain.runOnce(drivetrain::seedFieldCentric);
                        var mt = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-bow");
                        if (mt != null && mt.tagCount > 0 && mt.pose.getX() != 0) {
                                drivetrain.resetPose(mt.pose);
                        }
                }));
                drivetrain.registerTelemetry(logger::telemeterize);
                // joystick.povUp().whileTrue(new AutoAlignPOI(drivetrain, POI.Hub));
                // joystick.povLeft().whileTrue(new AutoAlignPOI(drivetrain, POI.poi1));
                joystick.povRight().whileTrue(new AutoAlignPOI(drivetrain, POI.Right));
                joystick.povLeft().whileTrue(new AutoAlignPOI(drivetrain, POI.Left));
                joystick.x().whileTrue(new AutoAlignPOI(drivetrain, POI.LeftStage));
                joystick.y().whileTrue(new AutoAlignPOI(drivetrain, POI.CenterStage));
                joystick.b().whileTrue(new AutoAlignPOI(drivetrain, POI.RightStage));
                controlBox.button(2).whileTrue(new ShootCmd(shooter, vision, drivetrain, intake));
                controlBox.button(1).onTrue(Commands.runOnce(() -> shooter.targetRPMShooter(1600)));
                controlBox.button(1)
                                .whileTrue(Commands.sequence(
                                                Commands.waitUntil(
                                                                () -> shooter.shooterAtSpeed(shooter.getTargetRPM())),
                                                Commands.run(() -> {
                                                        shooter.indexControl(Shooter.indexing.INDEX);
                                                        shooter.setAgitator(Shooter.Agitate.IN);
                                                })));

                controlBox.button(1).onFalse(Commands.runOnce(() -> {
                        shooter.targetRPMShooter(1000);
                        shooter.indexControl(Shooter.indexing.STOP);
                        shooter.setAgitator(Shooter.Agitate.STOP);
                }));
                /*
                 * controlBox.button(2).onFalse(Commands.runOnce(() -> {
                 * shooter.targetRPMShooter(800);
                 * shooter.indexControl(Shooter.indexing.STOP);
                 * shooter.setAgitator(Shooter.Agitate.STOP);
                 * }));
                 */
                controlBox.button(4).whileTrue(Commands.runOnce(() -> {
                        shooter.indexControl(indexing.EJECT);
                        shooter.setAgitator(Agitate.OUT);
                }));
                controlBox.button(4).onFalse(Commands.runOnce(() -> {

                        shooter.indexControl(indexing.STOP);
                        shooter.setAgitator(Agitate.STOP);

                }));
                /*
                 * 
                 * controlBox.button(2).whileFalse(
                 * Commands.sequence(
                 * Commands.runOnce(() -> shooter.targetRPMShooter(0)),
                 * Commands.runOnce(() -> shooter.indexControl(indexing.STOP)))
                 * );
                 */
                // operator.a().whileTrue(shooter.index());
                // operator.b().whileTrue(shooter.indexEject());
                controlBox.button(3).whileTrue(intake.eject()); // inverted
                controlBox.button(3).onFalse(Commands.either(intake.intake(), intake.stopRoller(), () -> ctrlBtn));

                controlBox.button(6).whileTrue(intake.intake()); // inverted
                controlBox.button(6).whileTrue(Commands.run(() -> {
                        shooter.setAgitator(Agitate.OUT);
                        ctrlBtn = true;
                }));
                controlBox.button(6).onFalse(intake.stopRoller()); // inverted
                controlBox.button(6).onFalse(Commands.runOnce(() -> {
                        shooter.setAgitator(Agitate.STOP);
                        ctrlBtn = false;
                }));
                controlBox.button(5).whileTrue(intake.agitatePivot());
                controlBox.button(5).whileTrue(intake.intake());
                controlBox.button(5).onFalse(Commands.either(intake.intake(), intake.stopRoller(), () -> ctrlBtn));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public Command updatePose() {
                return limelight.run(() -> {
                        double omega = drivetrain.getState().Speeds.omegaRadiansPerSecond;
                        if (Math.abs(omega) > 2 * Math.PI)
                                return;
                        final Pose2d currentPose = drivetrain.getState().Pose;
                        var measurements = limelight.getMeasurement(currentPose);
                        for (Limelight.Measurement m : measurements) {
                                drivetrain.addVisionMeasurement(
                                                m.poseEstimate.pose,
                                                m.poseEstimate.timestampSeconds,
                                                m.standardDeviations);
                        }
                })
                                .ignoringDisable(true);
        }
}
