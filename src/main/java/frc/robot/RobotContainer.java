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
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Roller;
import frc.robot.subsystems.intake.Intake.PivotState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.indexing;
import frc.robot.subsystems.shooter.Shooter.Agitate;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;

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
        private final CommandXboxController simController = new CommandXboxController(3);
        public static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public static Intake intake = Intake.getInstance();
        public static Shooter shooter = Shooter.getInstance();
        private final Vision vision = Vision.getInstance(drivetrain);
        private final Superstructure superstructure = Superstructure.getInstance(drivetrain);
        private SendableChooser<Command> autoChooser;

        public RobotContainer() {
                RobotModeTriggers.teleop().onTrue(intake.homing()); // if commented out its temp removed for testing
                                                                    // since intake is not chained
                NamedCommands.registerCommand(
                                "Home Intake", intake.homing());
                NamedCommands.registerCommand("Orbit",
                                drivetrain.trackHub(vision, 0, () -> 0, () -> 0, true));
                NamedCommands.registerCommand(
                                "Shooting Sequence", superstructure.shootingSequence(5.0));
                NamedCommands.registerCommand(
                                "Quick Shooting", superstructure.shootingSequence(3.0));
                NamedCommands.registerCommand(
                                "Intake Start Sequence", Commands.parallel(
                                                intake.intakeJamReverse(),
                                                Commands.runOnce(() -> shooter.setAgitator(Agitate.IN))));
                NamedCommands.registerCommand(
                                "Intake Stop", intake.stopRoller());
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
        }

        /**
         * Called every scheduler run from {@link Robot#robotPeriodic()} to publish live
         * dashboard values.
         */
        public void periodic() {
                SmartDashboard.putNumber("Current Speed Up/Down", -joystick.getLeftY() * MaxSpeed);
                SmartDashboard.putNumber("Current Speed Right/Left", -joystick.getLeftX() * MaxSpeed);
                SmartDashboard.putNumber("Current Angle Speed", -joystick.getRightX() * MaxAngularRate);
        }

        private void configureBindings() {
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
                                drivetrain.trackHub(vision, MaxSpeed, joystick::getLeftX, joystick::getLeftY, false));
                joystick.rightTrigger().whileTrue(
                                drivetrain.trackPassTarget(vision, MaxSpeed, joystick::getLeftX, joystick::getLeftY,
                                                false));
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

                // run quasistatic sysid routine on button hold, with forward and reverse
                // directions then do dynamic
                // drivetrain
                sysid.leftBumper().onTrue(Commands.runOnce(drivetrain::useTranslationSysId));
                sysid.leftTrigger().onTrue(Commands.runOnce(drivetrain::useSteerSysId));
                sysid.rightTrigger().onTrue(Commands.runOnce(drivetrain::useRotationSysId));

                sysid.y().and(sysid.leftBumper())
                                .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                sysid.a().and(sysid.leftBumper())
                                .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                sysid.b().and(sysid.leftBumper()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
                sysid.x().and(sysid.leftBumper()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
                // Reset the field-centric headiazng on left bumper press.
                joystick.leftBumper().onTrue(Commands.runOnce(() -> {
                        drivetrain.runOnce(drivetrain::seedFieldCentric);
                        vision.getPoseResetEstimate().ifPresent(drivetrain::resetPose);
                }));
                drivetrain.registerTelemetry(logger::telemeterize);
                // joystick.povUp().whileTrue(drivetrain.driveToPOI(POI.Hub));
                // joystick.povLeft().whileTrue(drivetrain.driveToPOI(POI.poi1));
                joystick.povRight().whileTrue(drivetrain.driveToPOI(POI.Right));
                joystick.povLeft().whileTrue(drivetrain.driveToPOI(POI.Left));
                joystick.x().whileTrue(drivetrain.driveToPOI(POI.LeftStage));
                joystick.y().whileTrue(drivetrain.driveToPOI(POI.CenterStage));
                joystick.b().whileTrue(drivetrain.driveToPOI(POI.RightStage));
                /*
                 * Operator control board -- pure routing into Superstructure state requests.
                 * Manual fixed-RPM shooting lives behind the "Shooter Tuning Mode" dashboard
                 * toggle, which shootCmd() honors through the Superstructure's RPM arbitration.
                 */
                controlBox.button(2).whileTrue(superstructure.shootCmd());
                controlBox.button(6).whileTrue(superstructure.intakeCmd());
                controlBox.button(3).whileTrue(superstructure.ejectCmd());

                if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
                        // 1. Override default drivetrain command with Port 3 Controller
                        drivetrain.setDefaultCommand(
                                        drivetrain.applyRequest(() -> drive
                                                        .withVelocityX(-simController.getLeftY() * MaxSpeed)
                                                        .withVelocityY(-simController.getLeftX() * MaxSpeed)
                                                        .withRotationalRate(
                                                                        -simController.getRightX() * MaxAngularRate)));

                        // 2. Vision Target Tracking (Hold Right Bumper or Right Trigger)
                        simController.rightBumper().whileTrue(
                                        drivetrain.trackHub(vision, MaxSpeed, simController::getLeftX,
                                                        simController::getLeftY, false));
                        simController.rightTrigger().whileTrue(
                                        drivetrain.trackPassTarget(vision, MaxSpeed, simController::getLeftX,
                                                        simController::getLeftY, false));

                        // 3. Reset Gyro & Heading (Left Bumper)
                        simController.leftBumper().onTrue(Commands.runOnce(() -> {
                                drivetrain.runOnce(drivetrain::seedFieldCentric);
                                vision.getPoseResetEstimate().ifPresent(drivetrain::resetPose);
                        }));

                        // 4. Superstructure routing -- mirrors the operator control board
                        simController.a().whileTrue(superstructure.shootCmd());
                        simController.x().whileTrue(superstructure.intakeCmd());
                        simController.b().whileTrue(superstructure.ejectCmd());
                }
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
