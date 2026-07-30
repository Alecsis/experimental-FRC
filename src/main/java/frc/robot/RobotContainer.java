// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Set;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.TrajectoryErrorTracker;
import frc.robot.auto.AutonomousHealthMonitor;

@SuppressWarnings("unused")

public class RobotContainer {
        /**
         * NamedCommands that are structurally guaranteed to terminate on their own -- checked by
         * {@code AutoCommandSafetyTest} against every NamedCommand used inside a PathPlanner
         * "parallel" block (which requires every branch to finish). A NamedCommand belongs here
         * only once it has a real time bound or self-finishing condition; see
         * {@link frc.robot.subsystems.superstructure.Superstructure#intakeSequence(double)}'s
         * javadoc for the bug class this guards against.
         */
        public static final Set<String> BOUNDED_NAMED_COMMANDS = Set.of(
                        "Home Intake", "Orbit", "Shooting Sequence", "Quick Shooting",
                        "Intake Start Sequence", "Intake Stop");

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
        private final CommandXboxController sysid = new CommandXboxController(2);
        private final OperatorControls operatorControls = new OperatorControls();
        public static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public static Intake intake = Intake.getInstance();
        public static Shooter shooter = Shooter.getInstance();
        private final Vision vision = Vision.getInstance(drivetrain);
        private final Superstructure superstructure = Superstructure.getInstance(drivetrain);
        // Owned here, not a singleton -- constructed with drivetrain's pose supplier, so it
        // necessarily postdates drivetrain's own construction above. Injected into drivetrain
        // below since configureAutoBuilder() already registered its callbacks by this point.
        private final TrajectoryErrorTracker trajectoryErrorTracker =
                        new TrajectoryErrorTracker(() -> drivetrain.getState().Pose, Timer::getFPGATimestamp);
        // Phase 0.5 (Autonomous Observability Layer, docs/Autonomous_Recovery_Readiness_Assessment.md)
        // -- read-only telemetry aggregation, wired at the same point as trajectoryErrorTracker
        // (see trajectoryTrackerPeriodic()) since it consumes that tracker's own live outputs.
        private final AutonomousHealthMonitor autonomousHealthMonitor = new AutonomousHealthMonitor(
                        trajectoryErrorTracker::getLateralErrorMeters,
                        trajectoryErrorTracker::getLongitudinalErrorMeters,
                        () -> drivetrain.getState().Pose,
                        Timer::getFPGATimestamp,
                        trajectoryErrorTracker::getSecondsSinceLastFreshTargetPose,
                        vision::getLastRejectedJumpMeters);
        private SendableChooser<Command> autoChooser;

        public RobotContainer() {
                drivetrain.setTrajectoryErrorTracker(trajectoryErrorTracker);
                RobotModeTriggers.teleop().onTrue(intake.homing()); // if commented out its temp removed for testing
                                                                    // since intake is not chained
                NamedCommands.registerCommand(
                                "Home Intake", intake.homing());
                // trackHub(..., finishOnAlign=true) only self-finishes once heading error drops
                // under trackTarget's tolerance -- with no fallback, a stale pose or an
                // unconverged heading PID hangs this forever. It runs inside a
                // parallel(Orbit, Shooting Sequence) block in every auto (a ParallelCommandGroup,
                // which requires every branch to finish), so an unbounded Orbit would silently
                // stall the whole auto -- the same bug class as the old intakeCmd() hang (see
                // Superstructure.intakeSequence's javadoc). Bounded here the same way.
                NamedCommands.registerCommand("Orbit",
                                drivetrain.trackHub(vision, 0, () -> 0, () -> 0, true).withTimeout(2.0));
                NamedCommands.registerCommand(
                                "Shooting Sequence", superstructure.shootingSequence(5.0));
                NamedCommands.registerCommand(
                                "Quick Shooting", superstructure.shootingSequence(3.0));
                // Jam recovery now lives inside Intake.setRoller() itself (mechanism-level), so
                // Superstructure.INTAKING gets it automatically -- routing through the state machine no
                // longer drops stall protection from autos.
                //
                // intakeSequence(5.0), not the raw hold-forever intakeCmd(): every .auto file runs this
                // inside a PathPlanner "parallel" block (ParallelCommandGroup, waits for every branch),
                // and intakeCmd() never self-finishes -- discovered via the auto-regression-suite work
                // stalling every in-scope auto dead after its first path segment.
                NamedCommands.registerCommand(
                                "Intake Start Sequence", superstructure.intakeSequence(5.0));
                NamedCommands.registerCommand(
                                "Intake Stop", superstructure.stowCmd());
                autoChooser = AutoBuilder.buildAutoChooser();
                configureBindings();
                dashboard();
                SmartDashboard.putNumber("offset", 0);

        }

        private void dashboard() {
                SmartDashboard.putData(shooter);
                // State-machine-respecting dashboard actions only -- these route through Superstructure
                // so testing from the dashboard can't fight the periodic() arbitration the way raw
                // subsystem InstantCommands used to (e.g. dashboard "Intake" + operator "Eject" held
                // at once used to fight over the roller; now both funnel into the same wanted-state).
                SmartDashboard.putData("State: Intake", superstructure.intakeCmd());
                SmartDashboard.putData("State: Eject", superstructure.ejectCmd());
                SmartDashboard.putData("State: Stow", superstructure.stowCmd());
                SmartDashboard.putData("State: Shoot", superstructure.shootCmd());
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        /**
         * Called every scheduler run from {@link Robot#robotPeriodic()} to publish live
         * dashboard values.
         */
        public void periodic() {
                operatorControls.periodic(MaxSpeed, MaxAngularRate);
        }

        /**
         * Called after {@link CommandScheduler#run()} from {@link Robot#robotPeriodic()} --
         * deliberately after, not before. By this point this loop's PathPlanner setpoint (if any)
         * has already been produced by the scheduler run that just finished, so pairing it with
         * drivetrain.getState().Pose here reads both halves from the same instant instead of
         * pairing this loop's pose against last loop's stale setpoint. autonomousHealthMonitor
         * runs right after for the same reason -- it consumes trajectoryErrorTracker's
         * just-updated outputs and needs the same same-instant pose pairing.
         */
        public void trajectoryTrackerPeriodic() {
                trajectoryErrorTracker.periodic();
                autonomousHealthMonitor.periodic();
        }

        private void configureBindings() {
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
                drivetrain.registerTelemetry(logger::telemeterize);

                // Driver, operator, and sim-mirror HID bindings -- see OperatorControls.
                operatorControls.configureBindings(drivetrain, vision, superstructure, drive, MaxSpeed, MaxAngularRate);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        /** Releases drivetrain simulation and Phoenix odometry threads during test/sim shutdown. */
        public void close() {
                drivetrain.close();
        }
}
