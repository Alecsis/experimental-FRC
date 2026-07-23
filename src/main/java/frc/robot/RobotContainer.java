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
import java.util.Set;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.TrajectoryErrorTracker;

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
                        new TrajectoryErrorTracker(() -> drivetrain.getState().Pose);
        private SendableChooser<Command> autoChooser;

        /**
         * NamedCommand names allowed inside a PathPlanner "parallel" block (which compiles to
         * ParallelCommandGroup, requiring every branch to finish). AutoCommandSafetyTest asserts
         * every parallel-block NamedCommand across all .auto files is in this set -- add a new name
         * here the moment you register a NamedCommand that will be used inside a "parallel" block,
         * or the test will fail and tell you why.
         *
         * <p>Most entries ("Intake Start Sequence", "Shooting Sequence", "Quick Shooting") are
         * provably bounded -- each wraps a hard {@code .withTimeout(...)}, so they are guaranteed to
         * finish. "Orbit" is different: it maps to {@code drivetrain.trackHub(vision, 0, () -> 0, ()
         * -> 0, true)}, a {@code finishOnAlign}-style condition with no timeout backstop. It is
         * trusted to converge (vision acquires a target and heading settles within tolerance), not
         * proven to terminate -- if vision never acquires a target or heading never converges,
         * {@code isFinished()} can return false forever and hang the auto, the exact failure mode
         * this allowlist/test exists to catch. Do not treat "Orbit" as bounded the way the
         * timeout-based entries are.
         */
        public static final Set<String> BOUNDED_NAMED_COMMANDS = Set.of(
                        "Intake Start Sequence", "Orbit", "Shooting Sequence", "Quick Shooting");

        public RobotContainer() {
                drivetrain.setTrajectoryErrorTracker(trajectoryErrorTracker);
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
         * pairing this loop's pose against last loop's stale setpoint.
         */
        public void trajectoryTrackerPeriodic() {
                trajectoryErrorTracker.periodic();
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
}
