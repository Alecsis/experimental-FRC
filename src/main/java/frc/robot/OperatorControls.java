// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;

/**
 * Owns the driver (`joystick`), operator (`controlBox`), and sim-mirror (`simController`) HID
 * objects and their button bindings -- split out of RobotContainer so control-surface changes
 * don't require touching subsystem wiring and vice versa. The bench/tuning `sysid` controller
 * stays in RobotContainer: it's characterization tooling, not driver/operator gameplay controls.
 */
public class OperatorControls {
  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandGenericHID controlBox = new CommandGenericHID(1);
  private final CommandXboxController simController = new CommandXboxController(3);

  /** Wires driver, operator, and sim-mirror bindings. Call once from RobotContainer's constructor. */
  public void configureBindings(CommandSwerveDrivetrain drivetrain, Vision vision,
      Superstructure superstructure, SwerveRequest.FieldCentric drive,
      double maxSpeed, double maxAngularRate) {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * maxSpeed)
            .withVelocityY(-joystick.getLeftX() * maxSpeed)
            .withRotationalRate(-joystick.getRightX() * maxAngularRate)));

    joystick.rightBumper().whileTrue(
        drivetrain.trackHub(vision, maxSpeed, joystick::getLeftX, joystick::getLeftY, false));
    joystick.rightTrigger().whileTrue(
        drivetrain.trackPassTarget(vision, maxSpeed, joystick::getLeftX, joystick::getLeftY, false));

    // Reset the field-centric heading on left bumper press.
    joystick.leftBumper().onTrue(Commands.runOnce(() -> {
      drivetrain.seedFieldCentric();
      vision.getPoseResetEstimate().ifPresent(drivetrain::resetPose);
    }));

    joystick.povRight().whileTrue(drivetrain.driveToPOI(POI.Right));
    joystick.povLeft().whileTrue(drivetrain.driveToPOI(POI.Left));
    joystick.x().whileTrue(drivetrain.driveToPOI(POI.LeftStage));
    joystick.y().whileTrue(drivetrain.driveToPOI(POI.CenterStage));
    joystick.b().whileTrue(drivetrain.driveToPOI(POI.RightStage));

    /*
     * Operator control board.
     *
     * The board is wired as Superstructure's DEFAULT command, not as button Triggers. Buttons 5
     * and 6 are maintained physical switches, and an edge-triggered binding cannot represent them
     * correctly: a switch already ON at enable produces no false->true edge, and a whileTrue
     * command cancelled by anything else is never rescheduled while the switch stays held. A
     * level-triggered default command reads the live switch positions every tick and is
     * re-scheduled automatically by the CommandScheduler whenever nothing else requires
     * Superstructure, which covers already-on-at-enable, disable->enable, and interruption with
     * one mechanism and no latch.
     *
     * The command itself is inert outside teleop (see operatorPolicyCmd), so these controls have
     * no autonomous effect. STOWED is just the zero-active-control teleop intent, so there is no
     * separate teleop-start stow command competing for the subsystem and no dependence on the
     * order in which RobotModeTriggers were registered.
     */
    Trigger fallbackShot = controlBox.button(1);
    Trigger visionShot = controlBox.button(2);
    Trigger unifiedEject = controlBox.button(3);
    // Button 4 is intentionally reserved and left unbound -- its absence is deliberate, not an
    // oversight. The legacy separate indexer-only eject it used to hold is now folded into the
    // unified button 3 eject.
    Trigger bounceToggle = controlBox.button(5);
    Trigger intakeToggle = controlBox.button(6);

    if (RobotBase.isSimulation()) {
      // Mirror the board onto the sim controller through the SAME policy, so simulation exercises
      // the real arbitration -- including its teleop-only gate -- instead of a second set of
      // competing commands with different lifecycle rules.
      fallbackShot = fallbackShot.or(simController.y());
      visionShot = visionShot.or(simController.a());
      unifiedEject = unifiedEject.or(simController.b());
      bounceToggle = bounceToggle.or(simController.povUp());
      intakeToggle = intakeToggle.or(simController.x());
    }

    superstructure.setDefaultCommand(superstructure.operatorPolicyCmd(
        fallbackShot, visionShot, unifiedEject, bounceToggle, intakeToggle));

    if (RobotBase.isSimulation()) {
      // 1. Override default drivetrain command with Port 3 Controller
      drivetrain.setDefaultCommand(
          drivetrain.applyRequest(() -> drive
              .withVelocityX(-simController.getLeftY() * maxSpeed)
              .withVelocityY(-simController.getLeftX() * maxSpeed)
              .withRotationalRate(-simController.getRightX() * maxAngularRate)));

      // 2. Vision Target Tracking (Hold Right Bumper or Right Trigger)
      simController.rightBumper().whileTrue(
          drivetrain.trackHub(vision, maxSpeed, simController::getLeftX, simController::getLeftY, false));
      simController.rightTrigger().whileTrue(
          drivetrain.trackPassTarget(vision, maxSpeed, simController::getLeftX, simController::getLeftY, false));

      // 3. Reset Gyro & Heading (Left Bumper)
      simController.leftBumper().onTrue(Commands.runOnce(() -> {
        drivetrain.seedFieldCentric();
        vision.getPoseResetEstimate().ifPresent(drivetrain::resetPose);
      }));

      // 4. Superstructure routing is folded into the operator-intent trigger above (a/b/x/y/povUp
      // are OR'd into the board's own controls) so sim and the real board share one arbitration
      // path instead of running a second set of commands that fight it.
    }
  }

  /** Called every scheduler run from {@link RobotContainer#periodic()} to publish driver-input telemetry. */
  public void periodic(double maxSpeed, double maxAngularRate) {
    SmartDashboard.putNumber("Current Speed Up/Down", -joystick.getLeftY() * maxSpeed);
    SmartDashboard.putNumber("Current Speed Right/Left", -joystick.getLeftX() * maxSpeed);
    SmartDashboard.putNumber("Current Angle Speed", -joystick.getRightX() * maxAngularRate);
  }
}
