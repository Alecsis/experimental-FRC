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
      drivetrain.runOnce(drivetrain::seedFieldCentric);
      vision.getPoseResetEstimate().ifPresent(drivetrain::resetPose);
    }));

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
        drivetrain.runOnce(drivetrain::seedFieldCentric);
        vision.getPoseResetEstimate().ifPresent(drivetrain::resetPose);
      }));

      // 4. Superstructure routing -- mirrors the operator control board
      simController.a().whileTrue(superstructure.shootCmd());
      simController.x().whileTrue(superstructure.intakeCmd());
      simController.b().whileTrue(superstructure.ejectCmd());
    }
  }

  /** Called every scheduler run from {@link RobotContainer#periodic()} to publish driver-input telemetry. */
  public void periodic(double maxSpeed, double maxAngularRate) {
    SmartDashboard.putNumber("Current Speed Up/Down", -joystick.getLeftY() * maxSpeed);
    SmartDashboard.putNumber("Current Speed Right/Left", -joystick.getLeftX() * maxSpeed);
    SmartDashboard.putNumber("Current Angle Speed", -joystick.getRightX() * maxAngularRate);
  }
}
