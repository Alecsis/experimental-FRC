package frc.robot.controlboard;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.io.MotorIO.Mode;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.rebuilt.ShiftUtil;
import frc.lib.util.ControllerUtil;
import frc.lib.util.FieldLayout;
import frc.robot.Robot;
import frc.robot.shooting.Shooting;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.feederrollers.FeederRollers;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopperrollers.HopperRollers;
import frc.robot.subsystems.intakedeploy.IntakeDeploy;
import frc.robot.subsystems.intakedeploy.IntakeDeployConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.tunnelrollers.TunnelRollers;

public class ControlBoard extends SubsystemBase {
	public static final ControlBoard mInstance = new ControlBoard();

	private CommandXboxController driver = ControlBoardConstants.mDriverController;
	private CommandXboxController operator = ControlBoardConstants.mOperatorController;

	private final Trigger overrideTrigger = driver.rightBumper();

	public void configureBindings() {
		Drive.mInstance.setDefaultCommand(Drive.mInstance.followSwerveRequestCommand(
				DriveConstants.teleopRequest, DriveConstants.teleopRequestUpdater));
		driver.back()
				.onTrue(Commands.runOnce(
								() -> Drive.mInstance.getGeneratedDrive().seedFieldCentric(), Drive.mInstance)
						.ignoringDisable(true));

		driver.start()
				.onTrue(Commands.runOnce(() -> Robot.resetPoseForAuto = true).ignoringDisable(true));

		ShiftUtil.bindWinOverride(operator.back(), operator.start());

		driverControls();
		// shotTuningControls();
		// jogControls();
		// bringupControls();
	}

	public void bringupControls() {
		driver.x().onTrue(TunnelRollers.mInstance.setpointCommand(TunnelRollers.UP));
		driver.b().onTrue(TunnelRollers.mInstance.setpointCommand(TunnelRollers.IDLE));
		driver.y().onTrue(TunnelRollers.mInstance.setpointCommand(TunnelRollers.SLOW));
		driver.a().onTrue(TunnelRollers.mInstance.setpointCommand(TunnelRollers.DOWN));

		driver.leftTrigger().onTrue(Shooter.mInstance.setpointCommand(Shooter.FERRY));
		driver.leftBumper().onTrue(Shooter.mInstance.setpointCommand(Shooter.TESTSHOT));
		driver.rightBumper().onTrue(Shooter.mInstance.setpointCommand(Shooter.SLOW));
		driver.rightTrigger().onTrue(Shooter.mInstance.setpointCommand(Shooter.IDLE));

		driver.povLeft().onTrue(FeederRollers.mInstance.setpointCommand(FeederRollers.SLOW_REVERSE));
		driver.povRight().onTrue(FeederRollers.mInstance.setpointCommand(FeederRollers.FEED_VOLTAGE));
		driver.povDown().onTrue(FeederRollers.mInstance.setpointCommand(FeederRollers.IDLE));

		operator.povUp().onTrue(HopperRollers.mInstance.setpointCommand(HopperRollers.INTAKE));
		operator.povDown().onTrue(HopperRollers.mInstance.setpointCommand(HopperRollers.IDLE));

		operator.x().onTrue(Hood.mInstance.setpointCommand(Hood.TEST));
		operator.y().onTrue(Hood.mInstance.setpointCommand(Hood.TEST2));
		operator.a().onTrue(Hood.mInstance.setpointCommand(Hood.STOW));

		operator.leftBumper().onTrue(IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.DEPLOYED_SETPOINT));
		operator.rightBumper().onTrue(IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.STOWED_SETPOINT));
		operator.leftTrigger().onTrue(IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.PARTIAL_IN));

		// operator.leftBumper().onTrue(Climber.mInstance.setpointCommand(Climber.MAX));
		// operator.rightBumper().onTrue(Climber.mInstance.setpointCommand(Climber.CLIMB));
		// operator.leftTrigger().onTrue(Climber.mInstance.setpointCommand(Climber.STOW));

		// driver.rightBumper().onTrue(IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.DEPLOYED_SETPOINT));
		// // driver.x().onTrue(IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.STOWED_SETPOINT));
		// driver.leftBumper().onTrue(IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.STOWED_SETPOINT));

		// driver.x().onTrue(FeederRollers.mInstance.setpointCommand(FeederRollers.FEED));
		// driver.y().onTrue(FeederRollers.mInstance.setpointCommand(FeederRollers.REVERSE));
		// driver.leftBumper().onTrue(FeederRollers.mInstance.setpointCommand(FeederRollers.IDLE));

		// driver.leftTrigger().onTrue(Shooter.mInstance.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(12.0))));
		// driver.rightTrigger().onTrue(Shooter.mInstance.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(6.0))));

		// driver.leftTrigger().onTrue(Shooter.mInstance.setpointCommand(Shooter.FERRY));
		// driver.rightTrigger().onTrue(HopperRollers.mInstance.setpointCommand(HopperRollers.INTAKE));
		// driver.leftBumper().onTrue(TunnelRollers.mInstance.setpointCommand(TunnelRollers.UP));
		// driver.rightBumper().onTrue(FeederRollers.mInstance.setpointCommand(FeederRollers.FEED));

		// driver.a().onTrue(Superstructure.mInstance.idle());
		// driver.y().onTrue(Climber.mInstance.setpointCommand(Climber.STOW));
		// driver.b().onTrue(Climber.mInstance.setpointCommand(Climber.MAX));
		// driver.x().onTrue(FeederRollers.mInstance.setpointCommand(FeederRollers.REVERSE));

		// driver.a().onTrue(Superstructure.mInstance.tuck());
		// driver.x().onTrue(IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.DEPLOYED_SETPOINT));

		// operator.leftBumper().onTrue(IntakeRollers.mInstance.setpointCommand(IntakeRollers.INTAKE));

		// driver.x().onTrue(Hood.mInstance.setpointCommand(Hood.STOW));

		// driver.leftTrigger().onTrue(Superstructure.mInstance.intake());
		// driver.rightTrigger()
		// 		.onTrue(Superstructure.mInstance.shootWithMovement())
		// 		.onTrue(Shooting.mInstance.followShootOnTheMoveRequest().onlyWhile(driver.rightTrigger()));

		// driver.leftBumper().onTrue(Superstructure.mInstance.idle());
		// driver.rightBumper().onTrue(Superstructure.mInstance.continousShooting());

		// ControllerUtil.bindJog(
		// 		Shooter.mInstance,
		// 		RPM.of(100).baseUnitMagnitude(),
		// 		RPM.of(0.0).baseUnitMagnitude(),
		// 		RPM.of(9999999999.0).baseUnitMagnitude(),
		// 		Mode.VELOCITY,
		// 		Setpoint.withNeutralSetpoint(),
		// 		Milliseconds.of(50.0),
		// 		driver.x(),
		// 		driver.y());

		// driver.a().onTrue(Shooter.mInstance.setpointCommand(Shooter.FERRY));

		// driver.povUp().onTrue(Commands.runOnce(() -> Shooting.mInstance.setTest(true)));
		// driver.povRight().onTrue(Commands.runOnce(() -> Shooting.mInstance.setTest(false)));
	}

	public void jogControls() {

		ControllerUtil.bindJog(
				IntakeDeploy.mInstance,
				Volts.of(0.01).baseUnitMagnitude(),
				Volts.of(0.0).baseUnitMagnitude(),
				Volts.of(12.0).baseUnitMagnitude(),
				Mode.VOLTAGE,
				Setpoint.withNeutralSetpoint(),
				Milliseconds.of(50.0),
				operator.a(),
				operator.b(),
				operator.rightBumper());

		ControllerUtil.bindJog(
				HopperRollers.mInstance,
				Volts.of(0.5).baseUnitMagnitude(),
				Volts.of(0.0).baseUnitMagnitude(),
				Volts.of(12.0).baseUnitMagnitude(),
				Mode.VOLTAGE,
				Setpoint.withNeutralSetpoint(),
				Milliseconds.of(50.0),
				operator.povUp(),
				operator.povDown(),
				operator.leftTrigger());

		ControllerUtil.bindJog(
				TunnelRollers.mInstance,
				Volts.of(0.1).baseUnitMagnitude(),
				Volts.of(0.0).baseUnitMagnitude(),
				Volts.of(12.0).baseUnitMagnitude(),
				Mode.VOLTAGE,
				Setpoint.withNeutralSetpoint(),
				Milliseconds.of(50.0),
				driver.povUp(),
				driver.povDown(),
				driver.leftBumper());

		ControllerUtil.bindJog(
				FeederRollers.mInstance,
				Volts.of(0.1).baseUnitMagnitude(),
				Volts.of(0.0).baseUnitMagnitude(),
				Volts.of(12.0).baseUnitMagnitude(),
				Mode.VOLTAGE,
				Setpoint.withNeutralSetpoint(),
				Milliseconds.of(50.0),
				driver.povLeft(),
				driver.povRight(),
				driver.rightTrigger());

		ControllerUtil.bindJog(
				Shooter.mInstance,
				Volts.of(0.01).baseUnitMagnitude(),
				Volts.of(0.0).baseUnitMagnitude(),
				Volts.of(12.0).baseUnitMagnitude(),
				Mode.VOLTAGE,
				Setpoint.withNeutralSetpoint(),
				Milliseconds.of(50.0),
				driver.a(),
				driver.b(),
				driver.rightBumper());

		ControllerUtil.bindJog(
				Hood.mInstance,
				Volts.of(0.1).baseUnitMagnitude(),
				Volts.of(0.0).baseUnitMagnitude(),
				Volts.of(12.0).baseUnitMagnitude(),
				Mode.VOLTAGE,
				Setpoint.withNeutralSetpoint(),
				Milliseconds.of(50.0),
				driver.x(),
				driver.y(),
				driver.leftBumper());
	}

	public void driverControls() {
		driver.leftTrigger()
				.onTrue(Superstructure.mInstance.intake().onlyWhile(driver.leftTrigger()))
				.onFalse(Superstructure.mInstance.idleIntake())
				.whileTrue(Commands.sequence(
								Commands.waitSeconds(0.7),
								Commands.deadline(
										Commands.waitUntil((() -> IntakeDeploy.mInstance.nearPosition(
												IntakeDeployConstants.kDeployedAngle, Units.Degrees.of(3.0)))),
										rumbleCommand(Seconds.of(Double.MAX_VALUE))))
						.finallyDo(() -> setRumble(false)));

		driver.rightTrigger(0.1)
				.and(overrideTrigger.negate())
				.onTrue(Commands.parallel(
						Superstructure.mInstance.shoot(), Shooting.mInstance.followShootOnTheMoveRequest()));

		driver.rightTrigger(0.1).and(overrideTrigger).onTrue(Superstructure.mInstance.trenchShot());
		driver.rightTrigger(0.1)
				.onFalse(Commands.either(
								Superstructure.mInstance.stopShooting(),
								Superstructure.mInstance.idle(),
								() -> driver.leftTrigger().getAsBoolean())
						.alongWith(Hood.mInstance.setpointCommand(Hood.STOW)));

		driver.leftBumper().onTrue(Superstructure.mInstance.idle());
		driver.povRight().onTrue(Superstructure.mInstance.contractHopper());
		driver.povLeft().onTrue(Superstructure.mInstance.expandHopper());
		driver.x().onTrue(Superstructure.mInstance.prepClimb());

		driver.b().onTrue(Superstructure.mInstance.stowClimb());
		driver.a().onTrue(Superstructure.mInstance.tuck());

		driver.y().and(overrideTrigger.negate()).onTrue(Superstructure.mInstance.fenderShot());
		driver.y().and(overrideTrigger).onTrue(Superstructure.mInstance.towerShot());
		driver.y()
				.onFalse(Commands.either(
						Superstructure.mInstance.stopShooting(),
						Superstructure.mInstance.idle(),
						() -> driver.leftTrigger().getAsBoolean()));

		driver.povUp().onTrue(Superstructure.mInstance.stow());
		driver.povDown()
				.onTrue(Commands.either(
						Climber.mInstance.setpointCommand(Climber.TRENCH),
						Superstructure.mInstance.trenchContract(),
						overrideTrigger.negate()));

		operator.povUp().onTrue(Commands.runOnce(() -> {
			Setpoint newSetpoint =
					Setpoint.withMotionMagicSetpoint(ClimberConstants.converter.toAngle(ClimberConstants.converter
							.toDistance(Units.Radians.of(Climber.mInstance.getSetpoint().baseUnits))
							.plus(ClimberConstants.kJogDelta)));
			Climber.mInstance.applySetpoint(newSetpoint);
		}));

		operator.povLeft().onTrue(Commands.runOnce(() -> {
			Setpoint newSetpoint =
					Setpoint.withMotionMagicSetpoint(ClimberConstants.converter.toAngle(ClimberConstants.converter
							.toDistance(Units.Radians.of(Climber.mInstance.getSetpoint().baseUnits))
							.minus(ClimberConstants.kJogDelta)));
			Climber.mInstance.applySetpoint(newSetpoint);
		}));

		// operator.b().onTrue(Superstructure.mInstance.intakeExhaustCommand());
		operator.y().onTrue(Superstructure.mInstance.hoodForceHomeCommand());
		operator.a().onTrue(Superstructure.mInstance.intakeForceHomeCommand());
		operator.x().onTrue(Superstructure.mInstance.climbForceHomeCommand());
		operator.leftBumper().onTrue(Superstructure.mInstance.trimUp());
		operator.leftTrigger().onTrue(Superstructure.mInstance.trimDown());
		operator.rightBumper()
				.whileTrue(Superstructure.mInstance.slowShoot())
				.onFalse(Superstructure.mInstance.stopCommand());

		operator.povDown().onTrue(Commands.runOnce(() -> {
			Setpoint newSetpoint = Setpoint.withPositionSetpoint(
					Radians.of(IntakeDeploy.mInstance.getSetpoint().baseUnits).plus(Degrees.of(1.0)));
			IntakeDeploy.mInstance.applySetpoint(newSetpoint);
		}));

		operator.povRight().onTrue(Commands.runOnce(() -> {
			Setpoint newSetpoint = Setpoint.withPositionSetpoint(
					Radians.of(IntakeDeploy.mInstance.getSetpoint().baseUnits).plus(Degrees.of(-1.0)));
			IntakeDeploy.mInstance.applySetpoint(newSetpoint);
		}));
	}

	public void shotTuningControls() {
		operator.povUp()
				.onTrue(Commands.runOnce(() -> Superstructure.mInstance.jogShooter(RPM.of(50.0)))
						.ignoringDisable(true));
		operator.povDown()
				.onTrue(Commands.runOnce(() ->
								Superstructure.mInstance.jogShooter(RPM.of(50.0).unaryMinus()))
						.ignoringDisable(true));

		operator.povRight()
				.onTrue(Commands.runOnce(() -> Superstructure.mInstance.jogHood(Degrees.of(1.0)))
						.ignoringDisable(true));
		operator.povLeft()
				.onTrue(Commands.runOnce(() ->
								Superstructure.mInstance.jogHood(Degrees.of(1.0).unaryMinus()))
						.ignoringDisable(true));

		operator.leftBumper()
				.onTrue(Commands.runOnce(() -> Superstructure.mInstance.jogTunnelRoller(RPM.of(50.0)))
						.ignoringDisable(true));
		operator.rightBumper()
				.onTrue(Commands.runOnce(() -> Superstructure.mInstance.jogTunnelRoller(
								RPM.of(50.0).unaryMinus()))
						.ignoringDisable(true));

		operator.leftTrigger(0.1)
				.onTrue(Commands.runOnce(() -> Superstructure.mInstance.jogShooterOverrun(Percent.of(2.0)))
						.ignoringDisable(true));

		operator.rightTrigger(0.1)
				.onTrue(Commands.runOnce(() -> Superstructure.mInstance.jogShooterOverrun(
								Percent.of(2.0).unaryMinus()))
						.ignoringDisable(true));

		// Used when tuning to get a single signal of what our distance from whatever the target is

		// operator.a()
		// 		.onTrue(Commands.runOnce(() -> SmartDashboard.putNumber(
		// 						"Dist to hub",
		// 						Drive.mInstance
		// 								.getPose()
		// 								.getTranslation()
		// 								.getDistance(FieldLayout.handleAllianceFlip(FieldLayout.kBlueHub, false))))
		// 				.ignoringDisable(true));

		operator.a()
				.onTrue(Commands.runOnce(() -> SmartDashboard.putNumber(
						"Dist to Ferry",
						Drive.mInstance
								.getPose()
								.getTranslation()
								.getDistance(FieldLayout.handleAllianceFlip(FieldLayout.kBlueLeftFerry, true)))));
	}

	public Command rumbleCommand(Time duration) {
		return Commands.sequence(
						Commands.runOnce(() -> {
							setRumble(true);
						}),
						Commands.waitSeconds(duration.in(Units.Seconds)),
						Commands.runOnce(() -> {
							setRumble(false);
						}))
				.finallyDo(() -> {
					setRumble(false);
					;
				})
				.withName("Rumble");
	}

	public void setRumble(boolean on) {
		ControlBoardConstants.mDriverController.getHID().setRumble(RumbleType.kBothRumble, on ? 1.0 : 0.0);
	}
}
