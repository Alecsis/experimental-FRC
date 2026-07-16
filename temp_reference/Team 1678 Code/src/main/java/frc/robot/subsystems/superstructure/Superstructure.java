package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.MotorIO.Mode;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.trigger.CANRangeSensorIO;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout;
import frc.lib.util.FieldLayout.ClimbLocation;
import frc.lib.util.MathHelpers;
import frc.robot.RobotConstants;
import frc.robot.commands.FollowSyncedPIDToPose;
import frc.robot.controlboard.ControlBoard;
import frc.robot.shooting.Shooting;
import frc.robot.shooting.Shooting.ShootingState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.feederrollers.FeederRollers;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hopperrollers.HopperRollers;
import frc.robot.subsystems.intakedeploy.IntakeDeploy;
import frc.robot.subsystems.intakedeploy.IntakeDeployConstants;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.superstructure.SuperstructureConstants.AutoTuckConstants;
import frc.robot.subsystems.superstructure.SuperstructureConstants.CANRanges.BallTunnel;
import frc.robot.subsystems.superstructure.SuperstructureConstants.CANRanges.Hopper;
import frc.robot.subsystems.superstructure.SuperstructureConstants.ForceHomeConstants;
import frc.robot.subsystems.tunnelrollers.TunnelRollerConstants;
import frc.robot.subsystems.tunnelrollers.TunnelRollers;
import frc.robot.subsystems.vision.Cameras;
import frc.robot.subsystems.vision.CamerasConstants;
import java.util.Arrays;
import java.util.Collections;
import java.util.Set;

public class Superstructure extends SubsystemBase {
	public static final Superstructure mInstance = new Superstructure();

	private boolean driveReady = false;
	private boolean superstructureDone = false;

	private boolean trenchContract = false;

	private boolean isPathFollowing = false;

	private State state = State.IDLE;

	private boolean isNearingTrench = false;

	private final Debouncer intakeForceHomeDebouncer =
			new Debouncer(ForceHomeConstants.INTAKE_FORCE_DEBOUNCE.in(Seconds));
	private final Debouncer climbForceHomeDebouncer =
			new Debouncer(ForceHomeConstants.INTAKE_FORCE_DEBOUNCE.in(Seconds));
	private final Debouncer hoodForceHomeDebouncer =
			new Debouncer(ForceHomeConstants.INTAKE_FORCE_DEBOUNCE.in(Seconds));

	private final Debouncer autoTuckTrenchDebouncer =
			new Debouncer(AutoTuckConstants.DISTANCE_FROM_TRENCH_DIRECTION_VELOCITY_DEBOUNCE_TIME.in(Seconds));

	private final Command contractCommand = Commands.parallel(
					Hood.mInstance.setpointCommandWithWait(Hood.STOW),
					Climber.mInstance.setpointCommandWithWait(Climber.STOW))
			.withName("Contract For Trench");

	private final Debouncer shotVerifiedDebouncer =
			new Debouncer(Milliseconds.of(40.0).in(Seconds), DebounceType.kFalling);

	private final Debouncer shooterVelocityReachedDebouncer =
			new Debouncer(Milliseconds.of(25.0).in(Seconds));

	private Dimensionless overrunShooterVelocity = Percent.zero();
	public TargetPositionInputs positionInputs = new TargetPositionInputs();

	public class TargetPositionInputs implements Sendable {
		public AngularVelocity shooterTarget = RPM.of(1000.0);
		public Angle hoodTarget = Degrees.of(18.0);
		public AngularVelocity feederTarget = RPM.of(250);
		public AngularVelocity tunnelRollerTarget = RPM.zero();
		public Dimensionless shooterOverrunTarget = SuperstructureConstants.SHOOTER_OVERRUN_PERCENT;
		public Dimensionless shooterOverrunTargetWhilePreload =
				SuperstructureConstants.SHOOTER_OVERRUN_PERCENT_WHILE_PRELOADED;
		public Time shooterOverrunTargetVolleyDuration = SuperstructureConstants.FIRST_VOLLEY_FORCED_TIME;
		public AngularVelocity trim = RPM.zero();

		public Setpoint getShooterSetpoint() {
			SmartDashboard.putNumber("Current Shooter Setpoint", shooterTarget.in(RPM));
			return Setpoint.withVelocitySetpoint(shooterTarget);
		}

		public Setpoint getHoodSetpoint() {
			return Setpoint.withMotionMagicSetpoint(hoodTarget);
		}

		public Setpoint getFeederSetpoint() {
			return Setpoint.withVelocitySetpoint(feederTarget);
		}

		public Setpoint getTunnelRollerSetpoint() {
			return Setpoint.withVelocitySetpoint(tunnelRollerTarget);
		}

		@Override
		public void initSendable(SendableBuilder builder) {
			builder.addDoubleProperty("Inputs/Shooter Target RPM", () -> shooterTarget.in(RPM), null);
			builder.addDoubleProperty("Inputs/Hood Target Degrees", () -> hoodTarget.in(Degrees), null);
			builder.addDoubleProperty("Inputs/Tunnel Rollers Target RPM", () -> tunnelRollerTarget.in(RPM), null);
			builder.addDoubleProperty("Inputs/Feeder Target RPM", () -> feederTarget.in(RPM), null);
			builder.addDoubleProperty(
					"Inputs/Overramp/Shooter Over Ramp Target RPM", () -> shooterOverrunTarget.in(Percent), null);
			builder.addDoubleProperty(
					"Inputs/Overramp/Duration", () -> shooterOverrunTargetVolleyDuration.in(Seconds), null);
			builder.addBooleanProperty(
					"CANRanges/Ball Tunnel",
					() -> SuperstructureConstants.CANRanges.BallTunnel.CAN.getDebounced(),
					null);
			builder.addBooleanProperty(
					"CANRanges/Hopper", () -> SuperstructureConstants.CANRanges.Hopper.CAN.getDebounced(), null);
			SuperstructureConstants.CANRanges.BallTunnel.CAN.initSendable(builder);
			SuperstructureConstants.CANRanges.Hopper.CAN.initSendable(builder);
			builder.addDoubleProperty("Shooter Trim Value", () -> positionInputs.trim.in(RPM), null);
		}
	}

	public void jogShooterOverrun(Dimensionless shooterOverrunInterval) {
		positionInputs.shooterOverrunTarget = positionInputs.shooterOverrunTarget.plus(shooterOverrunInterval);
	}

	public void jogShooterOverrunVolleyDuration(Time shooterOverrunTargetVolleyDuration) {
		positionInputs.shooterOverrunTargetVolleyDuration =
				positionInputs.shooterOverrunTargetVolleyDuration.plus(shooterOverrunTargetVolleyDuration);
	}

	public void jogShooter(AngularVelocity shooterInterval) {
		positionInputs.shooterTarget = positionInputs.shooterTarget.plus(shooterInterval);
	}

	public void jogHood(Angle hoodInterval) {
		positionInputs.hoodTarget = positionInputs.hoodTarget.plus(hoodInterval);
	}

	public void jogFeeder(AngularVelocity feederInterval) {
		positionInputs.feederTarget = positionInputs.feederTarget.plus(feederInterval);
	}

	public void jogTunnelRoller(AngularVelocity interval) {
		positionInputs.tunnelRollerTarget = positionInputs.tunnelRollerTarget.plus(interval);
	}

	public Pose2d getLookAheadPose() {
		return Drive.mInstance.getLookAheadPose(SuperstructureConstants.LOOK_AHEAD_TIME);
	}

	public void outputTelemetry() {
		LogUtil.log("Superstructure/Drive Look Ahead", getLookAheadPose());
	}

	@Override
	public void periodic() {
		outputTelemetry();
		updateIsNearingTrench();
		SuperstructureConstants.CANRanges.update();
	}

	public void updateIsNearingTrench() {
		if (Drive.mInstance == null) {
			isNearingTrench = false;
			return;
		}
		ChassisSpeeds velocities = ChassisSpeeds.fromRobotRelativeSpeeds(
				Drive.mInstance.getState().Speeds, Drive.mInstance.getPose().getRotation());
		Translation2d v = new Translation2d(velocities.vxMetersPerSecond, velocities.vyMetersPerSecond);
		Translation2d trench = handleTrenchTranslation();
		Translation2d trenchToRobot = trench.minus(Drive.mInstance.getPose().getTranslation());
		boolean nextStatus;
		if (getDistanceToClosestTrench().lte(AutoTuckConstants.DISTANCE_FROM_TRENCH_TO_STOW)
				&& v.dot(trenchToRobot) > 0.0
				&& Math.abs(v.getX()) > 0) {
			nextStatus = true;
		} else {
			nextStatus = false;
		}
		isNearingTrench = autoTuckTrenchDebouncer.calculate(nextStatus);
	}

	public boolean hasShooterReachedVelo(AngularVelocity velocity) {
		return shooterVelocityReachedDebouncer.calculate(Shooter.mInstance.nearVelocity(velocity)
				|| Shooter.mInstance.getVelocity().gte(velocity));
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);

		builder.addBooleanProperty(
				"Force Home/Climber At Home",
				() -> {
					Angle climberAngularVelocity = BaseUnits.AngleUnit.of(
							Climber.mInstance.getVelocity().in(BaseUnits.AngleUnit.per(BaseUnits.TimeUnit)));
					LinearVelocity climberLinearVelocity = ClimberConstants.converter
							.toDistance(climberAngularVelocity)
							.per(BaseUnits.TimeUnit);
					return climbForceHomeDebouncer.calculate(climberLinearVelocity.abs(InchesPerSecond)
									<= ForceHomeConstants.CLIMBER_MIN_HOME_VELOCITY.in(InchesPerSecond)
							&& Climber.mInstance.getSetpoint().mode.isVoltageControl());
				},
				null);
		builder.addBooleanProperty(
				"Force Home/Hood At Home",
				() -> hoodForceHomeDebouncer.calculate(
						Hood.mInstance.getVelocity().abs(DegreesPerSecond)
										<= ForceHomeConstants.HOOD_MIN_HOME_VELOCITY.in(DegreesPerSecond)
								&& Hood.mInstance.getSetpoint().mode == Mode.VOLTAGE),
				null);
		builder.addBooleanProperty(
				"Force Home/Intake At Home",
				() -> intakeForceHomeDebouncer.calculate(
						IntakeDeploy.mInstance.getVelocity().abs(DegreesPerSecond)
										<= ForceHomeConstants.INTAKE_MIN_HOME_VELOCITY.in(DegreesPerSecond)
								&& IntakeDeploy.mInstance.getSetpoint().mode == Mode.VOLTAGE),
				null);
		builder.addBooleanProperty("Drive Ready", () -> driveReady, null);
		builder.addDoubleProperty(
				"DistanceToHubMeters",
				() -> Drive.mInstance
						.getPose()
						.getTranslation()
						.getDistance(
								FieldLayout.handleAllianceFlip(FieldLayout.kBlueHub, RobotConstants.isRedAlliance)),
				null);
		builder.addBooleanProperty("Superstructure Done", () -> superstructureDone, null);
		builder.addBooleanProperty("Trench/Nearing", () -> isNearingTrench, null);
		builder.addBooleanProperty("Trench/Contracted", () -> trenchContract, null);
		builder.addStringProperty("State", () -> state.toString(), null);
		builder.addDoubleProperty("Shooter Overrun RPM", () -> overrunShooterVelocity.in(Percent), null);

		builder.addBooleanProperty("Spun Up", () -> spunUp(), null);

		builder.addDoubleProperty("Battery Voltage", () -> RobotController.getBatteryVoltage(), null);
		builder.addBooleanProperty("Can Fire", () -> canFire(), null);
		builder.addBooleanProperty(
				"Failed CANRanges/Ball Tunnel Failed", () -> BallTunnel.CAN instanceof CANRangeSensorIO, null);
		builder.addBooleanProperty(
				"Failed CANRanges/Hopper Failed", () -> Hopper.CAN instanceof CANRangeSensorIO, null);
		positionInputs.initSendable(builder);
	}

	public Command intakeForceHomeCommand() {
		return Commands.sequence(
						Commands.deadline(
								Commands.waitUntil(() -> intakeForceHomeDebouncer.calculate(
										IntakeDeploy.mInstance.getVelocity().abs(DegreesPerSecond)
														<= ForceHomeConstants.INTAKE_MIN_HOME_VELOCITY.in(
																DegreesPerSecond)
												&& IntakeDeploy.mInstance.getSetpoint().mode == Mode.VOLTAGE)),
								IntakeDeploy.mInstance.setpointCommand(Setpoint.withVoltageSetpoint(Volts.of(5.0)))),
						IntakeDeploy.mInstance.setpointCommand(Setpoint.withNeutralSetpoint()),
						IntakeDeploy.mInstance.runOnce(
								() -> IntakeDeploy.mInstance.setCurrentPosition(IntakeDeployConstants.kStowedAngle)))
				.ignoringDisable(true)
				.withName("Force Intake Home");
	}

	public Command climbForceHomeCommand() {
		return Commands.sequence(
						Commands.deadline(
								Commands.waitUntil(() -> {
									Angle climberAngularVelocity = BaseUnits.AngleUnit.of(Climber.mInstance
											.getVelocity()
											.in(BaseUnits.AngleUnit.per(BaseUnits.TimeUnit)));
									LinearVelocity climberLinearVelocity = ClimberConstants.converter
											.toDistance(climberAngularVelocity)
											.per(BaseUnits.TimeUnit);
									SmartDashboard.putNumber(
											"Climber Velo inches ", climberLinearVelocity.abs(InchesPerSecond));
									return climbForceHomeDebouncer.calculate(climberLinearVelocity.abs(InchesPerSecond)
													<= ForceHomeConstants.CLIMBER_MIN_HOME_VELOCITY.in(InchesPerSecond)
											&& Climber.mInstance
													.getSetpoint()
													.mode
													.isVoltageControl());
								}),
								Climber.mInstance.setpointCommand(Setpoint.withVoltageSetpoint(
										Volts.of(5.0).unaryMinus()))),
						Climber.mInstance.setpointCommand(Setpoint.withNeutralSetpoint()),
						Climber.mInstance
								.runOnce(() -> Climber.mInstance.setCurrentPosition(
										ClimberConstants.converter.toAngle(ClimberConstants.kStowPosition)))
								.withName("Force Climb Home"))
				.ignoringDisable(true);
	}

	public Command hoodForceHomeCommand() {
		return Commands.sequence(
						Commands.deadline(
								Commands.waitUntil(() -> hoodForceHomeDebouncer.calculate(
										Hood.mInstance.getVelocity().abs(DegreesPerSecond)
														<= ForceHomeConstants.HOOD_MIN_HOME_VELOCITY.in(
																DegreesPerSecond)
												&& Hood.mInstance.getSetpoint().mode == Mode.VOLTAGE)),
								Hood.mInstance.setpointCommand(Setpoint.withVoltageSetpoint(
										Volts.of(5.0).unaryMinus()))),
						Hood.mInstance.setpointCommand(Setpoint.withNeutralSetpoint()),
						Hood.mInstance.runOnce(() -> Hood.mInstance.setCurrentPosition(HoodConstants.kMinAngle)))
				.withName("Force Intake Home")
				.ignoringDisable(true);
	}

	/*
	 * Stops coral rollers, coral indexer, and algae rollers
	 */
	public Command idleIntake() {
		return Commands.parallel(IntakeRollers.mInstance.setpointCommand(IntakeRollers.IDLE))
				.withName("Idle Intake");
	}

	public Command stopCommand() {
		return Commands.parallel(
				IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.STOWED_SETPOINT),
				Climber.mInstance.setpointCommand(Climber.STOW),
				Hood.mInstance.setpointCommand(Hood.STOW),
				Commands.parallel(
						HopperRollers.mInstance.setpointCommand(HopperRollers.IDLE),
						Shooter.mInstance.setpointCommand(Shooter.IDLE),
						FeederRollers.mInstance.setpointCommand(FeederRollers.IDLE),
						TunnelRollers.mInstance.setpointCommand(TunnelRollers.IDLE)));
	}

	public Command intakeExhaustCommand() {
		return Commands.sequence(
				IntakeDeploy.mInstance.setpointCommandWithWait(IntakeDeploy.DEPLOYED_SETPOINT),
				Commands.parallel(
						IntakeRollers.mInstance.setpointCommand(IntakeRollers.OUTTAKE),
						HopperRollers.mInstance.setpointCommand(HopperRollers.OUTTAKE),
						FeederRollers.mInstance.setpointCommand(FeederRollers.REVERSE)));
	}

	public Command idle() {
		return Commands.parallel(
						IntakeRollers.mInstance.setpointCommand(IntakeRollers.IDLE),
						HopperRollers.mInstance.setpointCommand(HopperRollers.IDLE),
						TunnelRollers.mInstance.setpointCommand(TunnelRollers.IDLE),
						FeederRollers.mInstance.setpointCommand(FeederRollers.SLOW_REVERSE),
						shooterRampDown(),
						IntakeDeploy.mInstance.setpointCommand(Setpoint.withNeutralSetpoint()))
				.withName("Idle");
	}

	public Command tuck() {
		return Commands.parallel(
				IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.STOWED_SETPOINT),
				Climber.mInstance.setpointCommand(Climber.STOW),
				Hood.mInstance.setpointCommand(Hood.STOW));
	}

	public Command tuckWithoutIntakeStow() {
		return Commands.parallel(
				Climber.mInstance.setpointCommand(Climber.STOW), Hood.mInstance.setpointCommand(Hood.STOW));
	}

	public Command intake() {
		return Commands.sequence(
						Commands.parallel(
								IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.DEPLOYED_SETPOINT),
								FeederRollers.mInstance.setpointCommand(FeederRollers.IDLE),
								IntakeRollers.mInstance.setpointCommand(IntakeRollers.INTAKE)),
						Commands.waitUntil(() -> prefillReady() && !prefillDone()),
						prefillBallTunnel())
				.withName("Intake");
	}

	/**
	 * Command to prefill the ball tunnel allowing 8-10 more balls in the hopper
	 * @return The command to prefill the tunnel
	 * @apiNote This doesn't start up the hopper floor which needs to be called separately
	 */
	public Command prefillBallTunnel() {
		return Commands.sequence(
						TunnelRollers.mInstance.setpointCommand(TunnelRollers.PRELOAD_SPEED),
						HopperRollers.mInstance.setpointCommand(HopperRollers.PRELOAD_SPEED),
						FeederRollers.mInstance.setpointCommand(FeederRollers.SLOW_FEED_VOLTAGE),
						Commands.waitUntil(() -> prefillDone()),
						TunnelRollers.mInstance.setpointCommand(TunnelRollers.IDLE),
						HopperRollers.mInstance.setpointCommand(HopperRollers.IDLE),
						FeederRollers.mInstance.setpointCommand(FeederRollers.IDLE))
				.handleInterrupt(() -> {
					TunnelRollers.mInstance.applySetpoint(TunnelRollers.IDLE);
					HopperRollers.mInstance.applySetpoint(HopperRollers.IDLE);
					FeederRollers.mInstance.applySetpoint(FeederRollers.IDLE);
				})
				.withName("Prefill tunnel");
	}

	public Command stopShooting() {
		return Commands.parallel(
						HopperRollers.mInstance.setpointCommand(HopperRollers.IDLE),
						TunnelRollers.mInstance.setpointCommand(TunnelRollers.IDLE),
						FeederRollers.mInstance.setpointCommand(FeederRollers.IDLE),
						shooterRampDown())
				.withName("StopShooting");
	}

	public Command intakeWithExtensionCheck() {
		return Commands.sequence(
						Climber.mInstance.setpointCommand(Climber.CLEAR_INTAKE),
						Commands.parallel(
								IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.DEPLOYED_SETPOINT),
								FeederRollers.mInstance.setpointCommand(FeederRollers.IDLE)),
						IntakeRollers.mInstance.setpointCommand(IntakeRollers.INTAKE))
				.withName("Intake With Extension Check");
	}

	public Command continousShooting() {
		return Commands.sequence(
				IntakeDeploy.mInstance.setpointCommandWithWait(IntakeDeploy.DEPLOYED_SETPOINT),
				IntakeRollers.mInstance.setpointCommand(IntakeRollers.INTAKE),
				Shooter.mInstance.setpointCommandWithWait(Shooter.FERRY),
				Commands.parallel(
						TunnelRollers.mInstance.setpointCommand(TunnelRollers.UP),
						HopperRollers.mInstance.setpointCommand(HopperRollers.INTAKE)));
	}

	public Command ejectWarmUpCommand() {
		return Commands.parallel(
				Shooter.mInstance.followSetpointCommand(() -> Shooting.mInstance.getLatestShooterSetpoint()),
				Hood.mInstance.followSetpointCommand(() -> Shooting.mInstance.getLatestHoodTargetSetpoint()),
				TunnelRollers.mInstance.followSetpointCommand(() -> Shooting.mInstance.getLatestTunnelSetpoint()));
	}

	public Command shootWithTimeout() {
		Timer autoAlignmentTimer = new Timer();
		return Commands.sequence(
						Commands.runOnce(autoAlignmentTimer::restart),
						FeederRollers.mInstance.setpointCommand(FeederRollers.SLOW_REVERSE),
						Commands.runOnce(() -> overrunShooterVelocity = positionInputs.shooterOverrunTarget),
						Commands.deadline(
								Commands.sequence(
										Commands.waitUntil(() -> spunUp()
												&& Shooting.mInstance.autoShotAlignedOrTimedOut(
														Seconds.of(autoAlignmentTimer.get()))),
										Commands.parallel(
												HopperRollers.mInstance.setpointCommand(HopperRollers.INTAKE),
												FeederRollers.mInstance.setpointCommand(FeederRollers.FEED_VOLTAGE)),
										Commands.waitTime(positionInputs.shooterOverrunTargetVolleyDuration)),
								Shooter.mInstance.followSetpointCommand(
										() -> Setpoint.withVelocitySetpoint(BaseUnits.AngleUnit.per(BaseUnits.TimeUnit)
												.of(Shooting.mInstance.getLatestShooterSetpoint().baseUnits)
												.times(overrunShooterVelocity.plus(Percent.of(100))))),
								Hood.mInstance.followSetpointCommand(
										() -> Shooting.mInstance.getLatestHoodTargetSetpoint()),
								TunnelRollers.mInstance.followSetpointCommand(
										() -> Shooting.mInstance.getLatestTunnelSetpoint())),
						Commands.runOnce(() -> overrunShooterVelocity = Percent.zero()))
				.withName("Shoot In Auto");
	}

	public Command spinUpInAuto() {
		return Commands.sequence(
				FeederRollers.mInstance.setpointCommand(FeederRollers.SLOW_REVERSE),
				Commands.parallel(
						Shooter.mInstance.setpointCommand(Setpoint.withVelocitySetpoint(Units.RPM.of(2700.0))),
						Hood.mInstance.setpointCommand(Setpoint.withMotionMagicSetpoint(Units.Degrees.of(26.0))),
						TunnelRollers.mInstance.setpointCommand(Setpoint.withVelocitySetpoint(Units.RPM.of(1950.0)))));
	}

	public Command spinUpInAuto(AngularVelocity shooterTarget, Angle hoodTarget, AngularVelocity tunnelTarget) {
		return Commands.parallel(
						FeederRollers.mInstance.setpointCommand(FeederRollers.SLOW_REVERSE),
						Shooter.mInstance.setpointCommand(Setpoint.withVelocitySetpoint(shooterTarget)),
						Hood.mInstance.setpointCommand(Setpoint.withMotionMagicSetpoint(hoodTarget)),
						TunnelRollers.mInstance.setpointCommand(Setpoint.withVelocitySetpoint(tunnelTarget)))
				.withName("Auto Marker Spin Up");
	}

	public Command contractClimber() {
		return Climber.mInstance.setpointCommand(Climber.STOW).withName("Auto Marker Contract Hopper");
	}

	public Command stow() {
		return Commands.parallel(
						Climber.mInstance.setpointCommand(Climber.STOW), Hood.mInstance.setpointCommand(Hood.STOW))
				.withName("Auto Marker Stow");
	}

	public Command stowWithWait() {
		return Commands.sequence(
						stow(),
						Commands.waitUntil(() -> Climber.mInstance.nearHomingLocation()
								&& Hood.mInstance.nearPosition(HoodConstants.kMinAngle)))
				.withName("Auto Marker Stow Wait");
	}

	public Command idleTunnelAndIntake() {
		return Commands.sequence(
						Commands.parallel(
								HopperRollers.mInstance.setpointCommand(HopperRollers.SLOW_INTAKE),
								tunnelRampDown(),
								FeederRollers.mInstance.setpointCommand(FeederRollers.SLOW_REVERSE),
								shooterRampDown(),
								IntakeDeploy.mInstance.setpointCommandWithWait(IntakeDeploy.DEPLOYED_SETPOINT)),
						IntakeRollers.mInstance.setpointCommand(IntakeRollers.INTAKE))
				.withName("IdleTunnelAndIntake");
	}

	public Command idleEjectionAndIntake() {
		return Commands.parallel(
				FeederRollers.mInstance.setpointCommand(FeederRollers.SLOW_REVERSE),
				Shooter.mInstance.setpointCommand(Setpoint.withVelocitySetpoint(RPM.of(2000.0))),
				TunnelRollers.mInstance.setpointCommand(Setpoint.withVelocitySetpoint(RPM.of(1500.0))),
				Hood.mInstance.setpointCommand(Hood.STOW));
	}

	public boolean isFerrying() {
		return Shooting.getState() != ShootingState.HUB;
	}

	public boolean isFarFerrying() {
		return Shooting.getState() == ShootingState.LEFT_FAR_ALLIANCE_FERRY
				|| Shooting.getState() == ShootingState.RIGHT_FAR_ALLIANCE_FERRY;
	}

	public Command shoot() {
		Dimensionless overrun =
				(prefillDone()) ? positionInputs.shooterOverrunTargetWhilePreload : positionInputs.shooterOverrunTarget;

		Dimensionless decreasePerIteration = overrun.div(positionInputs.shooterOverrunTargetVolleyDuration.magnitude())
				.times(0.02);

		return Commands.parallel(
						Cameras.mInstance.setStdDevCommand(CamerasConstants.ALIGN_STD_DEVATION),
						Commands.runOnce(() -> {
							FeederRollers.mInstance.applySetpoint(FeederRollers.IDLE);
							overrunShooterVelocity = overrun;
						}),
						Shooter.mInstance.followSetpointCommand(
								() -> Setpoint.withVelocitySetpoint(AngularVelocity.ofBaseUnits(
												// positionInputs.getShooterSetpoint()
												Shooting.mInstance.getLatestShooterSetpoint().baseUnits, RPM)
										.plus(positionInputs.trim))),
						Commands.repeatingSequence(Commands.either(
								Hood.mInstance.setpointCommand(Hood.STOW),
								Hood.mInstance
										.followSetpointCommand(() ->
												// positionInputs.getHoodSetpoint()
												Shooting.mInstance.getLatestHoodTargetSetpoint())
										.onlyWhile(() -> !FieldLayout.isPoseWithinTrench(
												RobotConstants.isRedAlliance, Drive.mInstance.getPose())),
								() -> FieldLayout.isPoseWithinTrench(
										RobotConstants.isRedAlliance, Drive.mInstance.getPose()))),
						Commands.sequence(
								Commands.waitUntil(() -> {
									boolean fireReady = isFerrying() || DriverStation.isAutonomousEnabled()
											? Shooting.mInstance.shotAligned()
											: Shooting.mInstance.shotVerified();
									boolean spunUp =
											isFerrying() ? shooterAndHoodSpunUp(RPM.of(150)) : shooterAndHoodSpunUp();
									return fireReady
											&& spunUp
											&& MathHelpers.hypot(Drive.mInstance.getState().Speeds)
													.lte(
															isFerrying()
																	? SuperstructureConstants.MAX_FOTM_SPEED
																	: SuperstructureConstants.MAX_SOTM_SPEED);
								}),
								Commands.parallel(
										Cameras.mInstance.setStdDevCommand(CamerasConstants.DEFAULT_STD_DEVIATION),
										Commands.either(
												HopperRollers.mInstance.setpointCommand(HopperRollers.SLOW_INTAKE),
												HopperRollers.mInstance.followSetpointCommand(
														() -> shotVerifiedDebouncer.calculate(
																		Shooting.mInstance.shotVerified())
																? HopperRollers.INTAKE
																: HopperRollers.IDLE),
												this::isFarFerrying),
										Commands.either(
												FeederRollers.mInstance.setpointCommand(
														FeederRollers.SLOW_FEED_VOLTAGE),
												FeederRollers.mInstance.followSetpointCommand(
														() -> shotVerifiedDebouncer.calculate(
																		Shooting.mInstance.shotVerified())
																? FeederRollers.FEED_VOLTAGE
																: FeederRollers.IDLE),
												this::isFarFerrying),
										TunnelRollers.mInstance.followSetpointCommand(() -> {
											Setpoint tunnelSetpoint = Shooting.mInstance.getLatestTunnelSetpoint();
											if (isFarFerrying()) {
												return Setpoint.withVelocitySetpointAndVoltageLimit(
														AngularVelocity.ofBaseUnits(tunnelSetpoint.baseUnits, RPM),
														Volts.of(5.0),
														Volts.zero());
											} else if (shotVerifiedDebouncer.calculate(
													Shooting.mInstance.shotVerified())) {
												return Setpoint.withVelocitySetpointAndVoltageLimit(
														AngularVelocity.ofBaseUnits(tunnelSetpoint.baseUnits, RPM),
														Volts.of(12.0),
														Volts.zero());
											} else {
												return TunnelRollers.IDLE;
											}
										}),
										Commands.repeatingSequence(Commands.runOnce(
														() -> overrunShooterVelocity = MathHelpers.clampToZero(
																overrunShooterVelocity.minus(decreasePerIteration))))
												.until(() -> Shooter.mInstance.nearSetpointVelocity(
																Shooting.mInstance.getLatestShooterSetpoint())
														&& overrunShooterVelocity.isEquivalent(Percent.zero())))))
				.finallyDo(() -> Cameras.mInstance.setSTDDeviations(CamerasConstants.DEFAULT_STD_DEVIATION))
				.withName("Shoot");
	}

	public boolean spunUp() {
		boolean shooterReady = Shooter.mInstance.spunUpDebounced();
		boolean tunnelReady = TunnelRollers.mInstance.spunUpDebounced();
		boolean hoodReady = Hood.mInstance.nearPositionSetpoint();
		SmartDashboard.putBoolean("Shooter Spun Up", shooterReady);
		SmartDashboard.putBoolean("Tunnel Spun Up", tunnelReady);
		SmartDashboard.putBoolean("Hood In Position", hoodReady);
		return shooterReady && hoodReady && tunnelReady;
	}

	public boolean shooterAndHoodSpunUp() {
		boolean shooterReady = Shooter.mInstance.spunUpDebounced();
		boolean hoodReady = Hood.mInstance.nearPositionSetpoint();
		SmartDashboard.putBoolean("Shooter Spun Up", shooterReady);
		SmartDashboard.putBoolean("Hood In Position", hoodReady);
		return shooterReady && hoodReady;
	}

	public boolean shooterAndHoodSpunUp(AngularVelocity epsilon) {
		boolean shooterReady = Shooter.mInstance.spunUpDebounced(epsilon);
		boolean hoodReady = Hood.mInstance.nearPositionSetpoint();
		SmartDashboard.putBoolean("Shooter Spun Up", shooterReady);
		SmartDashboard.putBoolean("Hood In Position", hoodReady);
		return shooterReady && hoodReady;
	}

	public Command spinUp() {
		return Commands.parallel(
				FeederRollers.mInstance.setpointCommand((FeederRollers.REVERSE)),
				Shooter.mInstance.followSetpointCommand(() -> positionInputs.getShooterSetpoint()),
				Hood.mInstance.followSetpointCommand(() -> positionInputs.getHoodSetpoint()),
				Commands.sequence(
						Commands.waitSeconds(0.25),
						TunnelRollers.mInstance.followSetpointCommand(() -> positionInputs.getTunnelRollerSetpoint())));
	}

	public Command feed() {
		return Commands.parallel(
				FeederRollers.mInstance.followSetpointCommand(() -> positionInputs.getFeederSetpoint()),
				HopperRollers.mInstance.setpointCommand(HopperRollers.INTAKE));
	}

	public Command fenderShot() {
		return Commands.sequence(
						Shooter.mInstance.setpointCommand(SuperstructureConstants.getShooterSetpointFender()),
						Hood.mInstance.setpointCommand(SuperstructureConstants.getHoodSetpointFender()),
						TunnelRollers.mInstance.setpointCommand(
								SuperstructureConstants.getTunnelRollerSetpointFender()),
						Commands.waitUntil(() -> spunUp()),
						FeederRollers.mInstance.setpointCommand(FeederRollers.FEED_VOLTAGE),
						HopperRollers.mInstance.setpointCommand(HopperRollers.INTAKE))
				.withName("Fender shot");
	}

	public Command towerShot() {
		return Commands.sequence(
						Shooter.mInstance.setpointCommand(SuperstructureConstants.getShooterSetpointTower()),
						Hood.mInstance.setpointCommand(SuperstructureConstants.getHoodSetpointTower()),
						TunnelRollers.mInstance.setpointCommand(SuperstructureConstants.getTunnelRollerSetpointTower()),
						Commands.waitUntil(() -> spunUp()),
						FeederRollers.mInstance.setpointCommand(FeederRollers.FEED_VOLTAGE),
						HopperRollers.mInstance.setpointCommand(HopperRollers.INTAKE))
				.withName("Fender shot");
	}

	public Command trenchShot() {
		return Commands.sequence(
						Shooter.mInstance.setpointCommand(SuperstructureConstants.getShooterSetpointTrench()),
						Hood.mInstance.setpointCommand(SuperstructureConstants.getHoodSetpointTrench()),
						TunnelRollers.mInstance.setpointCommand(
								SuperstructureConstants.getTunnelRollerSetpointTrench()),
						Commands.waitUntil(() -> spunUp()),
						FeederRollers.mInstance.setpointCommand(FeederRollers.FEED_VOLTAGE),
						HopperRollers.mInstance.setpointCommand(HopperRollers.INTAKE))
				.withName("Trench shot");
	}

	public Command slowShoot() {
		return Commands.sequence(
				Commands.parallel(
						Shooter.mInstance.setpointCommand(Setpoint.withVelocitySetpoint(RPM.of(1000))),
						TunnelRollers.mInstance.setpointCommand(Setpoint.withVelocitySetpoint(RPM.of(1000)))),
				Commands.waitSeconds(1.0),
				Commands.parallel(
						FeederRollers.mInstance.setpointCommand(FeederRollers.SLOW_FEED_VOLTAGE),
						HopperRollers.mInstance.setpointCommand(HopperRollers.INTAKE)));
	}

	public Command shootWithMovement() {
		return Commands.parallel(
						Commands.defer(
										() -> Commands.parallel(
												Shooter.mInstance.setpointCommand(
														Shooting.mInstance.getLatestShooterSetpoint()),
												Hood.mInstance.setpointCommand(
														Shooting.mInstance.getLatestHoodTargetSetpoint())),
										Set.of(Shooter.mInstance, Hood.mInstance))
								.repeatedly(),
						enableBallTunnelWithVerification())
				.withName("Shoot and Account for Movement");
	}

	public Command trimUp() {
		return Commands.runOnce(() -> positionInputs.trim = positionInputs.trim.plus(RPM.of(10.0)));
	}

	public Command trimDown() {
		return Commands.runOnce(() -> positionInputs.trim = positionInputs.trim.minus(RPM.of(10.0)));
	}

	public Command enableBallTunnelWithVerification() {
		return Commands.sequence(
				Commands.parallel(
								TunnelRollers.mInstance.setpointCommand(TunnelRollers.IDLE),
								HopperRollers.mInstance.setpointCommand(HopperRollers.IDLE))
						.onlyIf(() -> !Shooting.mInstance.shotVerified()),
				Commands.repeatingSequence(
						Commands.waitUntil(() -> Shooting.mInstance.shotVerified()),
						Commands.parallel(
								TunnelRollers.mInstance.setpointCommand(TunnelRollers.UP),
								HopperRollers.mInstance.setpointCommand(HopperRollers.INTAKE)),
						Commands.waitUntil(() -> !Shooting.mInstance.shotVerified()),
						Commands.parallel(
								TunnelRollers.mInstance.setpointCommand(TunnelRollers.IDLE),
								HopperRollers.mInstance.setpointCommand(HopperRollers.IDLE))));
	}

	public Command trenchContract() {
		return Commands.sequence(
				Commands.parallel(
						IntakeRollers.mInstance.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(2.0))),
						IntakeDeploy.mInstance.setpointCommandWithWait(IntakeDeploy.PARTIAL_IN)),
				IntakeDeploy.mInstance.setpointCommandWithWait(IntakeDeploy.LESS_PARTIAL_IN),
				Commands.parallel(
						IntakeDeploy.mInstance.setpointCommandWithWait(IntakeDeploy.MORE_PARTIAL_IN),
						Climber.mInstance.setpointCommand(Climber.TRENCH)),
				Commands.waitSeconds(0.5),
				IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.STOWED_SETPOINT));
	}

	public Command expandHopper() {
		return Commands.sequence(Climber.mInstance.setpointCommand(Climber.MAX));
	}

	public Command contractHopper() {
		return Commands.sequence(
				Commands.parallel(
						IntakeRollers.mInstance.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(2.0))),
						IntakeDeploy.mInstance.setpointCommandWithWait(IntakeDeploy.MORE_PARTIAL_IN)),
				IntakeDeploy.mInstance.setpointCommandWithWait(IntakeDeploy.LESS_PARTIAL_IN),
				Commands.parallel(
						IntakeDeploy.mInstance.setpointCommandWithWait(IntakeDeploy.MORE_PARTIAL_IN),
						Climber.mInstance.setpointCommand(Climber.CLEAR_INTAKE)),
				Commands.waitSeconds(0.5),
				IntakeDeploy.mInstance.setpointCommandWithWait(IntakeDeploy.PARTIAL_IN),
				IntakeDeploy.mInstance
						.setpointCommand(IntakeDeploy.DEPLOYED_SETPOINT)
						.alongWith(Climber.mInstance.setpointCommand(Climber.STOW)));
	}

	public Command contractHopperWithoutStowing() {
		return Commands.sequence(
				Commands.parallel(
						IntakeRollers.mInstance.setpointCommand(Setpoint.withVoltageSetpoint(Units.Volts.of(2.0))),
						IntakeDeploy.mInstance.setpointCommandWithWait(IntakeDeploy.PARTIAL_IN)),
				IntakeDeploy.mInstance.setpointCommandWithWait(IntakeDeploy.LESS_PARTIAL_IN),
				Commands.parallel(
						IntakeDeploy.mInstance.setpointCommandWithWait(IntakeDeploy.MORE_PARTIAL_IN),
						Climber.mInstance.setpointCommandWithWait(Climber.STOW)));
	}

	/**
	 * Send the climber hooks into the cage and hoist up robot
	 */
	public Command prepClimb() {
		return Commands.parallel(
						IntakeDeploy.mInstance.setpointCommand(IntakeDeploy.DEPLOYED_SETPOINT),
						IntakeRollers.mInstance.setpointCommand(IntakeRollers.IDLE),
						Hood.mInstance.setpointCommand(Hood.STOW),
						HopperRollers.mInstance.setpointCommand(HopperRollers.IDLE),
						Shooter.mInstance.setpointCommand(Shooter.IDLE),
						FeederRollers.mInstance.setpointCommand(FeederRollers.IDLE),
						TunnelRollers.mInstance.setpointCommand(TunnelRollers.IDLE),
						Climber.mInstance.setpointCommand(Climber.MAX))
				.withName("Prep Climb");
	}

	public Command deployIntake() {
		return IntakeDeploy.mInstance
				.setpointCommand(IntakeDeploy.DEPLOYED_SETPOINT)
				.withName("Deploy Intake Without Intaking");
	}

	/**
	 * Hoist up robot
	 */
	public Command stowClimb() {
		return Commands.sequence(Climber.mInstance.setpointCommand(Climber.CLIMB))
				.withName("Stow Climb");
	}

	public Command climb(ClimbLocation closestClimbLocation) {
		return Commands.defer(
				() -> {
					// ClimbLocation closestClimbLocation = getClosestClimbLocation();
					Pose2d climbPose = getClimbPose(closestClimbLocation, true);

					return Commands.either(
							Commands.sequence(
									Superstructure.mInstance.prepClimb(),
									goToSafety(closestClimbLocation, climbPose),
									goToClimb(closestClimbLocation, climbPose)),
							ControlBoard.mInstance.rumbleCommand(
									Units.Seconds.of(1.0)), // rumble controller if not in min dist
							() -> climbPose
											.getTranslation()
											.getDistance(
													Drive.mInstance.getPose().getTranslation())
									< 3.5); // min distance for climbing
				},
				Set.of(Drive.mInstance));
	}

	public Command goToClimb(ClimbLocation closestClimbLocation, Pose2d climbPose) {
		return Commands.sequence(
				new FollowSyncedPIDToPose(
								climbPose,
								closestClimbLocation.xTolerance,
								closestClimbLocation.yTolerance,
								closestClimbLocation.rotationTolerance)
						.onlyWhile(() -> !Superstructure.mInstance.getDriveReady())
						.withDeadline(Commands.waitUntil(() -> (Units.Meters.of(climbPose
												.getMeasureX()
												.minus(Drive.mInstance.getPose().getMeasureX())
												.abs(Units.Meters))
										.lt(closestClimbLocation.xTolerance.plus(Units.Meters.of(0.116))) // 0.04
								&& Units.Meters.of(climbPose
												.getMeasureY()
												.minus(Drive.mInstance.getPose().getMeasureY())
												.abs(Units.Meters))
										.lt(closestClimbLocation.yTolerance.plus(Units.Meters.of(0.055)))))),
				Commands.parallel(
						Drive.mInstance.followSwerveRequestCommand(
								DriveConstants.hitTower, r -> r.withVelocityY(closestClimbLocation.yV)
										.withRotationalRate(Units.RadiansPerSecond.of(-0.1))),
						Commands.sequence(
								Commands.waitTime(Units.Seconds.of(0.25)), Superstructure.mInstance.stowClimb())));
	}

	public Command goToSafety(ClimbLocation closestClimbLocation, Pose2d climbPose) {
		Translation2d safetyTranslation = closestClimbLocation.safetyTranslation;
		Angle safetyAngle = closestClimbLocation.safetyAngle;

		if (RobotConstants.isRedAlliance) {
			safetyTranslation = closestClimbLocation.safetyTranslation.times(-1);
		}

		return new FollowSyncedPIDToPose(
						new Pose2d(
								climbPose.getTranslation().plus(safetyTranslation),
								new Rotation2d(
										climbPose.getRotation().getMeasure().plus(safetyAngle))),
						closestClimbLocation.safetyXTolerance,
						closestClimbLocation.safetyYTolerance,
						closestClimbLocation.safetyRotationTolerance)
				.onlyWhile(() -> !Superstructure.mInstance.getDriveReady());
	}

	public static ClimbLocation getClosestClimbLocation() {
		ClimbLocation targetClimbLocation = ClimbLocation.C; // default option

		java.util.List<Distance> poses = Arrays.asList(
				Units.Meters.of(getClimbPose(ClimbLocation.A, true)
						.getTranslation()
						.getDistance(Drive.mInstance.getPose().getTranslation())),
				Units.Meters.of(getClimbPose(ClimbLocation.C, true)
						.getTranslation()
						.getDistance(Drive.mInstance.getPose().getTranslation())),
				Units.Meters.of(getClimbPose(ClimbLocation.E, true)
						.getTranslation()
						.getDistance(Drive.mInstance.getPose().getTranslation())));

		Distance minDistance = Collections.min(poses);

		int iterable = 0;
		for (Distance pose : poses) {
			if (pose.equals(minDistance)) {
				targetClimbLocation = ClimbLocation.values()[iterable];
				break;
			}
			iterable++;
		}
		return targetClimbLocation;
	}

	public static Pose2d getClimbPose(ClimbLocation location, boolean handleAllianceFlip) {
		Pose2d targetPose;
		switch (location) {
			default:
				targetPose = Pose2d.kZero;
				break;
			case A:
				targetPose = new Pose2d(
						FieldLayout.kBlueTowerRightRung.getMeasureX().plus(Units.Inches.of(-3.025)),
						FieldLayout.kBlueTowerRightRung.getMeasureY().plus(Units.Inches.of(-7.5538669)),
						Rotation2d.kZero);
				break;
			case C:
				targetPose = new Pose2d(
						FieldLayout.kBlueTowerFaceCenter.getMeasureX().plus(Units.Inches.of(3.025)),
						FieldLayout.kBlueTowerFaceCenter.getMeasureY(),
						Rotation2d.kZero);
				break;
			case E:
				targetPose = new Pose2d(
						FieldLayout.kBlueTowerLeftRung.getMeasureX().plus(Units.Inches.of(-3.025)),
						FieldLayout.kBlueTowerLeftRung.getMeasureY().plus(Units.Inches.of(7.0538669)),
						Rotation2d.kZero);
				break;
		}
		if (!handleAllianceFlip) return targetPose;
		targetPose = FieldLayout.handleAllianceFlip(targetPose, RobotConstants.isRedAlliance);
		return targetPose;
	}

	public Command shooterRampDown() {
		return Commands.either(
				Commands.sequence(
						Shooter.mInstance.setpointCommand(Shooter.IDLE),
						Commands.waitUntil(() -> Shooter.mInstance.nearVelocity(ShooterConstants.kSlow)),
						Shooter.mInstance.setpointCommand(Shooter.SLOW)),
				Shooter.mInstance.setpointCommand(Shooter.SLOW),
				() -> Shooter.mInstance.getVelocity().gte(ShooterConstants.kSlow.times(1.25)));
	}

	public Command tunnelRampDown() {
		return Commands.either(
				Commands.sequence(
						TunnelRollers.mInstance.setpointCommand(TunnelRollers.IDLE),
						Commands.waitUntil(() -> TunnelRollers.mInstance.nearVelocity(TunnelRollerConstants.slowSpeed)),
						TunnelRollers.mInstance.setpointCommand(TunnelRollers.SLOW)),
				TunnelRollers.mInstance.setpointCommand(TunnelRollers.SLOW),
				() -> TunnelRollers.mInstance.getVelocity().gte(TunnelRollerConstants.slowSpeed.times(1.25)));
	}

	// public Command waitToStartScoreSequence() {
	// 	return Commands.waitUntil(() -> endEffectorCoralBreak.getDebounced() && readyToRaiseElevator)
	// 			.withName("Wait To Start Score Sequence");
	// }

	public enum State {
		IDLE,
		OUTTAKE,
		SHOOT
	}

	public Command setState(State state) {
		return Commands.runOnce(() -> this.state = state);
	}

	public State getState() {
		return state;
	}

	public void setDriveReady(boolean valToSet) {
		driveReady = valToSet;
	}

	public void setSuperstructureDone(boolean valToSet) {
		superstructureDone = valToSet;
	}

	public boolean getDriveReady() {
		return driveReady;
	}

	public boolean getSuperstructureDone() {
		return superstructureDone;
	}

	public void setPathFollowing(boolean isFollowing) {
		isPathFollowing = isFollowing;
	}

	public Command contractForTrenchCommand() {
		return contractCommand;
	}

	public Command unContrackCommand() {
		return Commands.defer(
				() -> {
					if (CommandScheduler.getInstance().isScheduled(contractCommand)) {
						return Commands.runOnce(
								() -> {
									CommandScheduler.getInstance().cancel(contractCommand);
								},
								Climber.mInstance,
								Hood.mInstance);
					}
					return Commands.none();
				},
				Set.of());
	}

	public boolean getNearingTrench() {
		return isNearingTrench;
	}

	public Translation2d handleTrenchTranslation() {
		Translation2d trench = new Translation2d(
				(getLookAheadPose().getMeasureX().lte(FieldLayout.kFieldLength.div(2.0)))
						? FieldLayout.kBlueLeftTrenchX
						: FieldLayout.flipAcrossX(FieldLayout.kBlueLeftTrenchX),
				(getLookAheadPose().getMeasureY().gte(FieldLayout.kFieldWidth.div(2.0)))
						? FieldLayout.kBlueLeftTrenchY
						: FieldLayout.flipAcrossY(FieldLayout.kBlueLeftTrenchY));

		return trench;
	}

	private Distance getDistanceToClosestTrench() {
		if (Drive.mInstance == null) return Meters.of(9.0);
		Translation2d trench = handleTrenchTranslation();
		return Meters.of(getLookAheadPose().getTranslation().getDistance(trench));
	}

	public boolean prefillReady() {
		return SuperstructureConstants.CANRanges.Hopper.CAN.getDebounced();
	}

	public boolean prefillDone() {
		return SuperstructureConstants.CANRanges.BallTunnel.CAN.getDebounced();
	}

	public boolean canFire() {
		return Shooting.mInstance.shotVerified() && spunUp();
	}
}
