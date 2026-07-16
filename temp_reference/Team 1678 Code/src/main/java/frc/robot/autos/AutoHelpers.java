package frc.robot.autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.Stopwatch;
import frc.robot.shooting.Shooting;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feederrollers.FeederRollers;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopperrollers.HopperRollers;
import frc.robot.subsystems.intakedeploy.IntakeDeploy;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;

public class AutoHelpers {
	public static Stopwatch stopwatch = new Stopwatch();

	public static Command waitForAutoAlignedOrTimeout() {
		Timer autoAlignmentTimer = new Timer();
		return Commands.sequence(
				Commands.runOnce(autoAlignmentTimer::restart),
				Commands.waitUntil(
						() -> Shooting.mInstance.autoShotAlignedOrTimedOut(Seconds.of(autoAlignmentTimer.get()))));
	}

	public static void bindEventMarkers(AutoFactory mAutoFactory) {
		Superstructure superstructure = Superstructure.mInstance;

		mAutoFactory.bind("intake", Commands.runOnce(() -> {
			IntakeDeploy.mInstance.applySetpoint(IntakeDeploy.DEPLOYED_SETPOINT);
			FeederRollers.mInstance.applySetpoint(FeederRollers.IDLE);
			HopperRollers.mInstance.applySetpoint(HopperRollers.IDLE);
			IntakeRollers.mInstance.applySetpoint(IntakeRollers.INTAKE);
		}));
		mAutoFactory.bind("spinUp", Commands.runOnce(() -> {
			Shooter.mInstance.applySetpoint(Setpoint.withVelocitySetpoint(Units.RPM.of(1900.0)));
			Hood.mInstance.applySetpoint(Setpoint.withMotionMagicSetpoint(Degrees.of(19.5)));
		}));
		mAutoFactory.bind("expandHopper", Commands.runOnce(() -> Climber.mInstance.applySetpoint(Climber.MAX)));
	}

	public static String getName(AutoModeSelector mAutoModeSelector) {
		return mAutoModeSelector.getSelectedCommand().getName();
	}

	public static Command resetPoseIfWithoutEstimate(Pose2d pose) {
		return Commands.runOnce(() -> Drive.mInstance.resetPose(pose));
	}

	public static Command fireCommand(Time shootTimeout, Time timeToStartContract) {
		return Commands.sequence(
				Commands.parallel(
								Commands.parallel(
										Shooting.mInstance.followShootOnTheMoveRequest(),
										Superstructure.mInstance.shoot()),
								Commands.sequence(
										Commands.waitUntil(() -> Superstructure.mInstance.spunUp()),
										Commands.waitSeconds(timeToStartContract.in(Seconds)),
										Superstructure.mInstance.contractHopper()))
						.andThen(Commands.waitTime(shootTimeout))
						.withTimeout(shootTimeout),
				Commands.parallel(
						IntakeDeploy.mInstance.setpointCommandWithWait(IntakeDeploy.DEPLOYED_SETPOINT),
						Hood.mInstance.setpointCommandWithWait(Hood.STOW),
						Climber.mInstance.setpointCommand(Climber.STOW)));
	}

	public static Command fireCommandWithoutDrop(Time shootTimeout, Time timeToStartContract) {
		return Commands.sequence(Commands.parallel(
						Commands.parallel(
								Shooting.mInstance.followShootOnTheMoveRequest(), Superstructure.mInstance.shoot()),
						Commands.sequence(
								Commands.waitUntil(() -> Superstructure.mInstance.spunUp()),
								Commands.waitSeconds(timeToStartContract.in(Seconds)),
								Superstructure.mInstance.contractHopper()))
				.andThen(Commands.waitTime(shootTimeout))
				.withTimeout(shootTimeout));
	}

	public static boolean isInsideNeutralZone(Pose2d pose) {
		boolean inMidfieldWindow = pose.getMeasureX().gte(AutoConstants.MIDFIELD_CONTACT_WINDOW_MIN_X)
				&& pose.getMeasureX().lte(AutoConstants.MIDFIELD_CONTACT_WINDOW_MAX_X)
				&& pose.getMeasureY().gte(AutoConstants.MIDFIELD_CONTACT_WINDOW_MIN_Y)
				&& pose.getMeasureY().lte(AutoConstants.MIDFIELD_CONTACT_WINDOW_MAX_Y);
		return inMidfieldWindow;
	}
}
