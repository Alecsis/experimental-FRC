package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logging.LogUtil;
import frc.lib.util.DelayedBoolean;
import frc.lib.util.Util;
import frc.lib.util.controller.SynchronousPIDF;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;

public class PIDToPoseCommand extends Command {
	Pose2d finalPose;
	Rotation2d targetRotation;
	Distance epsilonDist;
	Distance epsilonX;
	Distance epsilonY;
	Angle epsilonAngle;
	Util.Pose2dTimeInterpolable interpolable;
	Time lookaheadTime;
	DelayedBoolean atTarget;
	boolean isAuto;
	PIDController translationController;
	SynchronousPIDF headingController;

	public PIDToPoseCommand(
			Pose2d finalPose,
			Distance epsilonDist,
			Angle epsilonAngle,
			Time delayTime,
			Time lookaheadTime,
			Rotation2d targetRotation,
			PIDController translationController,
			SynchronousPIDF headingController) {
		addRequirements(Drive.mInstance);

		this.finalPose = finalPose;
		this.epsilonDist = epsilonDist;
		this.epsilonAngle = epsilonAngle;
		this.lookaheadTime = lookaheadTime;
		this.targetRotation = targetRotation.plus(Rotation2d.k180deg);
		this.translationController = translationController;
		this.headingController = headingController;

		atTarget = new DelayedBoolean(Timer.getFPGATimestamp(), delayTime.in(Units.Seconds));
	}

	public PIDToPoseCommand(
			Pose2d finalPose,
			Distance epsilonX,
			Distance epsilonY,
			Angle epsilonAngle,
			Time delayTime,
			Time lookaheadTime,
			Rotation2d targetRotation,
			PIDController translationController,
			SynchronousPIDF headingController) {
		addRequirements(Drive.mInstance);

		this.finalPose = finalPose;
		this.epsilonX = epsilonX;
		this.epsilonY = epsilonY;
		this.epsilonAngle = epsilonAngle;
		this.lookaheadTime = lookaheadTime;
		this.targetRotation = targetRotation.plus(Rotation2d.k180deg);
		this.translationController = translationController;
		this.headingController = headingController;

		atTarget = new DelayedBoolean(Timer.getFPGATimestamp(), delayTime.in(Units.Seconds));
	}

	public PIDToPoseCommand(
			Pose2d finalPose,
			Distance epsilonDist,
			Angle epsilonAngle,
			Time delayTime,
			Time lookaheadTime,
			Rotation2d targetRotation) {
		this(
				finalPose,
				epsilonDist,
				epsilonAngle,
				delayTime,
				lookaheadTime,
				targetRotation,
				DriveConstants.mAutoAlignTranslationController,
				DriveConstants.mAutoAlignHeadingController);
	}

	public PIDToPoseCommand(
			Pose2d finalPose,
			Distance epsilonDist,
			Angle epsilonAngle,
			Time delayTime,
			Time lookaheadTime,
			PIDController translationController,
			SynchronousPIDF headingController) {
		this(
				finalPose,
				epsilonDist,
				epsilonAngle,
				delayTime,
				lookaheadTime,
				finalPose.getRotation(),
				translationController,
				headingController);
	}

	public PIDToPoseCommand(
			Pose2d finalPose,
			Time delayTime,
			Time lookaheadTime,
			Rotation2d targetRotation,
			PIDController translationController,
			SynchronousPIDF headingController) {
		this(
				finalPose,
				Units.Centimeters.of(4.0),
				Units.Degrees.of(0.8),
				delayTime,
				lookaheadTime,
				targetRotation,
				translationController,
				headingController);
	}

	public PIDToPoseCommand(
			Pose2d finalPose,
			Time delayTime,
			Time lookaheadTime,
			PIDController translationController,
			SynchronousPIDF headingController) {
		this(finalPose, delayTime, lookaheadTime, finalPose.getRotation(), translationController, headingController);
	}

	public PIDToPoseCommand(Pose2d rawEndPose, boolean diffParams) {
		this(
				rawEndPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.k180deg)),
				Units.Meters.of(0.5),
				Units.Degrees.of(0.5),
				Units.Seconds.of(0.5),
				Units.Seconds.of(0.5),
				rawEndPose.getRotation(),
				DriveConstants.mAutoAlignTranslationController,
				DriveConstants.mAutoAlignHeadingController);
	}

	public PIDToPoseCommand(
			Pose2d finalPose,
			Distance epsilonDist,
			Angle epsilonAngle,
			PIDController translationController,
			SynchronousPIDF headingController) {
		this(
				finalPose,
				epsilonDist,
				epsilonAngle,
				Units.Seconds.of(0.0),
				Units.Seconds.of(0.16),
				finalPose.getRotation(),
				translationController,
				headingController);
	}

	// public PIDToPoseCommand(
	//		Pose2d finalPose, SynchronousPIDF translationController, SynchronousPIDF headingController) {
	//	this(finalPose, Units.Seconds.of(0.00), Units.Seconds.of(0.16), translationController, headingController);
	// }

	public PIDToPoseCommand(Pose2d finalPose, Distance distTolerance, Angle rotationTolerance) {
		this(
				finalPose,
				DriveConstants.mAutoAlignTranslationController,
				DriveConstants.mAutoAlignHeadingController,
				distTolerance,
				rotationTolerance);
	}

	public PIDToPoseCommand(Pose2d finalPose, Distance xTolerance, Distance yTolerance, Angle rotationTolerance) {
		this(
				finalPose,
				DriveConstants.mAutoAlignTranslationController,
				DriveConstants.mAutoAlignHeadingController,
				xTolerance,
				yTolerance,
				rotationTolerance);
	}

	/* AUTO ALIGN PID TO POSE COMMANDS */
	public PIDToPoseCommand(
			Pose2d finalPose,
			PIDController translationController,
			SynchronousPIDF headingController,
			Distance distTolerance,
			Angle rotationTolerance) {
		this(
				finalPose,
				distTolerance, // pose tolerance
				rotationTolerance, // rotation tolerance
				Units.Milliseconds.of(40.0),
				Units.Seconds.of(0.16),
				finalPose.getRotation(),
				translationController,
				headingController);
	}

	public PIDToPoseCommand(
			Pose2d finalPose,
			PIDController translationController,
			SynchronousPIDF headingController,
			Distance xTolerance,
			Distance yTolerance,
			Angle rotationTolerance) {
		this(
				finalPose,
				xTolerance, // x tolerance
				yTolerance, // y tolerance
				rotationTolerance, // rotation tolerance
				Units.Milliseconds.of(40.0),
				Units.Seconds.of(0.16),
				finalPose.getRotation(),
				translationController,
				headingController);
	}

	// public PIDToPoseCommand(Pose2d finalPose, SynchronousPIDF translationController) {
	//	this(finalPose, translationController, DriveConstants.mAutoAlignHeadingController);
	// }

	// public PIDToPoseCommand(Pose2d finalPose) {
	//	this(
	//			finalPose,
	//			DriveConstants.mAutoAlignTranslationController,
	//			DriveConstants.mAutoAlignHeadingController);
	// }

	@Override
	public void initialize() {
		Superstructure.mInstance.setPathFollowing(true);
		translationController.reset();
		headingController.reset();
	}

	@Override
	public void execute() {
		LogUtil.log("FinalPose", finalPose);
		Drive.mInstance.setSwerveRequest(
				DriveConstants.getPIDToPoseRequestUpdater(finalPose, translationController, headingController)
						.apply(DriveConstants.PIDToPoseRequest));
	}

	@Override
	public void end(boolean interrupted) {
		Superstructure.mInstance.setPathFollowing(false);
		Drive.mInstance.setSwerveRequest(new SwerveRequest.ApplyRobotSpeeds());
	}

	@Override
	public boolean isFinished() {
		return atEndPose();
	}

	public boolean atEndPose() {
		Pose2d currentPose = Drive.mInstance.getPose();
		SmartDashboard.putNumber(
				"Auto Align PID/AngleModulus",
				MathUtil.angleModulus(Math.abs(
						currentPose.getRotation().minus(finalPose.getRotation()).getRadians())));

		boolean complete;
		if (epsilonDist != null) {
			complete = atTarget.update(
					Timer.getFPGATimestamp(),
					currentPose.getTranslation().getDistance(finalPose.getTranslation()) < epsilonDist.in(Units.Meters)
							&& MathUtil.angleModulus(Math.abs(currentPose
											.getRotation()
											.minus(finalPose.getRotation())
											.getRadians()))
									< epsilonAngle.in(Units.Radians));

			SmartDashboard.putBoolean(
					"Auto Align PID/Translation Completed",
					currentPose.getTranslation().getDistance(finalPose.getTranslation())
							< epsilonDist.in(Units.Meters));
		} else {
			complete = atTarget.update(
					Timer.getFPGATimestamp(),
					Math.abs(currentPose.getX() - finalPose.getX()) < epsilonX.in(Units.Meters)
							&& Math.abs(currentPose.getY() - finalPose.getY()) < epsilonY.in(Units.Meters)
							&& MathUtil.angleModulus(Math.abs(currentPose
											.getRotation()
											.minus(finalPose.getRotation())
											.getRadians()))
									< epsilonAngle.in(Units.Radians));

			SmartDashboard.putBoolean(
					"Auto Align PID/X Translation Completed",
					Math.abs(currentPose.getX() - finalPose.getX()) < epsilonX.in(Units.Meters));
			SmartDashboard.putBoolean(
					"Auto Align PID/Y Translation Completed",
					Math.abs(currentPose.getY() - finalPose.getY()) < epsilonY.in(Units.Meters));

			SmartDashboard.putBoolean(
					"X Climber Tolerance Reached",
					Math.abs(currentPose.getX() - finalPose.getX()) < (epsilonX.in(Units.Meters) + 0.04 + 0.076));
			SmartDashboard.putBoolean(
					"Y Climber Tolerance Reached",
					Math.abs(currentPose.getY() - finalPose.getY()) < (epsilonY.in(Units.Meters) + 0.035));
		}

		SmartDashboard.putBoolean("Auto Align PID/Completed", complete);

		SmartDashboard.putBoolean(
				"Auto Align PID/Rotation Completed",
				MathUtil.angleModulus(Math.abs(currentPose
								.getRotation()
								.minus(finalPose.getRotation())
								.getRadians()))
						< epsilonAngle.in(Units.Radians));
		SmartDashboard.putNumber(
				"X Distance",
				currentPose.getMeasureX().minus(finalPose.getMeasureX()).abs(Units.Meters));
		SmartDashboard.putNumber(
				"Y Distance",
				currentPose.getMeasureY().minus(finalPose.getMeasureY()).abs(Units.Meters));
		SmartDashboard.putNumber(
				"Auto Align PID/Distance Away Inches",
				currentPose.getTranslation().getDistance(finalPose.getTranslation()) * 39.37);

		return complete;
	}
}
