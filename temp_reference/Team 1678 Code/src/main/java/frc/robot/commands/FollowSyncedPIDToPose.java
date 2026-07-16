package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.superstructure.Superstructure;

public class FollowSyncedPIDToPose extends PIDToPoseCommand {

	public FollowSyncedPIDToPose(Pose2d finalPose, Distance distTolerance, Angle rotationTolerance) {
		super(finalPose, distTolerance, rotationTolerance);

		Superstructure.mInstance.setSuperstructureDone(false);
		Superstructure.mInstance.setDriveReady(false);
	}

	public FollowSyncedPIDToPose(Pose2d finalPose, Distance xTolerance, Distance yTolerance, Angle rotationTolerance) {
		super(finalPose, xTolerance, yTolerance, rotationTolerance);

		Superstructure.mInstance.setSuperstructureDone(false);
		Superstructure.mInstance.setDriveReady(false);
	}

	public boolean driveDone() {
		return super.isFinished();
	}

	@Override
	public void execute() {
		super.execute();
		Superstructure.mInstance.setDriveReady(driveDone());
	}

	@Override
	public boolean isFinished() {
		return Superstructure.mInstance.getSuperstructureDone();
	}
}
