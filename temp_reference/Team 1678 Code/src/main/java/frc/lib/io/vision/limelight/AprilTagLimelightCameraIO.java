package frc.lib.io.vision.limelight;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout;
import frc.lib.util.vision.LimelightHelpers;
import frc.lib.util.vision.LimelightHelpers.PoseEstimate;
import frc.lib.util.vision.VisionEstimate;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.Optional;

public class AprilTagLimelightCameraIO extends LimelightCameraIO {

	private Optional<VisionEstimate> lastGivenEstimate = Optional.empty();

	public AprilTagLimelightCameraIO(CameraIOConfig config) {
		super(config);
	}

	private void updateGyro() {
		LimelightHelpers.SetRobotOrientation(
				config.name, Drive.mInstance.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
	}

	// spotless:off

	@Override
	public Optional<VisionEstimate> getRobotPose() {
		updateGyro();
		PoseEstimate limelightEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name);
		if (limelightEstimate == null
				|| (limelightEstimate.pose.getX() == 0 && limelightEstimate.pose.getY() == 0)
				|| limelightEstimate.tagCount < 1)
			return (lastGivenEstimate = Optional.empty());

		int buffer[] = new int[limelightEstimate.rawFiducials.length];
		for (int i = 0; i < buffer.length; i++)
			buffer[i] = limelightEstimate.rawFiducials[i].id;
		lastGivenEstimate = Optional.of(new VisionEstimate(
						limelightEstimate.pose,
						Seconds.of(limelightEstimate.timestampSeconds),
						FieldLayout.getAprilTagArrayFromIDs(buffer))
				.withAverageDistance(Meters.of(limelightEstimate.avgTagDist)));
		return lastGivenEstimate;
	}

	// spotless:on

	@Override
	public Optional<List<VisionEstimate>> getLastEstimates() {
		updateGyro();
		PoseEstimate limelightEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name);
		if (limelightEstimate == null
				|| (limelightEstimate.pose.getX() == 0
						|| limelightEstimate.pose.getY() == 0
						|| limelightEstimate.tagCount < 1)) {
			return Optional.empty();
		}
		int buffer[] = new int[limelightEstimate.rawFiducials.length];
		for (int i = 0; i < buffer.length; i++) buffer[i] = limelightEstimate.rawFiducials[i].id;
		VisionEstimate estimate = new VisionEstimate(
						limelightEstimate.pose,
						Seconds.of(limelightEstimate.timestampSeconds),
						FieldLayout.getAprilTagArrayFromIDs(buffer))
				.withAverageDistance(Meters.of(limelightEstimate.avgTagDist));
		LogUtil.log("Vision ESTIMATE", estimate.getPose());
		return Optional.of(List.of(estimate));
	}

	@Override
	public void update() {
		outputTelemetry();
		updateGyro();
	}

	public void outputTelemetry() {
		lastGivenEstimate.ifPresent((lastEstimate) -> lastEstimate.log(config.name + "/Last Estimate"));
	}
}
