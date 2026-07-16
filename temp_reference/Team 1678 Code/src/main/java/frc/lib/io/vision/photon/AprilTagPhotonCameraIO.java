package frc.lib.io.vision.photon;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.util.MathHelpers;
import frc.lib.util.vision.VisionEstimate;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagPhotonCameraIO extends PhotonCameraIO {

	private final PhotonPoseEstimator estimator;
	private final AprilTagFieldLayout layout;

	private final PoseStrategy strategy;

	private Optional<List<VisionEstimate>> lastGivenEstimate = Optional.empty();

	public AprilTagPhotonCameraIO(CameraIOConfig config, PoseStrategy strategy, AprilTagFieldLayout layout) {
		super(config);
		this.layout = layout;
		this.strategy = strategy;
		estimator = new PhotonPoseEstimator(layout, MathHelpers.transformFromPose(config.robotToCameraOffset));
	}

	public Optional<EstimatedRobotPose> getPoseEstimateWithStrategy(PhotonPipelineResult result) {
		Optional<EstimatedRobotPose> estimate = Optional.empty();
		switch (strategy) {
			case MULTI_TAG_PNP_ON_COPROCESSOR:
				estimate = estimator.estimateAverageBestTargetsPose(result);
				return estimate;
			case PNP_DISTANCE_TRIG_SOLVE:
				estimate = estimator.estimatePnpDistanceTrigSolvePose(result);
				return estimate;
			case AVERAGE_BEST_TARGETS:
				estimate = estimator.estimateAverageBestTargetsPose(result);
				return estimate;
			case CLOSEST_TO_CAMERA_HEIGHT:
				estimate = estimator.estimateClosestToCameraHeightPose(result);
				return estimate;
			case LOWEST_AMBIGUITY:
				estimate = estimator.estimateLowestAmbiguityPose(result);
				return estimate;
			default:
				estimate = estimator.estimateLowestAmbiguityPose(result);
				return estimate;
		}
	}

	public Optional<VisionEstimate> convertRawEstimate(PhotonPipelineResult result) {
		Optional<EstimatedRobotPose> updated = getPoseEstimateWithStrategy(result);
		if (updated.isPresent()) {
			EstimatedRobotPose estimate = updated.get();
			int buffer[] = new int[estimate.targetsUsed.size()];
			for (int i = 0; i < buffer.length; i++) {
				buffer[i] = estimate.targetsUsed.get(i).fiducialId;
			}
			return Optional.of(new VisionEstimate(
					estimate.estimatedPose.toPose2d(), Seconds.of(estimate.timestampSeconds), buffer));
		}
		return Optional.empty();
	}

	public Optional<List<VisionEstimate>> getLastEstimates() {
		ArrayList<VisionEstimate> buffer = new ArrayList<>();
		if (getLastInputBuffer().isEmpty()) {
			return Optional.empty();
		}

		for (PhotonPipelineResult result : getLastInputBuffer()) {
			Optional<VisionEstimate> output = convertRawEstimate(result);
			output.ifPresent(converted -> buffer.add(converted));
		}
		return (lastGivenEstimate = buffer.isEmpty() ? Optional.empty() : Optional.of(buffer));
	}

	@Override
	public void update() {
		super.update();
		outputTelemetry();
		estimator.addHeadingData(
				Timer.getFPGATimestamp(), Drive.mInstance.getPose().getRotation());
	}

	public void outputTelemetry() {
		lastGivenEstimate.ifPresent((estimates) -> {
			for (int i = 0; i < estimates.size(); i++) {
				estimates.get(i).log(config.name + "/estimate/" + i);
			}
		});
	}
}
