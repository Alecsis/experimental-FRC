package frc.lib.io.vision.sim.photon;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.bases.CameraSubsystem.SimulatedCameraIOConfig;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout;
import frc.lib.util.MathHelpers;
import frc.lib.util.vision.VisionEstimate;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagSimulatedPhotonCameraIO extends SimulatedPhotonCameraIO {

	private final PhotonPoseEstimator estimator;
	PoseStrategy strategy;

	Optional<VisionEstimate> latestEstimate = Optional.empty();

	public AprilTagSimulatedPhotonCameraIO(
			VisionSystemSim visionSim,
			CameraIOConfig cameraConfig,
			SimulatedCameraIOConfig simConfig,
			PoseStrategy strategy) {
		super(visionSim, cameraConfig, simConfig);
		this.strategy = strategy;
		estimator = new PhotonPoseEstimator(
				FieldLayout.kAprilTagMap, MathHelpers.transformFromPose(config.robotToCameraOffset));
	}

	@Override
	public Optional<List<VisionEstimate>> getLastEstimates() {
		List<VisionEstimate> estimates = new ArrayList<>();
		List<VisionTargetSim> visionTargets = new ArrayList<>();
		visionTargets.addAll(visionSim.getVisionTargets());
		Pose3d cameraPose = new Pose3d(
				Drive.mInstance.getPose().getX() + (config.robotToCameraOffset.getX()),
				Drive.mInstance.getPose().getY() + (config.robotToCameraOffset.getY()),
				config.robotToCameraOffset.getZ(),
				Rotation3d.kZero);
		PhotonPipelineResult result =
				cameraSim.process(simConfig.latencyStdDevs.in(Milliseconds), cameraPose, visionTargets);
		Optional<EstimatedRobotPose> photonEstimate = getPoseEstimateWithStrategy(result);
		photonEstimate.ifPresentOrElse(
				est -> {
					visionSim.getDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d());
					var estimateToAdd = new VisionEstimate(
							est.estimatedPose.toPose2d(),
							Units.Seconds.of(est.timestampSeconds),
							getIDArrayFromPhotonTargets(est.targetsUsed));
					estimates.add(estimateToAdd);
					latestEstimate = Optional.of(estimateToAdd);
					SmartDashboard.putNumber("Last Valid Estimate Timestamp Seconds", Timer.getFPGATimestamp());
				},
				() -> {
					visionSim.getDebugField().getObject("VisionEstimation").setPoses();
					SmartDashboard.putNumber(
							"Last Vision Sim Null Estimate Timestamp Seconds", Timer.getFPGATimestamp());
				});
		return Optional.of(estimates);
	}

	@Override
	public void outputTelemetry() {
		latestEstimate.ifPresent((lastEstimate) -> {
			lastEstimate.log(config.name + "/Last Estimate");
			LogUtil.log("TEST LAST ESTIMATE", lastEstimate.getPose());
		});
	}

	public Optional<EstimatedRobotPose> getPoseEstimateWithStrategy(PhotonPipelineResult result) {
		/** Get Robot Pose for Strategy */
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
}
