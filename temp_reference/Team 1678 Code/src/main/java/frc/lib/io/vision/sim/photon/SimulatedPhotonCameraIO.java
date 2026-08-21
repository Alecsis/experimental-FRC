package frc.lib.io.vision.sim.photon;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.bases.CameraSubsystem.SimulatedCameraIOConfig;
import frc.lib.io.vision.photon.PhotonCameraIO;
import frc.lib.util.MathHelpers;
import frc.robot.subsystems.drive.Drive;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class SimulatedPhotonCameraIO extends PhotonCameraIO {
	protected PhotonCamera photonCamera;
	protected PhotonCameraSim cameraSim;
	protected VisionSystemSim visionSim;
	protected SimulatedCameraIOConfig simConfig;

	public SimulatedPhotonCameraIO(
			VisionSystemSim visionSim, CameraIOConfig cameraConfig, SimulatedCameraIOConfig simConfig) {
		super(cameraConfig);
		this.simConfig = simConfig;
		this.visionSim = visionSim;
		photonCamera = new PhotonCamera(config.name);

		var cameraProp = new SimCameraProperties();
		cameraProp.setCalibration(
				simConfig.resolutionWidthPixels,
				simConfig.resolutionHeightPixels,
				Rotation2d.fromDegrees(simConfig.kFieldOfView.in(Degrees)));
		cameraProp.setCalibError(simConfig.calibErrorPx, simConfig.calibErrorPy);
		cameraProp.setFPS(simConfig.frameRateFramesPerSecond);
		cameraProp.setAvgLatencyMs(simConfig.latency.in(Milliseconds));
		cameraProp.setLatencyStdDevMs(simConfig.latencyStdDevs.in(Milliseconds));
		cameraSim = new PhotonCameraSim(photonCamera, cameraProp);
		cameraSim.setMaxSightRange(simConfig.maxUpdateTagDistance.in(Meters));
		cameraSim.enableDrawWireframe(true);
		cameraSim.enableProcessedStream(true);
		cameraSim.enableRawStream(true);
		visionSim.addCamera(cameraSim, MathHelpers.transformFromPose(cameraConfig.robotToCameraOffset));
	}

	public void resetSimPose(Pose2d pose) {
		visionSim.resetRobotPose(pose);
	}

	@Override
	public void update() {
		outputTelemetry();
		visionSim.update(Drive.mInstance.getPose());
		visionSim.getDebugField().setRobotPose(Drive.mInstance.getPose());
	}

	public void outputTelemetry() {}
}
