package frc.lib.io.vision.photon;

import edu.wpi.first.apriltag.AprilTag;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.io.vision.CameraIO;
import frc.lib.util.FieldLayout;
import frc.lib.util.vision.CameraPipeline;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCameraIO extends CameraIO {

	private final PhotonCamera wrappedCamera;
	protected List<PhotonPipelineResult> lastInputBuffer = new ArrayList<>();
	private PhotonPipelineResult m_lastResult = new PhotonPipelineResult();

	public PhotonCameraIO(CameraIOConfig config) {
		super(config);
		wrappedCamera = new PhotonCamera(config.name);
	}

	public void updateToLatestResult() {
		List<PhotonPipelineResult> results = wrappedCamera.getAllUnreadResults();
		for (PhotonPipelineResult result : results) {
			if (m_lastResult.metadata.captureTimestampMicros <= result.metadata.captureTimestampMicros) {
				m_lastResult = result;
			}
		}

		if (results.isEmpty()) {
			lastInputBuffer = List.of();
		}
		lastInputBuffer = List.of(m_lastResult);
	}

	@Override
	public void update() {
		lastInputBuffer = wrappedCamera.getAllUnreadResults();
	}

	@Override
	public void updatePipeline(CameraPipeline pipeline) {
		wrappedCamera.setPipelineIndex(pipeline.index());
	}

	public List<PhotonPipelineResult> getLastInputBuffer() {
		return lastInputBuffer;
	}

	public static int[] getIDArrayFromPhotonTargets(List<PhotonTrackedTarget> targets) {
		return (targets == null ? Stream.<PhotonTrackedTarget>empty() : targets.stream())
				.mapToInt(t -> t.fiducialId)
				.toArray();
	}

	protected static AprilTag[] getTagArrayFromPhotonTargets(List<PhotonTrackedTarget> targets) {
		return FieldLayout.getAprilTagArrayFromIDs(getIDArrayFromPhotonTargets(targets));
	}
}
