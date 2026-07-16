package frc.lib.io.vision.sim;

import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.bases.CameraSubsystem.CombinedSimulatedCameraIOConfig;
import frc.lib.util.vision.VisionGamePiece;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.CopyOnWriteArrayList;

public class CombinedSimulatedCameraIO extends AprilTagSimulatedCameraIO {

	private final List<VisionGamePiece> buffer = new CopyOnWriteArrayList<>();
	private final HashMap<String, Short> lookUpTable = new HashMap<>();
	private final CombinedSimulatedCameraIOConfig combinedConfig;

	// spotless:off

	public CombinedSimulatedCameraIO(CameraIOConfig config, CombinedSimulatedCameraIOConfig simConfig) {
		super(config, simConfig);
		for (String target : simConfig.targetPieces)
			lookUpTable.put(target, (short) 0xff);
		combinedConfig = simConfig;
	}

	@Override
	public Optional<List<VisionGamePiece>> getAllGamePieces() {
		return Optional.empty();
	}

	// spotless:on
}
