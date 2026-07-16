package frc.lib.io.vision.sim;

import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.io.vision.CameraIO;
import frc.lib.util.vision.CameraPipeline;

public class EmptySimulatedCameraIO extends CameraIO {

	public EmptySimulatedCameraIO(CameraIOConfig config) {
		super(config);
	}

	@Override
	public void updatePipeline(CameraPipeline pipeline) {}
}
