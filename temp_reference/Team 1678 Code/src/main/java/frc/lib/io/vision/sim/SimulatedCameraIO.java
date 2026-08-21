package frc.lib.io.vision.sim;

import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.bases.CameraSubsystem.SimulatedCameraIOConfig;
import frc.lib.io.vision.CameraIO;
import frc.lib.util.vision.CameraPipeline;

public abstract class SimulatedCameraIO extends CameraIO {

	protected final SimulatedCameraIOConfig simConfig;

	public SimulatedCameraIO(CameraIOConfig config, SimulatedCameraIOConfig simConfig) {
		super(config);
		this.simConfig = simConfig;
	}

	@Override
	public void updatePipeline(CameraPipeline pipeline) {}
}
