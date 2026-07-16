package frc.lib.io.vision;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.util.vision.CameraPipeline;
import frc.lib.util.vision.VisionEstimate;
import frc.lib.util.vision.VisionGamePiece;
import java.util.List;
import java.util.Optional;

public abstract class CameraIO implements Sendable {

	protected final CameraIOConfig config;
	protected CameraPipeline pipeline = CameraPipeline.getDefault();

	protected CameraIO(CameraIOConfig config) {
		this.config = config;
	}

	public abstract void updatePipeline(CameraPipeline pipeline);

	public Optional<List<VisionGamePiece>> getAllGamePieces() {
		return Optional.empty();
	}

	@Deprecated
	public Optional<VisionEstimate> getRobotPose() {
		return Optional.empty();
	}

	public Optional<List<VisionEstimate>> getLastEstimates() {
		return Optional.empty();
	}

	public Vector<N3> getAprilTagStdDevs() {
		return config.aprilTagVisionStdDevs;
	}

	public void setStdDeviations(Vector<N3> standardDeviations) {
		config.aprilTagVisionStdDevs = standardDeviations;
	}

	public void setPipeline(CameraPipeline pipeline) {
		this.pipeline = pipeline;
		updatePipeline(pipeline);
	}

	public void update() {}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addStringProperty(config.name + "/Pipeline/Name", () -> pipeline.name(), null);
		builder.addStringProperty(
				config.name + "/Pipeline/Type", () -> pipeline.type().name(), null);
		builder.addIntegerProperty(config.name + "/Pipeline/index", () -> pipeline.index(), null);
		builder.addStringProperty(config.name + "/Name", () -> config.name, null);
	}
}
