package frc.lib.bases;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.vision.CameraIO;
import frc.lib.util.FieldLayout;
import frc.lib.util.vision.CameraPipeline;
import frc.lib.util.vision.VisionEstimate;
import frc.lib.util.vision.VisionGamePiece;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.Supplier;

public class CameraSubsystem extends SubsystemBase {

	private VisionConfig config = new VisionConfig();

	private Time lastUpdatePoseTime = Seconds.of(0d), lastUpdateDetectionTime = Seconds.of(0d);
	private Pose2d lastPose = new Pose2d();
	private int numPoseStableUpdates = 0;
	private ArrayList<VisionGamePiece> tracker = new ArrayList<>();

	private Vector<N3> m_deviations;

	private Optional<VisionEstimate> lastEstimate = Optional.empty();

	private boolean enabled = true;

	public CameraSubsystem(VisionConfig config) {
		super(config.name);
		this.config = config;
		m_deviations = config.cameras[0].getAprilTagStdDevs();
	}

	private void updateDetection() {
		ArrayList<VisionGamePiece> all = getAllDetections();
		if (all.size() == 0) return;
		Time now = Seconds.of(Timer.getFPGATimestamp());
		tracker.removeIf((piece) -> now.minus(piece.getTimeStamp()).gte(Seconds.of(0.2)));
		while (tracker.size() > 20) {
			tracker.remove(0);
		}
		for (VisionGamePiece detection : all) {
			// if (detection.type == 0) continue;
			tracker.add(detection);
		}
		getCoralPose();
	}

	private ArrayList<VisionGamePiece> getAllDetections() {
		ArrayList<VisionGamePiece> all = new ArrayList<>();
		for (CameraIO camera : config.cameras) {
			Optional<List<VisionGamePiece>> result = camera.getAllGamePieces();
			if (result.isEmpty()) continue;
			lastUpdateDetectionTime = Seconds.of(Timer.getFPGATimestamp());
			all.addAll(result.get());
		}

		return all;
	}

	public void applyVisionEstimate(CameraIO camera, VisionEstimate estimate) {
		Drive.mInstance.addVisionUpdate(
				estimate.getPose(),
				estimate.getTimestamp(),
				camera.getAprilTagStdDevs().times(estimate.getAverageDistance().in(Meters)));

		if (estimate.getPose() != lastPose) {
			if (Drive.mInstance
							.getPose()
							.getTranslation()
							.getDistance(estimate.getPose().getTranslation())
					< config.agreedTranslationUpdateEpsilon.in(Units.Meters)) {
				numPoseStableUpdates++;
			} else {
				numPoseStableUpdates = 0;
			}
		}
		lastEstimate = Optional.of(estimate);
		lastPose = estimate.getPose();
		lastUpdatePoseTime = Seconds.of(Timer.getFPGATimestamp());
	}

	private void updateLocalization() {
		if (Drive.mInstance.pitchStable()) {
			for (CameraIO camera : config.cameras) {
				Optional<List<VisionEstimate>> estimatesOptional = camera.getLastEstimates();
				estimatesOptional.ifPresent(estimates -> {
					for (VisionEstimate estimate : estimates) {
						applyVisionEstimate(camera, estimate);
						lastUpdatePoseTime = Seconds.of(Timer.getFPGATimestamp());
					}
				});
			}
		}
	}

	@Deprecated
	private void updatePose() {
		updateLocalization();
	}

	@Override
	public void periodic() {
		if (enabled) {
			for (CameraIO camera : config.cameras) {
				camera.update();
			}
			outputTelemetry();
			updateDetection();
			updatePose();
		}
	}

	public void setPipeline(Function<CameraIO, CameraPipeline> function) {
		for (CameraIO camera : config.cameras) {
			CameraPipeline pipelineToApply = function.apply(camera);
			camera.setPipeline(pipelineToApply);
		}
	}

	public void setPipeline(CameraPipeline pipeline) {
		setPipeline(io -> pipeline);
	}

	public void outputTelemetry() {}

	public final Pose2d getCoralPose() {
		return getCoralPose(Drive.mInstance.getPose().getTranslation());
	}

	public Optional<VisionGamePiece> getClosestsGamePiece(Translation2d base) {
		AtomicReference<Optional<VisionGamePiece>> closestGamePieceOptionalRefrence =
				new AtomicReference<>(Optional.empty());
		for (VisionGamePiece detection : tracker) {
			closestGamePieceOptionalRefrence
					.get()
					.ifPresentOrElse(
							(closestGamePiece) -> {
								if (closestGamePiece
												.getPose2d()
												.getTranslation()
												.getDistance(base)
										> detection.getPose2d().getTranslation().getDistance(base))
									closestGamePieceOptionalRefrence.set(Optional.ofNullable(detection));
							},
							() -> {
								closestGamePieceOptionalRefrence.set(Optional.ofNullable(detection));
							});
		}
		return closestGamePieceOptionalRefrence.get();
	}

	public final Optional<VisionGamePiece> getClosestsGamePiece() {
		return getClosestsGamePiece(Drive.mInstance.getPose().getTranslation());
	}

	public Pose2d getCoralPose(Translation2d base) {
		// Translation2d bestTranslation = null;
		// Pose2d bestCoralPose = null;
		// for (VisionGamePiece detection : tracker) {
		// if (bestTranslation == null
		// || bestCoralPose.getTranslation().getDistance(base)
		// > detection.getPose2d().getTranslation().getDistance(base)) {
		// bestTranslation = detection.getPose2d().getTranslation();
		// bestCoralPose = detection.getPose2d();
		// }
		// }
		// if (bestCoralPose != null) {}
		// return bestCoralPose;
		Optional<VisionGamePiece> piece = getClosestsGamePiece(base);
		if (piece.isPresent()) return piece.get().getPose2d();
		else return null;
	}

	public Pose2d getCoralTranslationAndPoint(Translation2d base) {
		Translation2d t = getCoralPose(base).getTranslation();
		Rotation2d r = t.minus(Drive.mInstance.getPose().getTranslation()).getAngle();
		return new Pose2d(t, r);
	}

	public Pose2d getCoralTranslationAndPoint() {
		return getCoralTranslationAndPoint(Drive.mInstance.getPose().getTranslation());
	}

	public int coralCount() {
		return getAllDetections().size();
	}

	public boolean hasGamePiece() {
		return coralCount() > 0;
	}

	public Time getLastUpdatedPoseTime() {
		updatePose();
		return lastUpdatePoseTime;
	}

	public boolean getPoseStable() {
		return numPoseStableUpdates >= config.agreedTranslationUpdatesThreshold;
	}

	public boolean getPoseStable(long count) {
		return numPoseStableUpdates >= count;
	}

	public Pose2d getLatestUpdate() {
		updatePose();
		return lastPose;
	}

	public long getNumPoseStableUpdates() {
		return numPoseStableUpdates;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addBooleanProperty("Has Game Piece", () -> hasGamePiece(), null);
		builder.addDoubleProperty("Last Updated Tag Time", () -> lastUpdatePoseTime.in(Seconds), null);
		builder.addDoubleProperty("Last Updated Detection Time", () -> lastUpdateDetectionTime.in(Seconds), null);
		builder.addDoubleProperty("Num Agreed Stable Updates", () -> numPoseStableUpdates, null);
		builder.addBooleanProperty("Pose Stable", this::getPoseStable, null);
		builder.addDoubleArrayProperty(
				"Standard Deviations",
				() -> new double[] {m_deviations.get(0), m_deviations.get(1), m_deviations.get(2)},
				null);
		for (CameraIO io : config.cameras) {
			io.initSendable(builder);
		}
	}

	public static class CameraIOConfig {
		public String name = null;
		public Pose3d robotToCameraOffset = null;
		public Vector<N3> aprilTagVisionStdDevs = VecBuilder.fill(0.3, 0.3, 99999.0);
	}

	public static class SimulatedCameraIOConfig {
		public Side side = Side.BACK;
		public Distance maxUpdateTagDistance = Meters.of(10d);
		public Angle kErrThreshold = Degrees.of(20d);
		public Angle kFieldOfView = Degrees.of(80d);

		public int resolutionHeightPixels = 600; // Pixels
		public int resolutionWidthPixels = 800; // Pixels

		public double calibErrorPx = 0.25;
		public double calibErrorPy = 0.25;

		public double frameRateFramesPerSecond = 60.0; // frames per second
		public Time latency = Units.Milliseconds.of(20.0);
		public Time latencyStdDevs = Units.Milliseconds.of(5.0);

		public SimulatedCameraIOConfig withSide(Side side) {
			this.side = side;
			return this;
		}
	}

	public static class CombinedSimulatedCameraIOConfig extends SimulatedCameraIOConfig {
		public String targetPieces[];
		public Angle fov = null;
		public Distance maxDetectionDistance = null;
		public Supplier<Pose2d> baseSupplier;
	}

	public static class VisionConfig {
		public String name = "Vision";
		public CameraIO cameras[];
		public int agreedTranslationUpdatesThreshold;
		public Distance agreedTranslationUpdateEpsilon;
	}

	public enum Side {
		BACK(BaseUnits.AngleUnit.zero()),
		FRONT(Degrees.of(180.0)),
		LEFT(Degrees.of(90.0)),
		RIGHT(Degrees.of(270.0));

		public final Rotation2d rotationOffset;

		private Side(Rotation2d rotationOffset) {
			this.rotationOffset = rotationOffset;
		}

		private Side(Angle offset) {
			this(Rotation2d.fromDegrees(offset.in(Degrees)));
		}
	}

	public void disable() {
		enabled = false;
	}

	public void enable() {
		enabled = true;
	}

	public boolean getEnabled() {
		return enabled;
	}

	public void setSTDDeviations(Vector<N3> deviations) {
		for (CameraIO camera : config.cameras) {
			camera.setStdDeviations(deviations);
		}
		m_deviations = deviations;
	}

	public Command setStdDevCommand(Vector<N3> deviations) {
		return Commands.runOnce(() -> setSTDDeviations(deviations));
	}

	public Integer[] getTargetIDs() {
		Integer[] ids = {};
		if (lastEstimate.isPresent()) {
			ids = FieldLayout.getIDArrayFromAprilTagArray(lastEstimate.get().getTags());
		}
		return ids;
	}
}
