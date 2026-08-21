package frc.lib.io.vision.sim;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.bases.CameraSubsystem.SimulatedCameraIOConfig;
import frc.lib.util.FieldLayout;
import frc.lib.util.vision.VisionEstimate;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.Optional;

public class AprilTagSimulatedCameraIO extends SimulatedCameraIO {

	private boolean pointedAtTag = false;
	private double lastUpdatedErr = 0;
	private long numFound = 0;
	private Optional<VisionEstimate> latestEstimate = Optional.empty();

	public AprilTagSimulatedCameraIO(CameraIOConfig config, SimulatedCameraIOConfig simConfig) {
		super(config, simConfig);
	}

	// spotless:off

	@Override
	public Optional<VisionEstimate> getRobotPose() {
		Rotation2d driveRotation = Drive.mInstance.getPose().getRotation();
		Translation2d driveTranslation = Drive.mInstance.getPose().getTranslation();

		ArrayList<AprilTag> found = new ArrayList<>();
		for (AprilTag tag : FieldLayout.kAprilTagMap.getTags()) {
			if (!FieldLayout.isAprilTagOnRobotAlliance(tag))
				continue;
			Translation2d tagTranslation = tag.pose.getTranslation().toTranslation2d();
			Rotation2d tagRotation = tag.pose.getRotation().toRotation2d();

			Distance dist = Meters.of(tagTranslation.getDistance(driveTranslation));
			if (dist.gt(simConfig.maxUpdateTagDistance))
				continue;
			int xIndex = 0, yIndex = 1;

			Vector<N2> deltaVec = VecBuilder.fill(
					tagTranslation.getX() - driveTranslation.getX(), tagTranslation.getY() - driveTranslation.getY());

			Rotation2d toTagRotation = new Rotation2d(deltaVec.get(xIndex), deltaVec.get(yIndex));

			Angle fov = Radians.of(Math.abs(tagRotation.minus(driveRotation).getRadians()));
			Angle err = Radians.of(Math.abs(driveRotation
					.minus(toTagRotation)
					.rotateBy(Rotation2d.k180deg)
					.getRadians()));

			if (err.lte(simConfig.kErrThreshold) && fov.lte(simConfig.kFieldOfView)) {
				pointedAtTag = true;
				found.add(tag);
			}
		}
		if (!found.isEmpty()) {
			AprilTag buffer[] = new AprilTag[found.size()];
			found.toArray(buffer);
			return (latestEstimate = Optional.of(new VisionEstimate(Drive.mInstance.getPose(), buffer)));
		}
		return (latestEstimate = Optional.empty());
	}

	// spotless:on

	@Override
	public void update() {
		outputTelemetry();
	}

	public void outputTelemetry() {
		latestEstimate.ifPresent((lastEstimate) -> lastEstimate.log(config.name + "/Last Estimate"));
	}
}
