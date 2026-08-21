package frc.lib.util.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;

public class VisionEstimate {

	private final AprilTag tags[];
	private final Pose2d pose;
	private final Time timestamp;
	private Optional<Distance> averageDistance = Optional.empty();

	public VisionEstimate(Pose2d pose, Time timestamp, AprilTag tags[]) {
		this.pose = pose;
		this.timestamp = timestamp;
		this.tags = tags;
	}

	public VisionEstimate(Pose2d pose, Time timestamp, int tagIDs[]) {
		this(pose, timestamp, FieldLayout.getAprilTagArrayFromIDs(tagIDs));
	}

	public VisionEstimate(Pose2d pose, int tagIDs[]) {
		this(pose, Seconds.of(Timer.getFPGATimestamp()), tagIDs);
	}

	public VisionEstimate(Pose2d pose, AprilTag tags[]) {
		this(pose, Seconds.of(Timer.getFPGATimestamp()), tags);
	}

	public Pose2d getPose() {
		return pose;
	}

	public Time getTimestamp() {
		return timestamp;
	}

	public AprilTag[] getTags() {
		return tags;
	}

	public VisionEstimate withAverageDistance(Distance averageDistance) {
		this.averageDistance = Optional.of(averageDistance);
		return this;
	}

	public Distance getAverageDistance() {
		return averageDistance.orElseGet(() -> {
			Distance average = BaseUnits.DistanceUnit.zero();
			if (tags.length > 0) {
				for (AprilTag tag : tags)
					average = average.plus(Meters.of(tag.pose
							.getTranslation()
							.toTranslation2d()
							.getDistance(Drive.mInstance.getPose().getTranslation())));
				average = average.div(tags.length);
			}
			return average;
		});
	}

	public void log(String key) {
		LogUtil.log(key + "/Pose", pose);
		SmartDashboard.putNumber(key + "/Timestamp Seconds", timestamp.in(Seconds));
	}
}
