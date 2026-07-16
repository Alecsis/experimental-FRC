package frc.lib.util.vision;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

public class VisionGamePiece {
	private final Pose3d pose;
	private final Time timeStamp;

	public VisionGamePiece(Pose3d pose, Time timeStamp) {
		this.pose = pose;
		this.timeStamp = timeStamp;
	}

	public VisionGamePiece(Pose3d pose) {
		this(pose, Seconds.of(Timer.getFPGATimestamp()));
	}

	public VisionGamePiece(Pose2d pose, Time timestamp) {
		this(new Pose3d(pose), timestamp);
	}

	public VisionGamePiece(Pose2d pose) {
		this(new Pose3d(pose));
	}

	public Pose3d getPose3d() {
		return pose;
	}

	public Pose2d getPose2d() {
		return pose.toPose2d();
	}

	public Time getTimeStamp() {
		return timeStamp;
	}
}
