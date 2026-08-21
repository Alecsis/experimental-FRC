package frc.lib.util;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.logging.LogUtil;
import frc.robot.RobotConstants;
import java.util.ArrayList;
import java.util.function.Supplier;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in
 * Meters</b> <br>
 * <br>
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldLayout {
	public enum FieldType {
		Andymark,
		Welded
	}

	public enum FieldArea {
		RED_DEEP,
		RED_SHALLOW,
		NEUTRAL,
		BLUE_SHALLOW,
		BLUE_DEEP;
	}

	public static final Distance kFieldLength = (RobotConstants.currentFieldType == FieldType.Andymark)
			? Units.Feet.of(54.0).plus(Units.Inches.of(2.12))
			: Units.Feet.of(54.0).plus(Units.Inches.of(3.2));

	public static final Distance kFieldWidth = (RobotConstants.currentFieldType == FieldType.Andymark)
			? Units.Feet.of(26.0).plus(Units.Inches.of(4.64))
			: Units.Feet.of(26.0).plus(Units.Inches.of(5.7));

	public static final AprilTagFieldLayout kAprilTagMap = (RobotConstants.currentFieldType == FieldType.Andymark)
			? AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark)
			: AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

	public static final Distance kAprilTagWidth = Units.Inches.of(6.5);

	public static final Distance kHubAltitude = Units.Inches.of(72.0);
	public static final Distance kHubRadius = Units.Meters.of(0.56);
	public static final Distance kGroundAltitude = Units.Inches.zero();

	public static final Distance kBlueAllianceLine = (RobotConstants.currentFieldType == FieldType.Andymark)
			? Units.Feet.of(13.0).plus(Units.Inches.of(0.06))
			: Units.Feet.of(13.0).plus(Units.Inches.of(0.61));

	public static final Translation2d kBlueHub = (RobotConstants.currentFieldType == FieldType.Andymark)
			? new Translation2d(
					Units.Feet.of(15.0).plus(Units.Inches.of(1.56)),
					Units.Feet.of(13.0).plus(Units.Inches.of(2.32)))
			: new Translation2d(
					Units.Feet.of(15.0).plus(Units.Inches.of(2.11)),
					Units.Feet.of(13.0).plus(Units.Inches.of(2.85)));

	public static final Translation2d kBlueDepotCenter = (RobotConstants.currentFieldType == FieldType.Andymark)
			? new Translation2d(
					Units.Feet.of(1.0).plus(Units.Inches.of(1.5)),
					Units.Feet.of(19.0).plus(Units.Inches.of(6.25)))
			: new Translation2d(
					Units.Feet.of(1.0).plus(Units.Inches.of(1.5)),
					Units.Feet.of(19.0).plus(Units.Inches.of(6.77)));

	public static final Translation2d kBlueOutpost = (RobotConstants.currentFieldType == FieldType.Andymark)
			? new Translation2d(Units.Meters.zero(), Units.Feet.of(2.0).plus(Units.Inches.of(1.62)))
			: new Translation2d(Units.Meters.zero(), Units.Feet.of(2.0).plus(Units.Inches.of(2.22)));

	public static final Translation2d kBlueTowerFaceCenter = (RobotConstants.currentFieldType == FieldType.Andymark)
			? new Translation2d(
					Units.Feet.of(3.0).plus(Units.Inches.of(4.045)),
					Units.Feet.of(12.0).plus(Units.Inches.of(2.86)))
			: new Translation2d(
					Units.Feet.of(3.0).plus(Units.Inches.of(3.805)),
					Units.Feet.of(12.0).plus(Units.Inches.of(3.46)));

	public static final Translation2d kBlueTowerLeftRung = (RobotConstants.currentFieldType == FieldType.Andymark)
			? new Translation2d(
					Units.Feet.of(3.0).plus(Units.Inches.of(4.045)),
					Units.Feet.of(14.0).plus(Units.Inches.of(2.9607)))
			: new Translation2d(
					Units.Feet.of(3.0).plus(Units.Inches.of(3.805)),
					Units.Feet.of(14.0)
							.plus(Units.Inches.of(2.96))); // left side of the rungs (while looking out DS window)

	public static final Translation2d kBlueTowerRightRung = (RobotConstants.currentFieldType == FieldType.Andymark)
			? new Translation2d(
					Units.Feet.of(3.0).plus(Units.Inches.of(4.045)),
					Units.Feet.of(10.0).plus(Units.Inches.of(2.36)))
			: new Translation2d(
					Units.Feet.of(3.0).plus(Units.Inches.of(3.805)),
					Units.Feet.of(10.0)
							.plus(Units.Inches.of(3.96))); // right side of the rungs (while looking out DS window)

	// Depot-Side Alliance Corner
	public static final Translation2d kBlueLeftFerry = (RobotConstants.currentFieldType == FieldType.Andymark)
			? new Translation2d(Units.Meters.of(0.5), Units.Feet.of(15.5))
			: new Translation2d(Units.Meters.of(0.5), Units.Feet.of(15.5));

	// Outpost-Side Alliance Corner
	public static final Translation2d kBlueRightFerry = (RobotConstants.currentFieldType == FieldType.Andymark)
			? new Translation2d(Units.Meters.of(0.5), Units.Feet.of(8.0))
			: new Translation2d(Units.Meters.of(0.5), Units.Feet.of(8.0));

	// Depot-Side Alliance Trench
	public static final Translation2d kBlueLeftNeutralFerry = (RobotConstants.currentFieldType == FieldType.Andymark)
			? new Translation2d(Units.Meters.of(5.5), Units.Feet.of(18.0))
			: new Translation2d(Units.Meters.of(5.5), Units.Feet.of(18.0));

	// Outpost-Side Alliance Trench
	public static final Translation2d kBlueRightNeutralFerry = (RobotConstants.currentFieldType == FieldType.Andymark)
			? new Translation2d(Units.Meters.of(5.5), Units.Feet.of(9.0))
			: new Translation2d(Units.Meters.of(5.5), Units.Feet.of(9.0));

	// Depot-Side Alliance Trench
	public static final Translation2d kBlueLeftFarAllianceFerry =
			(RobotConstants.currentFieldType == FieldType.Andymark)
					? new Translation2d(Units.Meters.of(0.5), Units.Feet.of(20.0))
					: new Translation2d(Units.Meters.of(0.5), Units.Feet.of(20.0));

	// Outpost-Side Alliance Trench
	public static final Translation2d kBlueRightFarAllianceFerry =
			(RobotConstants.currentFieldType == FieldType.Andymark)
					? new Translation2d(Units.Meters.of(0.5), Units.Feet.of(5.0))
					: new Translation2d(Units.Meters.of(0.5), Units.Feet.of(5.0));

	public static final Translation3d kBlueHubExit = RobotConstants.currentFieldType == FieldType.Andymark
			? new Translation3d(kBlueHub.getMeasureX(), kBlueHub.getMeasureY(), Units.Inches.of(30.0))
			: new Translation3d(kBlueHub.getMeasureX(), kBlueHub.getMeasureY(), Units.Inches.of(30.0));

	public static final Distance kBlueLeftTrenchX = (RobotConstants.currentFieldType == FieldType.Andymark)
			? Units.Feet.of(15.0).plus(Units.Inches.of(2.11))
			: Units.Feet.of(15.0).plus(Units.Inches.of(2.11));

	public static final Distance kBlueLeftTrenchY = (RobotConstants.currentFieldType == FieldType.Andymark)
			? Units.Feet.of(24.0).plus(Units.Inches.of(4.718))
			: Units.Feet.of(24.0).plus(Units.Inches.of(4.31));

	public static final Distance kTrenchWidth = Feet.of(4.25);
	public static final Distance TRENCH_NZ_DETECTION_LENGTH = Feet.of(5.0);
	public static final Distance TRENCH_AZ_DETECTION_LENGTH = Feet.of(1.0);

	public enum ClimbLocation {
		A(
				new Translation2d(Units.Meters.of(0.5), Units.Meters.of(-0.6)),
				Units.Degrees.of(5.0),
				Units.Centimeters.of(12.0),
				Units.Centimeters.of(12.0),
				Units.Degrees.of(-30.0),
				Units.Centimeters.of(10.0),
				Units.Centimeters.of(1.0),
				Units.Centimeters.of(1.0),
				Units.Degrees.of(1.5),
				Units.MetersPerSecond.of(2.0)),
		C(
				new Translation2d(Units.Meters.of(0.6), Units.Meters.of(0.0)),
				Units.Degrees.of(5.0),
				Units.Centimeters.of(1.0),
				Units.Centimeters.of(10.0),
				Units.Degrees.of(0.0),
				Units.Centimeters.of(2.0),
				Units.Centimeters.of(1.0),
				Units.Centimeters.of(5.0),
				Units.Degrees.of(0.75),
				Units.MetersPerSecond.of(0.0)),
		E(
				new Translation2d(Units.Meters.of(0.5), Units.Meters.of(0.3)),
				Units.Degrees.of(5.0),
				Units.Centimeters.of(12.0),
				Units.Centimeters.of(12.0),
				Units.Degrees.of(0.0),
				Units.Centimeters.of(10.0),
				Units.Centimeters.of(1.0),
				Units.Centimeters.of(1.0),
				Units.Degrees.of(1.5),
				Units.MetersPerSecond.of(-2.0));

		public final Translation2d safetyTranslation;
		public final Angle safetyRotationTolerance;
		public final Distance safetyXTolerance;
		public final Distance safetyYTolerance;
		public final Angle safetyAngle;
		public final Distance distTolerance;
		public final Distance xTolerance;
		public final Distance yTolerance;
		public final Angle rotationTolerance;
		public final LinearVelocity yV;

		private ClimbLocation(
				Translation2d safetyTranslation,
				Angle safetyRotationTolerance,
				Distance safetyXTolerance,
				Distance safetyYTolerance,
				Angle safetyAngle,
				Distance distTolerance,
				Distance xTolerance,
				Distance yTolerance,
				Angle rotationTolerance,
				LinearVelocity yV) {
			this.safetyTranslation = safetyTranslation;
			this.safetyRotationTolerance = safetyRotationTolerance;
			this.safetyXTolerance = safetyXTolerance;
			this.safetyYTolerance = safetyYTolerance;
			this.safetyAngle = safetyAngle;
			this.distTolerance = distTolerance;
			this.xTolerance = xTolerance;
			this.yTolerance = yTolerance;
			this.rotationTolerance = rotationTolerance;
			this.yV = yV;
		}
	}

	public static AprilTag[] getAprilTagArrayFromIDs(int ids[]) {
		AprilTag buffer[] = new AprilTag[ids.length];
		int offset = 0;
		for (AprilTag tag : kAprilTagMap.getTags()) {
			for (int id : ids)
				if (id == tag.ID) {
					buffer[offset++] = tag;
					break;
				}
		}
		return buffer;
	}

	public static AprilTag getAprilTagByID(int id) {
		for (AprilTag tag : kAprilTagMap.getTags()) if (tag.ID == id) return tag;
		return null;
	}

	public static AprilTag[] getTagsOnRobotAlliance() {
		ArrayList<AprilTag> tags = new ArrayList<>();
		for (AprilTag tag : kAprilTagMap.getTags()) if (isPoseOnRobotAlliance(tag.pose.toPose2d())) tags.add(tag);
		AprilTag buffer[] = new AprilTag[tags.size()];
		tags.toArray(buffer);
		return buffer;
	}

	public static Pose3d[] getPoseArrayFromAprilTagArray(AprilTag tags[]) {
		Pose3d poses[] = new Pose3d[tags.length];
		for (int i = 0; i < tags.length; i++) poses[i] = tags[i].pose;
		return poses;
	}

	public static Integer[] getIDArrayFromAprilTagArray(AprilTag tags[]) {
		Integer buffer[] = new Integer[tags.length];
		for (int i = 0; i < tags.length; i++) buffer[i] = tags[i].ID;
		return buffer;
	}

	public static boolean isPoseOnRedSide(Pose2d pose) {
		return !pose.getMeasureX().lt(kFieldLength.div(2));
	}

	public static boolean isPoseOnRobotAlliance(Pose2d pose, boolean isRedAlliance) {
		return isPoseOnRedSide(pose) && isRedAlliance;
	}

	public static boolean isPoseOnRobotAlliance(Pose2d pose) {
		return isPoseOnRobotAlliance(pose, RobotConstants.isRedAlliance);
	}

	public static boolean isAprilTagOnRedAlliance(AprilTag tag, boolean isRedAlliance) {
		return isPoseOnRobotAlliance(tag.pose.toPose2d(), isRedAlliance);
	}

	public static boolean isAprilTagOnRobotAlliance(AprilTag tag) {
		return isAprilTagOnRedAlliance(tag, RobotConstants.isRedAlliance);
	}

	public static boolean isPoseWithinAllianceZone(boolean isRedAlliance, Pose2d pose) {
		return distanceFromAllianceWall(pose.getMeasureX(), isRedAlliance).lte(kBlueAllianceLine.plus(Feet.of(4.0)));
	}

	public static boolean isPoseWithinTrenchY(Pose2d pose) {
		return (pose.getMeasureY().lte(kFieldWidth.div(2)))
				? pose.getMeasureY().lte(kTrenchWidth) // Bottom side of field
				: pose.getMeasureY().gte(kFieldWidth.minus(kTrenchWidth)); // Top side of field
	}

	public static boolean isPoseWithinTrenchX(Pose2d pose) {
		boolean poseOnBlueSide = !isPoseOnRedSide(pose);

		Distance trenchCenter = poseOnBlueSide ? kBlueLeftTrenchX : flipAcrossX(kBlueLeftTrenchX);

		return pose.getMeasureX()
						.gte(trenchCenter.minus(
								poseOnBlueSide
										? TRENCH_AZ_DETECTION_LENGTH // Logic for adusting tollerences when going in vs
										// out
										// of NZ
										: TRENCH_NZ_DETECTION_LENGTH))
				&& pose.getMeasureX()
						.lte(trenchCenter.plus(
								poseOnBlueSide ? TRENCH_NZ_DETECTION_LENGTH : TRENCH_AZ_DETECTION_LENGTH));
	}

	public static boolean isPoseWithinTrench(boolean isRedAlliance, Pose2d pose) {
		return isPoseWithinTrenchY(pose) && isPoseWithinTrenchX(pose);
	}

	public static boolean isPoseOnLeftSide(boolean isRedAlliance, Distance y_coordinate) {
		if (isRedAlliance) {
			return y_coordinate.lte(kFieldWidth.div(2.0));
		} else {
			return y_coordinate.gte(kFieldWidth.div(2.0));
		}
	}

	public static Pose2d handleAllianceFlip(Pose2d blue_pose, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_pose = rotateAboutCenter(blue_pose, Rotation2d.k180deg);
		}
		return blue_pose;
	}

	public static Translation2d handleAllianceFlip(Translation2d blue_translation, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_translation = blue_translation.rotateAround(
					new Translation2d(kFieldLength.div(2.0), kFieldWidth.div(2.0)), Rotation2d.k180deg);
		}
		return blue_translation;
	}

	public static Translation3d handleAllianceFlip(Translation3d blue_translation, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_translation = blue_translation.rotateAround(
					new Translation3d(kFieldLength.div(2.0), kFieldWidth.div(2.0), blue_translation.getMeasureY()),
					new Rotation3d(Rotation2d.k180deg));
		}
		return blue_translation;
	}

	public static Rotation2d handleAllianceFlip(Rotation2d blue_rotation, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_rotation = blue_rotation.plus(Rotation2d.k180deg);
		}
		return blue_rotation;
	}

	public static Distance distanceFromAllianceWall(Distance x_coordinate, boolean is_red_alliance) {
		if (is_red_alliance) {
			return kFieldLength.minus(x_coordinate);
		}
		return x_coordinate;
	}

	public static Translation2d mirrorAboutX(Translation2d t, Distance xValue) {
		return new Translation2d(xValue.in(Units.Meters) + (xValue.in(Units.Meters) - t.getX()), t.getY());
	}

	public static Translation2d mirrorAboutY(Translation2d t, Distance yValue) {
		return new Translation2d(t.getX(), yValue.in(Units.Meters) + (yValue.in(Units.Meters) - t.getY()));
	}

	public static Rotation2d mirrorAboutX(Rotation2d r) {
		return new Rotation2d(-r.getCos(), r.getSin());
	}

	public static Rotation2d mirrorAboutY(Rotation2d r) {
		return new Rotation2d(r.getCos(), -r.getSin());
	}

	public static Pose2d mirrorAboutX(Pose2d p, Distance xValue) {
		return new Pose2d(mirrorAboutX(p.getTranslation(), xValue), mirrorAboutX(p.getRotation()));
	}

	public static Pose2d mirrorAboutY(Pose2d p, Distance yValue) {
		return new Pose2d(mirrorAboutY(p.getTranslation(), yValue), mirrorAboutY(p.getRotation()));
	}

	public static Pose2d rotateAboutPose(Pose2d startPose, Translation2d point, Rotation2d rotation) {
		return new Pose2d(
				startPose.getTranslation().rotateAround(point, rotation),
				startPose.getRotation().plus(rotation));
	}

	public static Translation2d flipAboutMidline(Translation2d translation) {
		return new Translation2d(kFieldLength.minus(translation.getMeasureX()), translation.getMeasureY());
	}

	public static Translation2d flipAcrossY(Translation2d translation) {
		return new Translation2d(translation.getMeasureX(), kFieldWidth.minus(translation.getMeasureY()));
	}

	public static Pose2d flipAboutMidline(Pose2d pose) {
		return new Pose2d(kFieldLength.minus(pose.getMeasureX()), pose.getMeasureY(), pose.getRotation());
	}

	public static Pose2d flipAcrossY(Pose2d pose) {
		return new Pose2d(pose.getMeasureX(), kFieldWidth.minus(pose.getMeasureY()), pose.getRotation());
	}

	public static Distance flipAcrossX(Distance d) {
		return kFieldLength.minus(d);
	}

	public static Distance flipAcrossY(Distance d) {
		return kFieldWidth.minus(d);
	}

	public static AprilTag[] getAllianceAprilTagArray(boolean isRedAlliance) {
		if (isRedAlliance) {
			return redTags;
		} else {
			return blueTags;
		}
	}

	public static final AprilTag redTags[] = new AprilTag[] {
		getAprilTagByID(2),
		getAprilTagByID(5),
		getAprilTagByID(7),
		getAprilTagByID(8),
		getAprilTagByID(9),
		getAprilTagByID(10),
		getAprilTagByID(11),
		getAprilTagByID(12),
		getAprilTagByID(13),
		getAprilTagByID(14),
		getAprilTagByID(15),
		getAprilTagByID(16)
	};

	public static final AprilTag blueTags[] = new AprilTag[] {
		getAprilTagByID(18),
		getAprilTagByID(21),
		getAprilTagByID(23),
		getAprilTagByID(24),
		getAprilTagByID(25),
		getAprilTagByID(26),
		getAprilTagByID(27),
		getAprilTagByID(28),
		getAprilTagByID(29),
		getAprilTagByID(30),
		getAprilTagByID(31),
		getAprilTagByID(32),
	};

	public static Pose2d rotateAboutCenter(Pose2d startPose, Rotation2d rotation) {
		return rotateAboutPose(startPose, new Translation2d(kFieldLength.div(2.0), kFieldWidth.div(2.0)), rotation);
	}

	public static final Distance fieldCrop = Units.Feet.of(-0.5);

	public static boolean outsideField(Pose2d pose) {
		return pose.getMeasureX().gte(FieldLayout.kFieldLength.minus(fieldCrop))
				|| pose.getMeasureX().lte(Units.Meters.zero().plus(fieldCrop))
				|| pose.getMeasureY().gte(FieldLayout.kFieldWidth.minus(fieldCrop))
				|| pose.getMeasureY().lte(Units.Meters.zero().plus(fieldCrop));
	}

	public static Supplier<Translation2d> getAllianceHubTranslation(boolean isRedAlliance) {
		return () -> handleAllianceFlip(kBlueHub, isRedAlliance);
	}

	public static Supplier<Translation2d> getAllianceFerryTranslation(
			boolean isRedAlliance, boolean isLeft, FieldArea fieldArea) {
		if (fieldArea == FieldArea.RED_DEEP && isRedAlliance || fieldArea == FieldArea.BLUE_DEEP && !isRedAlliance) {
			if (isLeft) {
				return () -> handleAllianceFlip(kBlueLeftFerry, isRedAlliance);
			} else {
				return () -> handleAllianceFlip(kBlueRightFerry, isRedAlliance);
			}
		} else {
			if (fieldArea == FieldArea.RED_SHALLOW && isRedAlliance
					|| fieldArea == FieldArea.BLUE_SHALLOW && !isRedAlliance) {
				if (isLeft) {
					return () -> handleAllianceFlip(kBlueLeftFarAllianceFerry, isRedAlliance);
				} else {
					return () -> handleAllianceFlip(kBlueRightFarAllianceFerry, isRedAlliance);
				}
			} else {
				if (isLeft) {
					return () -> handleAllianceFlip(kBlueLeftNeutralFerry, isRedAlliance);
				} else {
					return () -> handleAllianceFlip(kBlueRightNeutralFerry, isRedAlliance);
				}
			}
		}
	}

	public static final Distance getFieldWidthMidline() {
		return kFieldWidth.div(2.0);
	}

	public static void publishPoses() {
		LogUtil.log("Field Layout/Blue Hub", kBlueHub);
		LogUtil.log("Field Layout/Blue Depot Center", kBlueDepotCenter);
		LogUtil.log("Field Layout/Blue Outpost", kBlueOutpost);
		LogUtil.log("Field Layout/Blue Tower Face Center", kBlueTowerFaceCenter);
		LogUtil.log("Field Layout/Blue Tower Left Rung", kBlueTowerLeftRung);
		LogUtil.log("Field Layout/Blue Tower Right Rung", kBlueTowerRightRung);
	}
}
