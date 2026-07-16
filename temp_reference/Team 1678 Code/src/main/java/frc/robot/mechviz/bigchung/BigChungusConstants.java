package frc.robot.mechviz.bigchung;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.mechviz.VisualizedRobot;
import frc.lib.mechviz.mechanism.mech.PivotingMechanism3d;
import frc.lib.util.ForwardDirection;
import frc.lib.util.MathHelpers;
import frc.lib.util.builder.Transform3dObjectBuilder;
import frc.robot.mechviz.bigchung.BigChungusIntakeMechanism3dHandler.BigChungusIntakeMechanism3dHandlerConfig;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intakedeploy.IntakeDeployConstants;
import frc.robot.tracking.IntakeHopperTrackerConstants;

public class BigChungusConstants {

	public static final NetworkTable PUBLISHING_TABLE =
			NetworkTableInstance.getDefault().getTable("Big Chungus");

	public static class BigChungusDriveConstants {
		public static final Pose3d OFFSET_POSE = Pose3d.kZero;

		public static PivotingMechanism3d getMechanism3d() {
			return new PivotingMechanism3d(OFFSET_POSE, () -> MathHelpers.transform3d(Drive.mInstance.getPose()));
		}
	}

	public static final class BigChungusHoodConstants {

		public static final Pose3d OFFSET_POSE = new Pose3d(0.295275, 0.0, -0.450950, Rotation3d.kZero).times(-1);

		public static PivotingMechanism3d getMechanism3d() {
			return new PivotingMechanism3d(OFFSET_POSE, () -> new Transform3dObjectBuilder()
					.withPitch(Hood.mInstance
							.getPosition()
							.minus(HoodConstants.kMinAngle)
							.unaryMinus())
					.build());
		}
	}

	public static final class BigChungusClimberConstants {

		public static final Pose3d OFFSET_POSE = new Pose3d();

		public static final PivotingMechanism3d getMechanism3d() {
			return new PivotingMechanism3d(OFFSET_POSE, () -> new Transform3dObjectBuilder()
					.withOffsetZ(ClimberConstants.converter.toDistance(Climber.mInstance.getPosition()))
					.build());
		}
	}

	public static final class BigChungusIntake {
		public static final Pose3d DEPLOY_OFFSET_POSE =
				new Pose3d(-0.309296, 0.0, -0.169010, Rotation3d.kZero).times(-1.0);
		public static final Pose3d EXTENSION_OFFSET_POSE = new Pose3d();

		public static final Pose3d FLOOR_OFFSET_POSE =
				new Pose3d(0.063648, 0.0, -0.131363, Rotation3d.kZero).times(-1.0);

		public static final Pose3d BRIDGE_OFFSET_POSE_WITHOUT_Y =
				new Pose3d(-0.246111, 0.0, -0.225568, Rotation3d.kZero).times(-1.0);

		public static final Distance BRIDGE_Y_ABS_OFFSET = Meters.of(0.296875);

		public static final Distance FLOOR_ARM_DISTANCE = Inches.of(12.195276);

		public static final double EXTENSION_GEARING = IntakeDeployConstants.HOPPER_MAX_EXTENSION_DISTANCE.in(Meters)
				/ IntakeDeployConstants.kStowedAngle.in(Degrees);

		public static final BigChungusIntakeMechanism3dHandler getHandler() {
			BigChungusIntakeMechanism3dHandlerConfig config = new BigChungusIntakeMechanism3dHandlerConfig();
			config.deployOffset = DEPLOY_OFFSET_POSE;
			config.extensionOffset = EXTENSION_OFFSET_POSE;
			config.floorOffset = FLOOR_OFFSET_POSE;
			config.angleDirection = ForwardDirection.NEGATIVE;
			config.translationDirection = ForwardDirection.POSITIVE;
			config.bridgeOffsetWithYDirection = BRIDGE_OFFSET_POSE_WITHOUT_Y;
			config.bridgeYAbsoluteOffset = BRIDGE_Y_ABS_OFFSET;
			config.floorArmLength = IntakeDeployConstants.HOPPER_FLOOR_ARM_LENGTH;
			config.intakeArmLength = IntakeDeployConstants.kArmLength;
			config.bridgeArmLength = IntakeHopperTrackerConstants.BRIDGE_ARM_LENGTH;
			return new BigChungusIntakeMechanism3dHandler(config);
		}
	}

	public static final VisualizedRobot allocateChungusVisualizedRobot() {
		VisualizedRobot robot = new VisualizedRobot("Big Chungus", PUBLISHING_TABLE, "Big Debug");
		robot.setDrive(BigChungusDriveConstants.getMechanism3d());
		robot.registerMechanism("Hood", BigChungusHoodConstants.getMechanism3d());
		robot.registerMechanism("Climber", BigChungusClimberConstants.getMechanism3d());

		robot.registerMechanism(
				(index, mech) -> {
					String baseName = "Intake", baseBridgeName = "Bridge";
					return switch (index) {
						case 0 -> baseName + "/Deploy";
						case 1 -> baseName + "/Extension";
						case 2 -> baseName + "/Floor";
						case 3 -> baseName + '/' + baseBridgeName + "/Left";
						case 4 -> baseName + '/' + baseBridgeName + "/Right";
						default -> {
							SmartDashboard.putNumber(
									"Last Unable to Add mechanism for Intake", Timer.getFPGATimestamp());
							throw new RuntimeException("Unable to Log values");
						}
					};
				},
				BigChungusIntake.getHandler());

		return robot;
	}
}
