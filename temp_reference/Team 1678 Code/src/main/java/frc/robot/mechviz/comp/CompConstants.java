package frc.robot.mechviz.comp;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.logging.LogUtil;
import frc.lib.mechviz.VisualizedRobot;
import frc.lib.mechviz.mechanism.mech.Mechanism3d;
import frc.lib.mechviz.mechanism.mech.PivotingMechanism3d;
import frc.lib.util.ForwardDirection;
import frc.lib.util.MathHelpers;
import frc.lib.util.builder.Rotation3dObjectBuilder;
import frc.lib.util.builder.Transform3dObjectBuilder;
import frc.robot.Robot;
import frc.robot.mechviz.comp.CompHopperMechanism3dHandler.CompHopperMechanism3dHandlerConfig;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intakedeploy.IntakeDeploy;
import frc.robot.tracking.HopperTracker;
import java.util.function.BiFunction;

public class CompConstants {

	public static final NetworkTable PUBLISHING_TABLE =
			NetworkTableInstance.getDefault().getTable("Comp");

	public static final class DriveConstants {
		public static final Pose3d OFFSET_POSE = Pose3d.kZero;

		public static PivotingMechanism3d getMechanism3d() {
			return new PivotingMechanism3d(OFFSET_POSE, () -> MathHelpers.transform3d(Drive.mInstance.getPose()));
		}
	}

	public static final class IntakeDeployConstants {
		public static final Pose3d OFFSET_POSE = new Pose3d(-0.309, 0.309, -0.169, Rotation3d.kZero).times(-1);

		public static PivotingMechanism3d getMechanism3d() {
			return new PivotingMechanism3d(OFFSET_POSE, () -> new Transform3dObjectBuilder()
					.withPitch(IntakeDeploy.mInstance.getPosition().unaryMinus())
					.build());
		}
	}

	public static final class HopperConstants {
		public static final Pose3d EXPANSION_OFFSET_POSE = Pose3d.kZero, EXTENSION_OFFSET_POSE = Pose3d.kZero;

		public static CompHopperMechanism3dHandlerConfig getConfig() {
			CompHopperMechanism3dHandlerConfig config = new CompHopperMechanism3dHandlerConfig();
			config.expansionDirection = ForwardDirection.POSITIVE;
			config.expansionOffset = EXPANSION_OFFSET_POSE;
			config.extensionOffset = EXTENSION_OFFSET_POSE;
			config.extensionDirection = ForwardDirection.POSITIVE;
			config.extensionGetter = () -> HopperTracker.mInstance.getHopperExtension();
			config.expansionGetter = () -> ClimberConstants.converter.toDistance(Climber.mInstance.getPosition());
			return config;
		}

		public static BiFunction<Integer, Mechanism3d, String> getNamingHandler() {
			return (index, mech) -> {
				String tableName = "Hopper/";
				return switch (index) {
					case 0 -> tableName + "Expansion";
					case 1 -> tableName + "Extension";
					default -> {
						LogUtil.timestampError("Last Unable to Register Hopper Mechanism at index " + index);
						if (Robot.isReal()) {
							yield " ";
						} else throw new RuntimeException("Unable to Register Hopper Mechanism");
					}
				};
			};
		}

		public static CompHopperMechanism3dHandler getHandler() {
			return new CompHopperMechanism3dHandler(getConfig());
		}
	}

	public static final class CompHoodConstants {
		public static final Pose3d OFFSET_POSE = new Pose3d(
						0.295275,
						0.0,
						-0.450950,
						new Rotation3dObjectBuilder()
								.withPitch(HoodConstants.kMinAngle.unaryMinus())
								.build())
				.times(-1.0);

		public static PivotingMechanism3d getMechanism3d() {
			return new PivotingMechanism3d(OFFSET_POSE, () -> new Transform3dObjectBuilder()
					.withPitch(Hood.mInstance.getPosition().unaryMinus())
					.build());
		}
	}

	public static final VisualizedRobot getCompVisualizedRobot() {
		VisualizedRobot robot = new VisualizedRobot("Comp", PUBLISHING_TABLE, "Comp/Debug");
		robot.setDrive("Drive", DriveConstants.getMechanism3d());
		robot.registerMechanism("Intake Deploy", IntakeDeployConstants.getMechanism3d());
		robot.registerMechanism(HopperConstants.getNamingHandler(), HopperConstants.getHandler());
		robot.registerMechanism("Hood", CompHoodConstants.getMechanism3d());
		return robot;
	}
}
