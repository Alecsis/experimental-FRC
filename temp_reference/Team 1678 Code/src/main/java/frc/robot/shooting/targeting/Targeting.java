package frc.robot.shooting.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import frc.lib.util.FieldLayout;
import frc.robot.RobotConstants;
import frc.robot.shooting.Shooting;
import frc.robot.shooting.Shooting.ShootingState;
import frc.robot.subsystems.drive.Drive;

public class Targeting {

	private final NetworkTableInstance instance = NetworkTableInstance.getDefault();

	private final NetworkTable targettingTable = instance.getTable("SmartDashboard/Targeting");
	private final StructPublisher<Pose2d> poseTarget =
			targettingTable.getStructTopic("Target Position", Pose2d.struct).publish();

	private Translation2d targetTranslation = new Translation2d();

	public Targeting() {}

	public void periodic() {
		// avoid target changing
		if (!Shooting.state.isFiring()) {
			calculateTarget();
		}
	}

	private void calculateTarget() {
		if (FieldLayout.isPoseWithinAllianceZone(RobotConstants.isRedAlliance, Drive.mInstance.getPose())) {
			Shooting.state = ShootingState.HUB;
			targetTranslation = Shooting.getState().getLocationTranslation(RobotConstants.isRedAlliance);
			// Cameras.mInstance.setPipeline(
			// 		RobotConstants.isRedAlliance
			// 				? PipelineConstants.APRIL_TAG_HUB_RED
			// 				: PipelineConstants.APRIL_TAG_HUB_BLUE);
		} else if (FieldLayout.isPoseOnLeftSide(
				RobotConstants.isRedAlliance, Drive.mInstance.getPose().getMeasureY())) {
			if (FieldLayout.distanceFromAllianceWall(
							Drive.mInstance.getPose().getMeasureX(), RobotConstants.isRedAlliance)
					.lte(FieldLayout.kFieldLength.minus(Units.Feet.of(13.0)))) {
				Shooting.state = ShootingState.LEFT_ALLIANCE_FERRY;
				targetTranslation = Shooting.getState().getLocationTranslation(RobotConstants.isRedAlliance);
				// Cameras.mInstance.setPipeline(PipelineConstants.APRIL_TAG_HUB_FIELD);
			} else {
				if (FieldLayout.distanceFromAllianceWall(
								Drive.mInstance.getPose().getMeasureX(), RobotConstants.isRedAlliance)
						.lte(FieldLayout.kFieldLength.minus(Units.Feet.of(6.5)))) {
					Shooting.state = ShootingState.LEFT_FAR_ALLIANCE_FERRY;
					targetTranslation = Shooting.getState().getLocationTranslation(RobotConstants.isRedAlliance);
				} else {
					Shooting.state = ShootingState.LEFT_NEUTRAL_FERRY;
					targetTranslation = Shooting.getState().getLocationTranslation(RobotConstants.isRedAlliance);
					// Cameras.mInstance.setPipeline(PipelineConstants.APRIL_TAG_HUB_FIELD);
				}
			}
		} else {
			if (FieldLayout.distanceFromAllianceWall(
							Drive.mInstance.getPose().getMeasureX(), RobotConstants.isRedAlliance)
					.lte(FieldLayout.kFieldLength.minus(Units.Feet.of(13.0)))) {
				Shooting.state = ShootingState.RIGHT_ALLIANCE_FERRY;
				targetTranslation = Shooting.getState().getLocationTranslation(RobotConstants.isRedAlliance);
				// Cameras.mInstance.setPipeline(PipelineConstants.APRIL_TAG_HUB_FIELD);
			} else {
				if (FieldLayout.distanceFromAllianceWall(
								Drive.mInstance.getPose().getMeasureX(), RobotConstants.isRedAlliance)
						.lte(FieldLayout.kFieldLength.minus(Units.Feet.of(6.5)))) {
					Shooting.state = ShootingState.RIGHT_FAR_ALLIANCE_FERRY;
					targetTranslation = Shooting.getState().getLocationTranslation(RobotConstants.isRedAlliance);
				} else {
					Shooting.state = ShootingState.RIGHT_NEUTRAL_FERRY;
					targetTranslation = Shooting.getState().getLocationTranslation(RobotConstants.isRedAlliance);
					// Cameras.mInstance.setPipeline(PipelineConstants.APRIL_TAG_HUB_FIELD);
				}
			}
		}

		poseTarget.set(
				new Pose2d(targetTranslation, RobotConstants.isRedAlliance ? Rotation2d.kZero : Rotation2d.k180deg));
	}
}
