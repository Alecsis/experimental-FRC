package frc.lib.io.vision.limelight;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.io.vision.CameraIO;
import frc.lib.util.vision.CameraPipeline;
import frc.lib.util.vision.LimelightHelpers;
import frc.robot.Robot;

public abstract class LimelightCameraIO extends CameraIO {

	public LimelightCameraIO(CameraIOConfig config) {
		super(config);
		if (Robot.isReal()) {
			LimelightHelpers.setCameraPose_RobotSpace(
					config.name,
					config.robotToCameraOffset.getX(),
					config.robotToCameraOffset.getY(),
					config.robotToCameraOffset.getZ(),
					config.robotToCameraOffset.getRotation().getMeasureX().in(Degrees),
					config.robotToCameraOffset.getRotation().getMeasureY().in(Degrees),
					config.robotToCameraOffset.getRotation().getMeasureZ().in(Degrees));
		}
	}

	@Override
	public void updatePipeline(CameraPipeline pipeline) {
		LimelightHelpers.setPipelineIndex(config.name, pipeline.index());
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		builder.addDoubleProperty("Heart Beat", () -> LimelightHelpers.getHeartbeat(config.name), null);
	}
}
