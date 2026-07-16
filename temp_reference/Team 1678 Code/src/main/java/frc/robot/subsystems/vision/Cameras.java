package frc.robot.subsystems.vision;

import frc.lib.bases.CameraSubsystem;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class Cameras extends CameraSubsystem {

	public static final Cameras mInstance = new Cameras();

	public Cameras() {
		super(CamerasConstants.getConfig());

		if (Robot.isSimulation() && !RobotConstants.simulateVision) {
			disable();
		}
	}
}
