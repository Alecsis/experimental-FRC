package frc.robot.mechviz;

import frc.lib.bases.RobotVisualizer;
import frc.robot.mechviz.bigchung.BigChungusConstants;
import frc.robot.mechviz.comp.CompConstants;

public class RebuiltRobotVisualizer extends RobotVisualizer {

	public RebuiltRobotVisualizer() {
		super(VisualizerConstants.PUBLISHING_TABLE);
	}

	@Override
	public void registerRobots() {
		registerRobot(BigChungusConstants.allocateChungusVisualizedRobot());
		registerRobot(CompConstants.getCompVisualizedRobot());
	}
}
