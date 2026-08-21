package frc.lib.bases;

import edu.wpi.first.networktables.NetworkTable;
import frc.lib.mechviz.VisualizedRobot;
import frc.lib.util.Loopable;
import java.util.ArrayList;
import java.util.List;

public abstract class RobotVisualizer implements Loopable {

	private final NetworkTable publishingTable;
	private final List<VisualizedRobot> registeredRobots = new ArrayList<>();

	public RobotVisualizer(NetworkTable publishingTable) {
		this.publishingTable = publishingTable;
	}

	public void registerRobot(VisualizedRobot robot) {
		registeredRobots.add(robot);
	}

	public abstract void registerRobots();

	public void init() {
		registerRobots();
	}

	@Override
	public void loop() {
		for (VisualizedRobot robot : registeredRobots) {
			robot.loop();
		}
	}

	public NetworkTable getPublishingTable() {
		return publishingTable;
	}
}
