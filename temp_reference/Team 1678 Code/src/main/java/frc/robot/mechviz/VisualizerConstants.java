package frc.robot.mechviz;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisualizerConstants {

	public static final NetworkTable PUBLISHING_TABLE =
			NetworkTableInstance.getDefault().getTable("Rebuilt Mechanisms");
}
