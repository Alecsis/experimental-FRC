package frc.lib.mechviz.mechanism;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.Robot;
import java.util.HashMap;

public class DevelopmentHandler {

	private final NetworkTable publishingTable;
	private final CommandScheduler commandScheduler = CommandScheduler.getInstance();

	private final HashMap<String, StructPublisher<Pose3d>> publishedEntries = new HashMap<>();

	public DevelopmentHandler(NetworkTable publishingTable) {
		this.publishingTable = publishingTable;
	}

	public void publishPose(String key, Pose3d pose) {
		if (publishedEntries.containsKey(key)) {
			publishedEntries.get(key).set(pose);
		} else {
			StructPublisher<Pose3d> publisher =
					publishingTable.getStructTopic(key, Pose3d.struct).publish();
			publisher.set(pose);
			publishedEntries.put(key, publisher);
		}
	}

	public void publishEmptyPose3d(String key) {
		publishPose(key, Pose3d.kZero);
	}

	public void applyDefaultCommand(Subsystem subsystem, Command command) {
		commandScheduler.setDefaultCommand(subsystem, command.onlyIf(() -> Robot.isSimulation()));
	}

	public void applyDefaultCommand(ServoMotorSubsystem<? extends MotorIO> servo, Setpoint setpoint, boolean wantWait) {
		applyDefaultCommand(
				servo, wantWait ? servo.setpointCommand(setpoint) : servo.setpointCommandWithWait(setpoint));
	}

	public void applyDefaultCommand(ServoMotorSubsystem<? extends MotorIO> servo, Setpoint setpoint) {
		applyDefaultCommand(servo, setpoint, false);
	}
}
