package frc.lib.mechviz.mechanism;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.ErrorMessages;
import frc.lib.mechviz.mechanism.mech.Mechanism3d;
import frc.lib.util.Loopable;
import frc.lib.util.LoopableHandler;
import java.util.ArrayList;
import java.util.HashMap;

public class MechanismRegistry implements Loopable {

	private final NetworkTable publishingTable;

	private final HashMap<Mechanism3d, StructPublisher<Pose3d>> mechToPublisherMap = new HashMap<>();
	private final ArrayList<Pair<Mechanism3d, StructPublisher<Pose3d>>> mechanismsBuffer = new ArrayList<>();

	private final LoopableHandler mechUpdateLoopableHandler = new LoopableHandler();

	public MechanismRegistry(NetworkTable publishingTable) {
		ErrorMessages.requireNonNullParam(publishingTable, "publishingTable", "MechanismRegistory");
		this.publishingTable = publishingTable;
		mechUpdateLoopableHandler.blacklist(this);
	}

	public StructPublisher<Pose3d> createPose3dPublisher(String key) {
		ErrorMessages.requireNonNullParam(key, "key", "createPose3dPublisher");
		return publishingTable.getStructTopic(key, Pose3d.struct).publish();
	}

	public void addLoop(Loopable loopable) {
		ErrorMessages.requireNonNullParam(loopable, "loopable", "addLoop");
		mechUpdateLoopableHandler.addLoop(loopable);
	}

	public void registerPublishedMechanism(String key, Mechanism3d mechanism) {
		ErrorMessages.requireNonNullParam(key, "key", "registerPublishedMechanism");
		ErrorMessages.requireNonNullParam(mechanism, "mechanism", "registerPublishedMechanism");
		associateMechanismWithPublisher(key, mechanism);
		mechanismsBuffer.add(Pair.of(mechanism, mechToPublisherMap.get(mechanism)));
	}

	public void associateMechanismWithPublisher(String key, Mechanism3d mechanism) {
		mechToPublisherMap.put(mechanism, createPose3dPublisher(key));
	}

	public void publish() {
		for (Pair<Mechanism3d, StructPublisher<Pose3d>> pair : mechanismsBuffer) {
			Mechanism3d mechanism = pair.getFirst();
			StructPublisher<Pose3d> publisher = pair.getSecond();
			publisher.accept(mechanism.getPivotedPose());
		}
	}

	public NetworkTable getPublishingTable() {
		return publishingTable;
	}

	@Override
	public void loop() {
		mechUpdateLoopableHandler.loop();
		publish();
	}
}
