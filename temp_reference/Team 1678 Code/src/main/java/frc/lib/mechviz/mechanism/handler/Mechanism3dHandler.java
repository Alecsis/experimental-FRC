package frc.lib.mechviz.mechanism.handler;

import frc.lib.mechviz.mechanism.mech.Mechanism3d;
import frc.lib.util.ArrayUtil;
import frc.lib.util.Loopable;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

public abstract class Mechanism3dHandler implements Loopable {

	private final List<Mechanism3d> registeredMechanims = new ArrayList<>();

	public Mechanism3dHandler() {}

	public void registerMech(Mechanism3d mech) {
		registeredMechanims.add(mech);
	}

	public void registerMech(Mechanism3d... mechs) {
		ArrayUtil.appendArrayToList(mechs, registeredMechanims);
	}

	public abstract void updatePositions();

	public void loop() {
		for (Mechanism3d mech : registeredMechanims) {
			mech.loop();
		}
		updatePositions();
	}

	public void foreachRegistered(BiConsumer<Integer, Mechanism3d> function) {
		for (int i = 0; i < registeredMechanims.size(); i++) {
			function.accept(i, registeredMechanims.get(i));
		}
	}

	public void foreachRegistered(Consumer<Mechanism3d> function) {
		foreachRegistered((_index, mech) -> function.accept(mech));
	}

	public List<Mechanism3d> getRegisteredMechanisms() {
		return registeredMechanims;
	}

	public int getRegisteredMechanimsSize() {
		return registeredMechanims.size();
	}
}
