package frc.lib.sim.field.rebuilt;

import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.logging.LogUtil;
import java.util.ArrayList;
import java.util.Iterator;

/**
 * Top-level field simulator that tracks and updates spawned gamepieces
 * (currently {@link Fuel}).
 */
public class FieldSim {
	public static final FieldSim mInstance = new FieldSim();

	private ArrayList<Fuel> fuels = new ArrayList<Fuel>();

	public FieldSim() {}

	/** Remove all spawned fuel from the simulation. */
	public void removeFuel() {
		fuels.clear();
	}

	/** Log the current positions of all fuel */
	public void logFuels() {
		LogUtil.log(
				"Fuel Simulation/Fuels", fuels.stream().map((fuel) -> fuel.pos).toArray(Translation3d[]::new));
	}

	public void update() {
		stepSim();
		logFuels();
	}

	/** Perform a full simulation step. */
	public void stepSim() {
		Iterator<Fuel> iterator = fuels.iterator();
		while (iterator.hasNext()) {
			Fuel fuel = iterator.next();
			if (fuel.shouldBeRemoved) {
				iterator.remove();
			}
		}

		for (int i = 0; i < FieldSimConstants.kFuelSubticks; i++) {
			for (Fuel fuel : fuels) {
				fuel.update();
			}
		}
	}

	/** Spawn a new fuel at the given position and velocity. */
	public void generateFuel(Translation3d position, Translation3d velocity) {
		fuels.add(new Fuel(position, velocity));
	}
}
