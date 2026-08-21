package frc.lib.sim.field;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Base simulation class for a generic gamepiece. Stores position and velocity
 * and provides an update hook for subclasses to implement discrete physics
 * integration.
 * <p>
 * Subclasses (for example, {@code Fuel}) should override {@link #update()} to
 * apply forces, collision checks, and lifecycle rules.
 */
public class GamepieceSim {
	/** Current position (x, y, z). */
	public Translation3d pos;

	/** Current velocity (x, y, z).
	 * This is a Translation3d instead of a linear velocity
	 * to express multiple components
	 */
	public Translation3d vel;

	/**
	 * Create a gamepiece with the given position and velocity.
	 *
	 * @param pos  initial position (meters)
	 * @param vel  initial velocity (meters/second)
	 */
	public GamepieceSim(Translation3d pos, Translation3d vel) {
		this.pos = pos;
		this.vel = vel;
	}

	/**
	 * Create a gamepiece with the given position and zero initial velocity.
	 *
	 * @param pos initial position (meters)
	 */
	public GamepieceSim(Translation3d pos) {
		this(pos, new Translation3d());
	}

	/**
	 * Called by the field simulator to advance the object's state by one
	 * simulation step. Subclasses should override to implement physics.
	 */
	protected void update() {}
}
