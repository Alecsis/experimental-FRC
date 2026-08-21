package frc.lib.sim.field.rebuilt;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.sim.field.GamepieceSim;
import frc.lib.util.FieldLayout;

/**
 * Simulation of a fuel gamepiece. Handles discrete integration (with
 * sub-steps), ground contact behavior (friction and small-velocity clamping),
 * hub scoring checks, and lifecycle removal conditions.
 */
public class Fuel extends GamepieceSim {
	/** Flag indicating this fuel should be removed from the simulator. */
	public boolean shouldBeRemoved = false;

	public Fuel(Translation3d pos, Translation3d vel) {
		super(pos, vel);
	}

	public Fuel(Translation3d pos) {
		super(pos);
	}

	/**
	 * Lifecycle: sets {@link #shouldBeRemoved} if the fuel falls below the
	 * playing surface or leaves the field bounds.
	 */
	@Override
	public void update() {
		double dt = FieldSimConstants.kBasePeriod.in(Seconds) / (double) FieldSimConstants.kFuelSubticks;

		// Integrate position by one sub-step
		pos = pos.plus(vel.times(dt));

		// if above ground, apply gravity
		if (pos.getZ() > FieldSimConstants.kFuelRadius.in(Meters)) {
			vel = vel.plus(FieldSimConstants.kGravityVector.times(dt));
		} else {
			// on/near ground: zero small vertical velocity and apply horizontal friction
			if (Math.abs(vel.getZ()) < FieldSimConstants.kSmallVelocityThreshold.in(MetersPerSecond)) {
				vel = new Translation3d(vel.getX(), vel.getY(), 0);
			}

			// apply friction to horizontal components
			double frictionFactor = 1.0 - FieldSimConstants.kFrictionCoefficient * dt;
			vel = new Translation3d(vel.getX() * frictionFactor, vel.getY() * frictionFactor, vel.getZ());

			// clamp position to resting radius
			if (pos.getZ() < FieldSimConstants.kFuelRadius.in(Meters)) {
				pos = new Translation3d(pos.getX(), pos.getY(), FieldSimConstants.kFuelRadius.in(Meters));
			}
		}

		handleHubDispersion();

		// Remove if below absolute floor
		if (pos.getZ() < 0.0) {
			shouldBeRemoved = true;
			return;
		}

		// Remove if outside the playing field bounds
		Pose2d checkPose = new Pose2d(pos.toTranslation2d(), Rotation2d.kZero);
		if (FieldLayout.outsideField(checkPose)) {
			shouldBeRemoved = true;
		}
	}

	private void handleHubDispersion() {
		handleHubDispersion(Hub.mBlueHub);
		handleHubDispersion(Hub.mRedHub);
	}

	private void handleHubDispersion(Hub hub) {
		hub.disperseFuel(this);
	}
}
