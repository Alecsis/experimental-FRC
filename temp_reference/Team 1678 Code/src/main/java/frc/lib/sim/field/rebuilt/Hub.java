package frc.lib.sim.field.rebuilt;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import frc.lib.util.FieldLayout;

/**
 * Model of the game's hub. Responsible for detecting scored fuels and
 * handling their dispersion/exit behavior.
 *
 * This class exposes two singleton instances, {@link #mBlueHub} and {@link #mRedHub},
 * positioned according to {@link FieldLayout}.
 */
public class Hub {
	/** Blue-alliance hub instance. */
	public static final Hub mBlueHub = new Hub(FieldLayout.kBlueHub, FieldLayout.kBlueHubExit);

	/** Red-alliance hub instance (field flipped if necessary). */
	public static final Hub mRedHub = new Hub(
			FieldLayout.handleAllianceFlip(FieldLayout.kBlueHub, true),
			FieldLayout.handleAllianceFlip(FieldLayout.kBlueHubExit, true));

	private final Translation2d center;
	private final Translation3d exit;

	private Hub(Translation2d center, Translation3d exit) {
		this.center = center;
		this.exit = exit;
	}

	/**
	 * If the given fuel has scored, mark it for removal.
	 * @param fuel the fuel to test and potentially disperse
	 */
	public void disperseFuel(Fuel fuel) {
		if (didFuelScore(fuel)) {
			fuel.shouldBeRemoved = true;
		}
	}

	/**
	 * Determine whether a fuel should be considered scored by the hub.
	 *
	 * Checks distance to center and vertical bounds, including the previous
	 * sub-step's height to ensure entering from above.
	 */
	private boolean didFuelScore(Fuel fuel) {
		return fuel.pos.toTranslation2d().getDistance(center) <= FieldLayout.kHubRadius.in(Units.Meters)
				&& fuel.pos.getMeasureZ().lte(FieldLayout.kHubAltitude)
				&& fuel.pos
						.minus(fuel.vel.times(
								FieldSimConstants.kBasePeriod.in(Seconds) / FieldSimConstants.kFuelSubticks))
						.getMeasureZ()
						.lte(FieldLayout.kHubAltitude);
	}
}
