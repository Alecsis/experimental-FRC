package frc.robot.shooting.regressions;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class DistanceToAirTimeRegressions {

	public static final InterpolatingDoubleTreeMap distanceToTimeHub = new InterpolatingDoubleTreeMap();

	// add example data to the tree (distance in meters, time in seconds)
	static {

		// 12.067
		// 13.113
		distanceToTimeHub.put(1.967387, 1.046);

		// 2.033
		// 3.133
		distanceToTimeHub.put(2.207408, 1.08);

		// 29.7
		// 30.800
		distanceToTimeHub.put(2.57, 1.1);

		// 30.933
		// 32.1
		distanceToTimeHub.put(2.663911, 1.107);

		// 14.900
		// 16.1
		distanceToTimeHub.put(2.75, 1.2);

		// 41.367
		// 42.600
		distanceToTimeHub.put(2.80631, 1.233);

		// 27.33
		// 28.567
		distanceToTimeHub.put(3.0, 1.237);

		// 17.333
		// 18.600
		distanceToTimeHub.put(3.5696633, 1.267);

		// distanceToTimeHub.put(4.18, 1.3);

		// 6.193
		// 7.659
		// distanceToTimeHub.put(4.5, 1.466);
	}

	public static Time getHubShotTime(Distance distanceFromHub) {
		return Seconds.of(distanceToTimeHub.get(distanceFromHub.in(Meters)));
	}

	// distance to time estimation map
	public static final InterpolatingDoubleTreeMap distanceToTimeAllianceFerry = new InterpolatingDoubleTreeMap();

	// add example data to the tree (distance in meters, time in seconds)
	static {
		// TODO add actual data
		distanceToTimeAllianceFerry.put(1.0, 0.78);
		distanceToTimeAllianceFerry.put(2.0, 0.97);
		distanceToTimeAllianceFerry.put(3.0, 1.12);
		distanceToTimeAllianceFerry.put(6.0, 4.12);
	}

	// distance to time estimation map
	public static final InterpolatingDoubleTreeMap distanceToTimeNeutralFerry = new InterpolatingDoubleTreeMap();

	// add example data to the tree (distance in meters, time in seconds)
	static {
		// TODO add actual data
		distanceToTimeNeutralFerry.put(1.0, 0.78);
		distanceToTimeNeutralFerry.put(2.0, 0.97);
		distanceToTimeNeutralFerry.put(3.0, 1.12);
		distanceToTimeNeutralFerry.put(3.0, 4.12);
	}
}
