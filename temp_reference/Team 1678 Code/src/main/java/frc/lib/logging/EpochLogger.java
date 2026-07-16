package frc.lib.logging;

import static edu.wpi.first.units.Units.Microseconds;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Time;
import java.util.HashMap;
import java.util.Map;

public class EpochLogger {
	private final Map<String, Long> epochMap = new HashMap<>();

	public void time(String key, long timestamp) {
		epochMap.put(key, timestamp);
	}

	public Time timeEnd(String key, long timestamp) {
		var previous = epochMap.get(key);
		if (previous != null) {
			epochMap.remove(key);
			// Get the difference between previous and current timestamps in microseconds
			// Divide by 1e6 to convert to seconds
			return Microseconds.of(timestamp - previous);
		}
		return BaseUnits.TimeUnit.zero();
	}
}
