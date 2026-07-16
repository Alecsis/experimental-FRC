package frc.lib.util.tracker;

import edu.wpi.first.math.filter.LinearFilter;

public class LinearFilterDerivativeTracker extends DerivativeTracker {

	private final LinearFilter m_filter;

	public LinearFilterDerivativeTracker(LinearFilter filter, double initial) {
		super(initial);
		m_filter = filter;
	}

	@Override
	public void update(double measure, double dt) {
		measure = m_filter.calculate(measure);
		super.update(measure, dt);
	}
}
