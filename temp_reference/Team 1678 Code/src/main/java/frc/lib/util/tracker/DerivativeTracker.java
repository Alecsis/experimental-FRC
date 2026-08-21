package frc.lib.util.tracker;

public class DerivativeTracker {

	protected double m_trackedMeasurement;
	protected double m_trackedDerivative;

	public DerivativeTracker(double initial) {
		m_trackedMeasurement = initial;
		m_trackedDerivative = 0.0;
	}

	public void update(double measure, double dt) {
		m_trackedDerivative = (measure - m_trackedMeasurement) / dt;
		m_trackedMeasurement = measure;
	}

	public double getTrackedDerivative() {
		return m_trackedDerivative;
	}
}
