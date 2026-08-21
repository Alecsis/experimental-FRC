package frc.lib.math;

import edu.wpi.first.math.Pair;
import java.util.List;

public class LinearLineCalculator {

	private final double m_slope, m_intercept;

	public LinearLineCalculator(double slope, double intercept) {
		m_slope = slope;
		m_intercept = intercept;
	}

	public double getSlope() {
		return m_slope;
	}

	public double getIntercept() {
		return m_intercept;
	}

	public double calculate(double input) {
		if (Double.isNaN(input)) return Double.NaN;
		return m_slope * input + m_intercept;
	}

	public static LinearLineCalculator bestFit(List<Pair<Double, Double>> data) {
		int n = data.size();
		double sumX = 0.0;
		double sumY = 0.0;
		double sumXY = 0.0;
		double sumX2 = 0.0;

		for (int i = 0; i < data.size(); i++) {
			Pair<Double, Double> point = data.get(i);
			double x = point.getFirst();
			double y = point.getSecond();

			sumX += x;
			sumY += y;
			sumXY += x * y;
			sumX2 += x * x;
		}
		double meanX = sumX / n;
		double meanY = sumY / n;

		double slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
		double intercept = meanY - slope * meanX;

		return new LinearLineCalculator(slope, intercept);
	}
}
