package frc.lib.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.sim.PivotSim.PivotSimConstants;

/**
 * Hood-specific pivot simulation that includes strong static and kinetic friction.
 */
public class PivotSimHelper extends PivotSim {
	private static final double MIN_SPEED_EPSILON_RAD_PER_SEC = 1e-3;
	private final double staticFrictionVolts;
	private final double kineticFrictionVolts;
	private final double stictionVelocityRadPerSec;
	private final double viscousFrictionVoltsPerRadPerSec;

	public PivotSimHelper(
			PivotSimConstants constants,
			Voltage staticFriction,
			Voltage kineticFriction,
			AngularVelocity stictionVelocity,
			double viscousFrictionVoltsPerRadPerSec) {
		super(constants);
		this.staticFrictionVolts = staticFriction.in(Units.Volts);
		this.kineticFrictionVolts = kineticFriction.in(Units.Volts);
		this.stictionVelocityRadPerSec = stictionVelocity.in(Units.RadiansPerSecond);
		this.viscousFrictionVoltsPerRadPerSec = viscousFrictionVoltsPerRadPerSec;
	}

	@Override
	public void setVoltage(Voltage voltage) {
		double requestedVolts = voltage.in(Units.Volts);
		double velocityRadPerSec = sim.getVelocityRadPerSec();
		double absVelocity = Math.abs(velocityRadPerSec);
		double absRequestedVolts = Math.abs(requestedVolts);

		// Hold position when commanded torque is too small to break static friction.
		if (absVelocity < stictionVelocityRadPerSec && absRequestedVolts < staticFrictionVolts) {
			sim.setInputVoltage(0.0);
			return;
		}

		double frictionDirection = absVelocity > MIN_SPEED_EPSILON_RAD_PER_SEC
				? Math.signum(velocityRadPerSec)
				: Math.signum(requestedVolts);
		double normalizedVelocity = absVelocity / Math.max(stictionVelocityRadPerSec, MIN_SPEED_EPSILON_RAD_PER_SEC);
		double staticToKineticBlend = Math.exp(-normalizedVelocity * normalizedVelocity);
		double coulombFrictionVolts =
				kineticFrictionVolts + (staticFrictionVolts - kineticFrictionVolts) * staticToKineticBlend;
		double frictionVolts =
				frictionDirection * coulombFrictionVolts + velocityRadPerSec * viscousFrictionVoltsPerRadPerSec;
		double netVolts = requestedVolts - frictionVolts;

		sim.setInputVoltage(MathUtil.clamp(netVolts, -12.0, 12.0));
	}
}
