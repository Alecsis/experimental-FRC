package frc.robot.shooting.regressions;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class ShootingRegressions {

	private final InterpolatingDoubleTreeMap distanceToHoodAngleMap,
			distanceToShooterVelocityMap,
			distanceToTunnelVelocityMap;

	public ShootingRegressions(
			InterpolatingDoubleTreeMap distanceToHoodAngleMap,
			InterpolatingDoubleTreeMap distanceToShooterVelocityMap,
			InterpolatingDoubleTreeMap distanceToTunnelVelocityMap) {
		this.distanceToHoodAngleMap = distanceToHoodAngleMap;
		this.distanceToShooterVelocityMap = distanceToShooterVelocityMap;
		this.distanceToTunnelVelocityMap = distanceToTunnelVelocityMap;
	}

	public Angle getHoodAngle(Distance input) {
		return Degrees.of(distanceToHoodAngleMap.get(input.in(Meters)));
	}

	public AngularVelocity getShooterVelocity(Distance input) {
		return RPM.of(distanceToShooterVelocityMap.get(input.in(Meters)));
	}

	public AngularVelocity getTunnelVelocity(Distance input) {
		return RPM.of(distanceToTunnelVelocityMap.get(input.in(Meters)));
	}
}
