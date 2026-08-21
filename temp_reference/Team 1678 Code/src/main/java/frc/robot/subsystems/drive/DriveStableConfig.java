package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class DriveStableConfig {

	private final LinearVelocity maxLinearVelocityThreshold;
	private final AngularVelocity maxAngularVelocityThreshold;

	private final Debouncer thresholdCheckDebouncer;
	private final Time thresholdCheckDebounce;

	public DriveStableConfig(
			LinearVelocity maxLinearVelocityThreshold,
			AngularVelocity maxAngularVelocityThreshold,
			Time thresholdCheckDebounce) {
		this.maxLinearVelocityThreshold = maxLinearVelocityThreshold;
		this.maxAngularVelocityThreshold = maxAngularVelocityThreshold;
		this.thresholdCheckDebounce = thresholdCheckDebounce;
		thresholdCheckDebouncer = new Debouncer(thresholdCheckDebounce.in(Seconds));
	}

	public boolean isStable(ChassisSpeeds speeds) {
		return thresholdCheckDebouncer.calculate(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
						<= maxLinearVelocityThreshold.in(MetersPerSecond)
				&& speeds.omegaRadiansPerSecond <= maxAngularVelocityThreshold.in(RadiansPerSecond));
	}

	public boolean isStable() {
		return isStable(Drive.mInstance.getState().Speeds);
	}
}
