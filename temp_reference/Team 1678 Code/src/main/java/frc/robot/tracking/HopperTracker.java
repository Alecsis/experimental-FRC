package frc.robot.tracking;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.robot.subsystems.intakedeploy.IntakeDeploy;
import frc.robot.subsystems.intakedeploy.IntakeDeployConstants;

public class HopperTracker implements Sendable {

	public static final HopperTracker mInstance = new HopperTracker();

	private HopperTracker() {
		SendableRegistry.setName(this, "Hopper");
	}

	public Distance getHopperExtension() {
		return IntakeDeployConstants.HOPPER_MAX_EXTENSION_DISTANCE.minus(
				Meters.of(IntakeHopperTrackerConstants.EXTENSION_GEARING
						* IntakeDeploy.mInstance.getPosition().in(Degrees)));
	}

	public Angle getFloorAngle() {
		Angle intakeAngle = IntakeDeploy.mInstance.getPosition();
		double gearing = 40.0 / 130.0;
		return intakeAngle.times(gearing);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty(
				"Hopper Extension Meters", () -> getHopperExtension().in(Meters), null);
	}
}
