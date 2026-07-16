package frc.robot.tracking;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.intakedeploy.IntakeDeployConstants;

public class IntakeHopperTrackerConstants {

	public static final double EXTENSION_GEARING =
			IntakeDeployConstants.HOPPER_MAX_EXTENSION_DISTANCE.in(Meter) / 130.0;

	public static final Distance BRIDGE_ARM_LENGTH = Meters.of(0.110920);
}
