package frc.lib.mechviz.mechanism.handler;

import edu.wpi.first.units.measure.Angle;
import frc.lib.mechviz.mechanism.mech.PoweredZAxisIntakeBarMechanism3d;
import frc.lib.mechviz.mechanism.mech.UnpoweredZAxisIntakeBarMechanism3d;

public class PoweringUnpoweredIntakeMechanism3dHandler extends Mechanism3dHandler {

	private final PoweredZAxisIntakeBarMechanism3d poweredBar;
	private final UnpoweredZAxisIntakeBarMechanism3d unpoweredBars[];

	public PoweringUnpoweredIntakeMechanism3dHandler(
			PoweredZAxisIntakeBarMechanism3d powered, UnpoweredZAxisIntakeBarMechanism3d... unpowerBars) {
		registerMech(powered);
		registerMech(unpowerBars);
		this.poweredBar = powered;
		this.unpoweredBars = unpowerBars;
	}

	@Override
	public void updatePositions() {
		Angle poweredPosition = poweredBar.getMechAngle();
		for (UnpoweredZAxisIntakeBarMechanism3d unpowered : unpoweredBars) {
			unpowered.calculateAndSetOffsetFromBase(poweredPosition);
		}
	}
}
