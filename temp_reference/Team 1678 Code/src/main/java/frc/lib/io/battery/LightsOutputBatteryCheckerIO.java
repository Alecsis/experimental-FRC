package frc.lib.io.battery;

import frc.lib.io.lights.LightsIO;
import frc.lib.util.BatteryCheckerInputs;
import frc.lib.util.lights.state.LightsState;

public class LightsOutputBatteryCheckerIO extends BatteryCheckerIO {

	public static class LightsOutputBatteryCheckerIOConfig {
		public LightsIO io = null;
		public LightsState warningStates[] = null;
		public LightsState emergencyStates[] = null;
	}

	private final LightsOutputBatteryCheckerIOConfig lightsIOConfig;

	/**
	 *
	 * @param config config to apply
	 */
	public LightsOutputBatteryCheckerIO(
			BatteryCheckerConfig config, LightsOutputBatteryCheckerIOConfig lightsIOConfig) {
		super(config);
		this.lightsIOConfig = lightsIOConfig;
	}

	private boolean verifyOptionChosen(LightsState buffer[]) {
		return buffer != null && lightsIOConfig.io != null;
	}

	@Override
	public void onWarningVoltage(BatteryCheckerInputs inputs) {
		if (verifyOptionChosen(lightsIOConfig.warningStates)) {
			for (LightsState state : lightsIOConfig.warningStates) {
				state.apply(lightsIOConfig.io);
			}
		}
	}

	@Override
	public void onEmergencyVoltage(BatteryCheckerInputs inputs) {
		if (verifyOptionChosen(lightsIOConfig.emergencyStates)) {
			for (LightsState state : lightsIOConfig.emergencyStates) {
				state.apply(lightsIOConfig.io);
			}
		}
	}
}
