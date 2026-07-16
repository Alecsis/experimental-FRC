package frc.lib.io.trigger;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.logging.LogUtil;

public class CANRangeSensorIO extends SensorIO {

	protected final CANrange range;
	protected final CANRangeTriggerIOConfig config;

	private final CANRangeTriggerIOInputs inputs = new CANRangeTriggerIOInputs();

	public CANRangeSensorIO(CANRangeTriggerIOConfig config, Time debounceTime) {
		super(debounceTime, config.name);
		this.config = config;
		range = new CANrange(config.portID(), config.bus());
		StatusCode endResultCode = StatusCode.TxTimeout;
		for (int i = 0; i < config.numMaxAppliedConfig; i++) {
			if ((endResultCode = range.getConfigurator().apply(config.deviceConfig)).isOK()) {
				break;
			}
		}
		if (endResultCode.isError()) {
			LogUtil.timestampError("UNABLE TO CREATE CANRANGE " + config.name);
			throw new RuntimeException("Failed to create object");
		}
	}

	@Override
	public void update() {
		super.update();
		inputs.signalStrength = range.getSignalStrength().getValueAsDouble();
		inputs.distance = range.getDistance().getValue();
	}

	@Override
	public boolean get() {
		return inputs.signalStrength >= (config.minSignalStrengthThreshold)
				&& inputs.distance.lte(config.maxDistanceThreshold);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addBooleanProperty(name + "/HeartBeat", range::isConnected, null);
		builder.addDoubleProperty(name + "/Distance", () -> inputs.distance.in(Meters), null);
		builder.addDoubleProperty(name + "/Signal Strength", () -> inputs.signalStrength, null);
		// builder.addBooleanProperty(name + "Within CAN Range", () -> Superstructure.withinCANRange(), null);
	}

	public record CANRangeTriggerIOConfig(
			int portID,
			CANBus bus,
			int numMaxAppliedConfig,
			CANrangeConfiguration deviceConfig,
			double minSignalStrengthThreshold,
			Distance maxDistanceThreshold,
			String name) {}

	private class CANRangeTriggerIOInputs {
		private double signalStrength = 0.0;
		private Distance distance = Units.Meters.of(0.0);
	}
}
