package frc.lib.io.lights;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.util.lights.LEDClosedInterval;
import frc.lib.util.lights.RGBColor;

public class CANdleLightsIO extends LightsIO {

	private final CANdle candle;

	private RGBColor currentColor = RGBColor.NONE;
	private final SolidColor request;

	public CANdleLightsIO(int id, CANBus bus, LEDClosedInterval segment, CANdleConfiguration config) {
		super(segment);
		request = new SolidColor(segment.getStart(), segment.getEnd());
		candle = new CANdle(id, bus);
		candle.getConfigurator().apply(config);
		candle.setControl(request.withLEDStartIndex(segment.getStart())
				.withLEDEndIndex(segment.getEnd())
				.withColor(RGBColor.NONE.toRGBWCOlor()));
	}

	@Override
	public void setLED(RGBColor color, LEDClosedInterval interval) {
		candle.setControl(request.withColor(color.toRGBWCOlor())
				.withLEDStartIndex(interval.getStart())
				.withLEDEndIndex(interval.getEnd()));
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addStringProperty("RGB Color", currentColor::toString, null);
	}
}
