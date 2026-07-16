package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.io.lights.CANdleLightsIO;
import frc.lib.io.lights.LightsIO;
import frc.lib.io.lights.SimLightsIO;
import frc.lib.util.lights.LEDClosedInterval;
import frc.lib.util.lights.LEDClosedInterval.LEDClosedIntervalCollisionBehavior;
import frc.lib.util.lights.RGBColor;
import frc.lib.util.lights.state.DynamicFlashingLightsState;
import frc.lib.util.lights.state.EmptyLightsState;
import frc.lib.util.lights.state.FlashingLightsState;
import frc.lib.util.lights.state.LightsState;
import frc.lib.util.lights.state.SolidLightsState;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.subsystems.vision.Cameras;
import frc.robot.subsystems.vision.CamerasConstants;

public class LEDsConstants {

	public static final int START_INDEX = 0;
	public static final int NUM_CANDLE_LIGHTS = 8;

	public static final int NUM_BACK_STRIP = 12;

	public static final int NUM_SIDE_STRIPS = 5;

	public static final int NUM_LEDS = NUM_CANDLE_LIGHTS + NUM_BACK_STRIP + NUM_SIDE_STRIPS;

	public static final LEDClosedInterval LEDS_SEGMENT = new LEDClosedInterval(0, NUM_LEDS);

	public static final LEDClosedInterval CANDLE = new LEDClosedInterval(START_INDEX, NUM_CANDLE_LIGHTS - 1);
	public static final LEDClosedInterval BACK =
			new LEDClosedInterval(NUM_CANDLE_LIGHTS, NUM_CANDLE_LIGHTS + NUM_BACK_STRIP - 1);
	public static final LEDClosedInterval SIDES = new LEDClosedInterval(
			NUM_CANDLE_LIGHTS + NUM_BACK_STRIP, NUM_CANDLE_LIGHTS + NUM_BACK_STRIP + NUM_SIDE_STRIPS - 1);

	public static final LightsState STATE_PIGEON_DISCONNECTED = new SolidLightsState(RGBColor.RED, LEDS_SEGMENT);

	public static final LightsState STATE_VISION_POSE_STABLE = new SolidLightsState(RGBColor.LIME, SIDES);
	public static final LightsState STATE_VISION_ENABLED =
			new FlashingLightsState(SIDES, Seconds.of(0.5), RGBColor.AQUA, RGBColor.GREEN);
	public static final LightsState STATE_VISION_DISABLED =
			new FlashingLightsState(SIDES, Seconds.of(0.5), RGBColor.RED, RGBColor.PURPLE);

	public static final LightsState STATE_ALLOW_SHOOTING =
			new FlashingLightsState(LEDS_SEGMENT, Milliseconds.of(90.0), RGBColor.YELLOW, RGBColor.BLUE);

	public static final LightsState STATE_NOT_ALLOW_SHOOTING =
			new FlashingLightsState(LEDS_SEGMENT, Milliseconds.of(90.0), RGBColor.NONE, RGBColor.RED);

	public static final LightsState STATE_INTAKING =
			new FlashingLightsState(LEDS_SEGMENT, Milliseconds.of(80.0), RGBColor.ORANGE, RGBColor.YELLOW);

	public static final LightsState STATE_SIDES_EMPTY = new EmptyLightsState(SIDES);
	public static final LightsState STATE_BACK_EMPTY = new EmptyLightsState(BACK);

	public static final LightsState STATE_BATTERY_WARNING_CHANGE = new FlashingLightsState(
					LEDS_SEGMENT, Seconds.of(5.0), RGBColor.RED, RGBColor.YELLOW)
			.withCollisionBehavior(LEDClosedIntervalCollisionBehavior.INCOMING_CANCEL);
	public static final LightsState STATE_BATTERY_EMERGENCY_CHANGE = new FlashingLightsState(
					LEDS_SEGMENT, Milliseconds.of(80.0), RGBColor.RED, RGBColor.NONE)
			.withCollisionBehavior(LEDClosedIntervalCollisionBehavior.INCOMING_CANCEL);

	public static final LightsState STATE_VISION_DYNAMIC_HAS_UPDATES = new DynamicFlashingLightsState(
			LEDS_SEGMENT,
			() -> {
				long high = CamerasConstants.getConfig().agreedTranslationUpdatesThreshold;
				long current = Cameras.mInstance.getNumPoseStableUpdates();
				if (high - current <= 0) {
					return BaseUnits.TimeUnit.zero();
				} else {
					return Seconds.of(Math.min(1.0, (high - current)));
				}
			},
			() -> {
				if (Seconds.of(Timer.getFPGATimestamp())
						.minus(Cameras.mInstance.getLastUpdatedPoseTime())
						.gte(Seconds.of(0.5))) {
					return new RGBColor[] {RGBColor.RED, RGBColor.PURPLE};
				} else {
					if (Cameras.mInstance.getPoseStable()) {
						return new RGBColor[] {RGBColor.GREEN};
					} else {
						return new RGBColor[] {RGBColor.GREEN, RGBColor.BLUE};
					}
				}
			});

	public static final CANdleConfiguration getLightsIOCandleConfig() {
		CANdleConfiguration config = new CANdleConfiguration();
		config.LED.StripType = StripTypeValue.RGB;
		config.LED.BrightnessScalar = 1.0;
		return config;
	}

	public static LightsIO io = null;

	public static final LightsIO createLightsIO() {
		LightsIO io = Robot.isReal()
				? new CANdleLightsIO(Ports.CANDLE.id, Ports.CANDLE.bus, LEDS_SEGMENT, getLightsIOCandleConfig())
				: new SimLightsIO(LEDS_SEGMENT);
		return io;
	}

	public static final LightsIO getLightsIO() {
		if (io == null) {
			io = createLightsIO();
		}
		return io;
	}

	public static final LightsState DISABLED = new EmptyLightsState(LEDS_SEGMENT);
	public static final LightsState ROBOT_DISABLED = new SolidLightsState(RGBColor.AQUA, LEDS_SEGMENT);
}
