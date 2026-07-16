package frc.lib.util;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

public class ControlRequestUtil {
	public static int getSlot(ControlRequest request) {
		Class<?> c = request.getClass();
		try {
			if (c.equals(MotionMagicVelocityVoltage.class)) {
				return ((MotionMagicVelocityVoltage) request).Slot;
			} else if (c.equals(MotionMagicExpoVoltage.class)) {
				return ((MotionMagicExpoVoltage) request).Slot;
			} else if (c.equals(PositionVoltage.class)) {
				return ((PositionVoltage) request).Slot;
			}
		} catch (Exception e) {
		}

		// if none or error return -1
		return -1;
	}
}
