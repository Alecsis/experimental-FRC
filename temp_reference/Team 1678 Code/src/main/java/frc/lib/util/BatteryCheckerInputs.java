package frc.lib.util;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class BatteryCheckerInputs implements Sendable {

	public Voltage currentVoltage;

	public Voltage warningThreshold;
	public Voltage emergencyThreshold;

	public Time atWarningThresholdTime;
	public Time atEmergencyThresholdTime;

	public final VoltageUnit voltageUnit;
	public final TimeUnit timeUnit;

	public BatteryCheckerInputs(
			Voltage currentVoltage,
			Voltage warningThreshold,
			Voltage emergencyThreshold,
			Time atWarningThresholdTime,
			Time atEmergencyThresholdTime,
			VoltageUnit voltageUnit,
			TimeUnit timeUnit) {
		update(currentVoltage, warningThreshold, emergencyThreshold, atWarningThresholdTime, atEmergencyThresholdTime);
		this.voltageUnit = voltageUnit;
		this.timeUnit = timeUnit;
	}

	public BatteryCheckerInputs(VoltageUnit voltageUnit, TimeUnit timeUnit) {
		this(
				BaseUnits.VoltageUnit.zero(),
				BaseUnits.VoltageUnit.zero(),
				BaseUnits.VoltageUnit.zero(),
				BaseUnits.TimeUnit.zero(),
				BaseUnits.TimeUnit.zero(),
				voltageUnit,
				timeUnit);
	}

	public BatteryCheckerInputs update(
			Voltage currentVoltage,
			Voltage warningThreshold,
			Voltage emergencyThreshold,
			Time atWarningThresholdTime,
			Time atEmergencyThresholdTime) {
		this.currentVoltage = currentVoltage;
		this.warningThreshold = warningThreshold;
		this.emergencyThreshold = emergencyThreshold;
		this.atWarningThresholdTime = atWarningThresholdTime;
		this.atEmergencyThresholdTime = atEmergencyThresholdTime;
		return this;
	}

	public BatteryCheckerInputs reset() {
		return update(
				BaseUnits.VoltageUnit.zero(),
				BaseUnits.VoltageUnit.zero(),
				BaseUnits.VoltageUnit.zero(),
				BaseUnits.TimeUnit.zero(),
				BaseUnits.TimeUnit.zero());
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Supplied Voltage " + voltageUnit.name(), () -> currentVoltage.in(voltageUnit), null);
		builder.addDoubleProperty(
				"Elapsed/Warning " + timeUnit.name(), () -> atWarningThresholdTime.in(timeUnit), null);
		builder.addDoubleProperty(
				"Elapsed/Emergency " + timeUnit.name(), () -> atEmergencyThresholdTime.in(timeUnit), null);
	}
}
