// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

/**
 * Shared TalonFX construction/telemetry wrapper for use inside {@code *IOReal.java} files only --
 * never from a subsystem class or an {@code *IOSim.java} file. See
 * {@code docs/claudex/design/robotmotor-refactor.md} for the full design rationale.
 */
public class RobotMotor extends TalonFX {
  public static class MotorConfig {
    public String name;
    public int canId;
    public TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    public SafetyTripConfig safetyTrip; // null = disabled
  }

  public static class SafetyTripConfig {
    public Current statorCurrentThreshold;
    public Time trippedFor = Units.Second.of(0);
    public Runnable onTrip = () -> {};

    public static SafetyTripConfig of(Current threshold, Time debounce) {
      SafetyTripConfig cfg = new SafetyTripConfig();
      cfg.statorCurrentThreshold = threshold;
      cfg.trippedFor = debounce;
      return cfg;
    }
  }

  private final String name;
  private final SafetyTripConfig tripConfig;
  private final Debouncer tripDebouncer;

  public RobotMotor(MotorConfig config) {
    super(config.canId); // rio bus, matches every existing *IOReal.java TalonFX(id) call -- no
                          // motor migrated by this refactor needs a non-default CAN bus
    this.name = config.name;
    getConfigurator().apply(config.talonConfig);
    this.tripConfig = config.safetyTrip;
    this.tripDebouncer = tripConfig == null
        ? null
        : new Debouncer(tripConfig.trippedFor.in(Units.Second), DebounceType.kRising);
  }

  /**
   * Call once per {@code *IOReal.java} tick. Purely observational -- never calls setControl().
   * Real current protection is already the firmware StatorCurrentLimitEnable config every
   * {@code *IOReal.java} motor sets; this is early/soft warning + telemetry only. Never
   * substitutes for subsystem-owned jam logic (e.g. {@code Intake.isJammed()}, which stays
   * exactly as tested).
   */
  public boolean pollSafetyTrip() {
    if (tripConfig == null) {
      return false;
    }
    boolean overCurrent = getStatorCurrent().getValue().abs(Amps) > tripConfig.statorCurrentThreshold.in(Amps);
    boolean tripped = tripDebouncer.calculate(overCurrent);
    if (tripped) {
      Logger.recordOutput(name + "/SafetyTripped", true);
      DriverStation.reportWarning(name + " tripped its stator current safety threshold", false);
      tripConfig.onTrip.run();
    }
    return tripped;
  }
}
