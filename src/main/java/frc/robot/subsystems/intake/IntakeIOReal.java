// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.utility.RobotMotor;

/** IntakeIO implementation for real TalonFX hardware. The only file allowed to touch CTRE APIs. */
public class IntakeIOReal implements IntakeIO {
  private final RobotMotor intakePivot = new RobotMotor(pivotConfig());
  private final RobotMotor roller = new RobotMotor(rollerConfig());

  private final VoltageOut pivotVoltageOut = new VoltageOut(0);
  private final MotionMagicVoltage pivotPositionOut = new MotionMagicVoltage(0);
  private final VoltageOut rollerVoltageOut = new VoltageOut(0);
  private final VelocityVoltage rollerVelocityOut = new VelocityVoltage(0);

  /**
   * Publish rate for the two pivot signals the homing hardstop detector reads.
   *
   * <p><b>Why the stock rate is not enough.</b> On the roboRIO CAN 2.0 bus a TalonFX publishes
   * StatorCurrent and SupplyCurrent at 4.0 Hz by default (Phoenix 6 26.1.3,
   * {@code CoreTalonFX.getStatorCurrent()} javadoc: "Default Rates: CAN 2.0: 4.0 Hz"). The homing
   * seek is bounded at 1.0 s, so the stock rate offers about FOUR current samples for the whole
   * seek, and the 50 Hz robot loop re-reads each of them roughly twelve times. Any
   * persistence/debounce built on that is counting repeats of one stale CAN frame, not independent
   * evidence -- which is precisely what {@code Intake.evaluateHardstopQualification()} needs.
   *
   * <p><b>Why 100 Hz and not the 1000 Hz maximum.</b> 100 Hz gives ~100 fresh current samples
   * inside the 1 s seek and at least two per 20 ms robot loop, so every loop sample the detector
   * consumes is genuinely new and at most ~10 ms old -- comfortably enough to resolve
   * {@code kHardstopQualifiedSeconds} (60 ms, three loops). It is also the rate CTRE itself ships
   * as the CAN FD default for these same signals, rather than a number invented here. Going faster
   * buys no additional decisions, because the consumer is a 50 Hz loop, while costing bus time
   * linearly.
   *
   * <p><b>Estimated CAN cost.</b> A Phoenix 6 status frame is an 8-byte extended-ID CAN frame:
   * 131 bits before stuffing, ~160 bits worst case with stuffing and the inter-frame space, i.e.
   * ~160 us on a 1 Mbit/s bus. Raising a 4 Hz signal to 100 Hz adds 96 frames/s = ~1.5% of the bus
   * per additional status frame. StatorCurrent and SupplyCurrent may share a frame (Phoenix
   * applies the fastest requested rate to a whole frame), so the current pair costs between ~1.5%
   * and ~3.0%. Velocity already defaults to 50 Hz, so raising it to 100 Hz adds 50 frames/s =
   * ~0.8%. Total worst case is therefore under 4% of one 1 Mbit/s bus for one motor -- modest, and
   * bounded to the single pivot TalonFX. No other device rate is touched, and
   * {@code optimizeBusUtilization()} is deliberately NOT called anywhere in this project, so
   * nothing else changes.
   */
  private static final double kPivotHomingSignalHz = 100.0;

  public IntakeIOReal() {
    configurePivotSignalRates();
  }

  /**
   * Raises the publish rate of exactly the three pivot signals the homing detector and its
   * telemetry consume. Velocity is included because the detector requires near-stall motion
   * simultaneously with high current: sampling the two halves of that signature at different rates
   * would let a stale velocity frame pair with a fresh current frame.
   *
   * <p>Applied once at construction, on real hardware only -- {@link IntakeIOSim} is a different
   * IntakeIO implementation and this class is never constructed off a roboRIO. A failure here is
   * reported and non-fatal: the detector still functions on the stock rates, just with far fewer
   * independent samples, so refusing to boot over it would be worse than homing conservatively.
   */
  private void configurePivotSignalRates() {
    var status = BaseStatusSignal.setUpdateFrequencyForAll(
        Hertz.of(kPivotHomingSignalHz),
        intakePivot.getStatorCurrent(),
        intakePivot.getSupplyCurrent(),
        intakePivot.getVelocity());
    if (!status.isOK()) {
      DriverStation.reportWarning(
          "Intake/Pivot homing signal rates could not be raised to " + kPivotHomingSignalHz
              + " Hz (" + status + "); the hardstop detector will run on the stock 4 Hz current"
              + " rate and see far fewer independent samples per seek.",
          false);
    }
  }

  private static RobotMotor.MotorConfig pivotConfig() {
    RobotMotor.MotorConfig cfg = new RobotMotor.MotorConfig();
    cfg.name = "Intake/Pivot";
    cfg.canId = Constants.intakePivot;
    cfg.talonConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive))
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.kIntakePivotReduction)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(70))
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RPM.of(6000))
                .withMotionMagicAcceleration(
                    RPM.of(6000).div(Constants.kIntakePivotReduction).per(Second)))
        .withSlot0(
            new Slot0Configs()
                .withKP(70.0)
                .withKD(0.0)
                .withKV(12 / RPM.of(6000).in(RotationsPerSecond)));
    cfg.safetyTrip = RobotMotor.SafetyTripConfig.of(Amps.of(100), Second.of(0.25));
    return cfg;
  }

  private static RobotMotor.MotorConfig rollerConfig() {
    RobotMotor.MotorConfig cfg = new RobotMotor.MotorConfig();
    cfg.name = "Intake/Roller";
    cfg.canId = Constants.roller;
    cfg.talonConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive))
        .withSlot0(
            new Slot0Configs()
                .withKP(.35)
                .withKV(1.44057623))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(90))
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true));
    // safetyTrip deliberately left null -- Intake.isJammed() already owns tuned jam-recovery on
    // this exact signal (kJamStatorCurrentAmps); a second independent detector risks the "two
    // writers, same tick" failure class documented in docs/claudex/architecture.md.
    return cfg;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakePivot.pollSafetyTrip();

    inputs.pivotConnected = intakePivot.isConnected();
    inputs.pivotPositionRads = intakePivot.getPosition().getValue().in(Radians);
    inputs.pivotVelocityRadsPerSec = intakePivot.getVelocity().getValue().in(RadiansPerSecond);
    inputs.pivotAppliedVolts = intakePivot.getMotorVoltage().getValue().in(Volts);
    inputs.pivotStatorCurrentAmps = intakePivot.getStatorCurrent().getValue().in(Amps);
    inputs.pivotSupplyCurrentAmps = intakePivot.getSupplyCurrent().getValue().in(Amps);
    inputs.pivotTempCelsius = intakePivot.getDeviceTemp().getValueAsDouble();

    inputs.rollerConnected = roller.isConnected();
    inputs.rollerPositionRads = roller.getPosition().getValue().in(Radians);
    inputs.rollerVelocityRadsPerSec = roller.getVelocity().getValue().in(RadiansPerSecond);
    inputs.rollerAppliedVolts = roller.getMotorVoltage().getValue().in(Volts);
    inputs.rollerStatorCurrentAmps = roller.getStatorCurrent().getValue().in(Amps);
    inputs.rollerSupplyCurrentAmps = roller.getSupplyCurrent().getValue().in(Amps);
    inputs.rollerTempCelsius = roller.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setPivotVoltage(double volts) {
    intakePivot.setControl(pivotVoltageOut.withOutput(Volts.of(volts)));
  }

  @Override
  public void setPivotPosition(double positionRads) {
    intakePivot.setControl(pivotPositionOut.withPosition(Radians.of(positionRads)));
  }

  @Override
  public void setPivotEncoderPosition(double positionRads) {
    intakePivot.setPosition(Radians.of(positionRads));
  }

  @Override
  public void setRollerVoltage(double volts) {
    roller.setControl(rollerVoltageOut.withOutput(Volts.of(volts)));
  }

  @Override
  public void setRollerVelocity(double velocityRadPerSec) {
    roller.setControl(rollerVelocityOut.withVelocity(RadiansPerSecond.of(velocityRadPerSec)));
  }
}
