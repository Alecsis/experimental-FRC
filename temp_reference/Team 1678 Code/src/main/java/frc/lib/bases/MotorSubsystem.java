package frc.lib.bases;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.logging.LoggedTracer;
import frc.lib.util.tunablenumbers.TunableNumber;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

/**
 * Base subsystem for any subsystem that uses motors.
 */
public class MotorSubsystem<IO extends MotorIO> extends SubsystemBase {
	protected final IO io;
	protected final String name;
	protected boolean tuningMode = false;
	// Tuneable Number Usage
	protected TunableNumber kP0;
	protected TunableNumber kI0;
	protected TunableNumber kD0;
	protected TunableNumber kV0;
	protected TunableNumber kS0;
	protected TunableNumber kG0;

	protected TunableNumber kP1;
	protected TunableNumber kI1;
	protected TunableNumber kD1;
	protected TunableNumber kV1;
	protected TunableNumber kS1;
	protected TunableNumber kG1;

	protected TunableNumber kP2;
	protected TunableNumber kI2;
	protected TunableNumber kD2;
	protected TunableNumber kV2;
	protected TunableNumber kS2;
	protected TunableNumber kG2;
	/**
	 * Creates a MotorSubsystem with a MotorIO and name for telemetry.
	 * @param io MotorIO for the subsystem.
	 * @param name Name for telemetry.
	 * @param tuningMode Boolean determining whether the user wants to enable tuning on the fly
	 */
	public MotorSubsystem(IO io, String name, boolean tuningMode) {
		super(name);
		this.io = io;
		this.name = name;
		this.tuningMode = tuningMode; // This should never be on for subsystems without PID
		tuning(name);
	}

	public MotorSubsystem(IO io, String name) {
		this(io, name, false);
	}

	public void tuning(String name) {
		if (DriverStation.isDisabled() && tuningMode == true) {

			kP0 = new TunableNumber(name + "kP0", io.getMotorIOConfig().Slot0.kP);
			kI0 = new TunableNumber(name + "kI0", io.getMotorIOConfig().Slot0.kI);
			kD0 = new TunableNumber(name + "kD0", io.getMotorIOConfig().Slot0.kD);
			kV0 = new TunableNumber(name + "kV0", io.getMotorIOConfig().Slot0.kV);
			kS0 = new TunableNumber(name + "kS0", io.getMotorIOConfig().Slot0.kS);
			kG0 = new TunableNumber(name + "kG0", io.getMotorIOConfig().Slot0.kG);

			kP1 = new TunableNumber(name + "kP1", io.getMotorIOConfig().Slot1.kP);
			kI1 = new TunableNumber(name + "kI1", io.getMotorIOConfig().Slot1.kI);
			kD1 = new TunableNumber(name + "kD1", io.getMotorIOConfig().Slot1.kD);
			kV1 = new TunableNumber(name + "kV1", io.getMotorIOConfig().Slot1.kV);
			kS1 = new TunableNumber(name + "kS1", io.getMotorIOConfig().Slot1.kS);
			kG1 = new TunableNumber(name + "kG1", io.getMotorIOConfig().Slot1.kG);

			kP2 = new TunableNumber(name + "kP2", io.getMotorIOConfig().Slot2.kP);
			kI2 = new TunableNumber(name + "kI2", io.getMotorIOConfig().Slot2.kI);
			kD2 = new TunableNumber(name + "kD2", io.getMotorIOConfig().Slot2.kD);
			kV2 = new TunableNumber(name + "kV2", io.getMotorIOConfig().Slot2.kV);
			kS2 = new TunableNumber(name + "kS2", io.getMotorIOConfig().Slot2.kS);
			kG2 = new TunableNumber(name + "kG2", io.getMotorIOConfig().Slot2.kG);
		} else {
			kP0 = null;
			kI0 = null;
			kD0 = null;
			kV0 = null;
			kS0 = null;
			kG0 = null;

			kP1 = null;
			kI1 = null;
			kD1 = null;
			kV1 = null;
			kS1 = null;
			kG1 = null;

			kP2 = null;
			kI2 = null;
			kD2 = null;
			kV2 = null;
			kS2 = null;
			kG2 = null;
		}
	}

	// Utilizes the Tuneable Numbers
	public void useTunableNumbers() {
		UnaryOperator<TalonFXConfiguration> configChanger = (TalonFXConfiguration config) -> {
			config.Slot0.kP = kP0.getAsDouble();
			config.Slot0.kI = kI0.getAsDouble();
			config.Slot0.kD = kD0.getAsDouble();
			config.Slot0.kV = kV0.getAsDouble();
			config.Slot0.kS = kS0.getAsDouble();
			config.Slot0.kG = kG0.getAsDouble();

			config.Slot1.kP = kP1.getAsDouble();
			config.Slot1.kI = kI1.getAsDouble();
			config.Slot1.kD = kD1.getAsDouble();
			config.Slot1.kV = kV1.getAsDouble();
			config.Slot1.kS = kS1.getAsDouble();
			config.Slot1.kG = kG1.getAsDouble();

			config.Slot2.kP = kP2.getAsDouble();
			config.Slot2.kI = kI2.getAsDouble();
			config.Slot2.kD = kD2.getAsDouble();
			config.Slot2.kV = kV2.getAsDouble();
			config.Slot2.kS = kS2.getAsDouble();
			config.Slot2.kG = kG2.getAsDouble();

			return config;
		};

		io.changeMainConfig(configChanger);
		io.changeFollowerConfig(configChanger);
	}

	@Override
	public void periodic() {
		io.updateInputs();
		outputTelemetry();
		if (DriverStation.isDisabled() && tuningMode == true) {
			if (kP0.hasChanged()
					|| kI0.hasChanged()
					|| kD0.hasChanged()
					|| kV0.hasChanged()
					|| kS0.hasChanged()
					|| kG0.hasChanged()
					|| kP1.hasChanged()
					|| kI1.hasChanged()
					|| kD1.hasChanged()
					|| kV1.hasChanged()
					|| kS1.hasChanged()
					|| kG1.hasChanged()
					|| kP2.hasChanged()
					|| kI2.hasChanged()
					|| kD2.hasChanged()
					|| kV2.hasChanged()
					|| kS2.hasChanged()
					|| kG2.hasChanged()) {
				useTunableNumbers();
			}
		}
	}

	/**
	 * Outputs subsystem readings and to SmartDashboard.
	 */
	public void outputTelemetry() {
		LoggedTracer.record(name);
	}

	/**
	 * Initializes this Sendable object. Override this should you need to output more information to SmartDashboard.
	 *
	 * @param builder Sendable builder.
	 */
	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		io.initSendable(builder);
	}

	/**
	 * Gets the last read position of the subsystem's main motor.
	 *
	 * @return Position of the subsystem.
	 */
	public Angle getPosition() {
		return io.getPosition();
	}

	/**
	 * Gets the last read velocity of the subsystem's main motor.
	 *
	 * @return Velocity of the subsystem.
	 */
	public AngularVelocity getVelocity() {
		return io.getVelocity();
	}
	/**
	 * Gets the last read stator current of the subsystem's main motor.
	 *
	 * @return Stator current of the subsystem.
	 */
	public Current getStatorCurrent() {
		return io.getStatorCurrent();
	}

	/**
	 * Gets the last read supply current of the subsystem's main motor.
	 *
	 * @return Supply current of the subsystem.
	 */
	public Current getSupplyCurrent() {
		return io.getSupplyCurrent();
	}

	/**
	 * Gets the last read output voltage of the subsystem's main motor.
	 *
	 * @return Output voltage of the subsystem.
	 */
	public Voltage getMotorVoltage() {
		return io.getMotorVoltage();
	}

	/**
	 * Gets the last applied setpoint to the MotorIO.
	 *
	 * @return Last applied Setpoint.
	 */
	public Setpoint getSetpoint() {
		return io.getSetpoint();
	}

	/**
	 * Applies a Setpoint to the MotorIO.
	 *
	 * @param setpoint Setpoint to apply.
	 */
	public void applySetpoint(Setpoint setpoint) {
		io.applySetpoint(setpoint);
	}

	/**
	 * Creates a one time, instantaneus command for the subsystem to go to a given Setpoint.
	 *
	 * @param setpoint Setpoint to go to.
	 * @return One time Command for the subsystem.
	 */
	public Command setpointCommand(Setpoint setpoint) {
		return runOnce(() -> applySetpoint(setpoint));
	}

	/**
	 * Creates a continous command for the subsystem to repeatedly go to a supplied setpoint.
	 *
	 * @param setpoint Supplier of setpoint to go to.
	 * @return Continuous Command for the subsystem.
	 */
	public Command followSetpointCommand(Supplier<Setpoint> supplier) {
		return run(() -> applySetpoint(supplier.get()));
	}

	/**
	 * Disabled this Subsystem's MotorIO. Setpoints can still be set when disabled but will not be applied until re-enabled.
	 */
	public void disable() {
		io.disable();
	}

	/**
	 * Enables this Subsystem's MotorIO. Immediatly applies the last set setpoint including setpoints set when disabled. MotorIO is enabled by default.
	 */
	public void enable() {
		io.enable();
	}

	/**
	 * Creates a command to disable this Subsystem's MotorIO.
	 *
	 * @return An instantaneus Command not requiring this Subsystem.
	 */
	public Command disableCommand() {
		return Commands.runOnce(() -> io.disable());
	}

	/**
	 * Creates a command to enable this Subsystem's MotorIO.
	 *
	 * @return An instantaneus Command not requiring this Subsystem.
	 */
	public Command enableCommand() {
		return Commands.runOnce(() -> io.enable());
	}

	public IO getIO() {
		return io;
	}
}
