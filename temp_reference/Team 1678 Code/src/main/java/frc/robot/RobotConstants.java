package frc.robot;

import choreo.auto.AutoFactory;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.util.sendable.Sendable;
import frc.lib.io.battery.BatteryCheckerIO;
import frc.lib.util.FieldLayout.FieldType;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feederrollers.FeederRollers;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopperrollers.HopperRollers;
import frc.robot.subsystems.intakedeploy.IntakeDeploy;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.tunnelrollers.TunnelRollers;
import frc.robot.subsystems.vision.Cameras;

public class RobotConstants {
	public static String kSerial;
	public static boolean isEpsilon;
	public static boolean isGamma;
	public static final FieldType currentFieldType = FieldType.Andymark;
	public static boolean simulateVision = false; // set it to false by default so unit tests and such dont fail

	public static final CANBus canivore1 = new CANBus("canivore1", "./logs/example.hoot");
	public static final CANBus rio = new CANBus();

	static {
		if (Robot.isReal()) {
			kSerial = System.getenv("serialnum");
			// isGamma = kSerial.startsWith("0332052D");
			// isEpsilon = kSerial.startsWith("033205D1");
			isGamma = false;
			isEpsilon = true;
		} else {
			kSerial = "";
			isEpsilon = true;
		}
		RobotConstants.isRedAlliance = false;
	}

	public static Sendable LOGGED_SENDABLES[] = new Sendable[] {
		Drive.mInstance,
		Cameras.mInstance,
		HopperRollers.mInstance,
		IntakeRollers.mInstance,
		TunnelRollers.mInstance,
		FeederRollers.mInstance,
		IntakeDeploy.mInstance,
		Shooter.mInstance,
		Hood.mInstance,
		LEDs.mInstance,
		Climber.mInstance,
		Superstructure.mInstance
	};

	public static boolean isRedAlliance;
	public static AutoFactory mAutoFactory;

	public static final BatteryCheckerIO batteryChecker = BatteryCheckerConstants.createBatteryChecker();
}
