package frc.robot.shooting.driveplanner;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.DriveConstants;

public class DynamicShootingPlannerConstants {

	public static class HubDynamicShootingConstants {

		public static final ProfiledPIDController LOCK_CONTROLLER = new ProfiledPIDController(
				RobotConstants.isEpsilon ? 5.5 : 5.0,
				RobotConstants.isEpsilon ? 0.0 : 0.0,
				RobotConstants.isEpsilon ? 0.0 : 0.0,
				new TrapezoidProfile.Constraints(
						DriveConstants.kMaxAngularRate.in(RadiansPerSecond),
						DriveConstants.kMaxAngularAcceleration.in(RadiansPerSecondPerSecond)),
				Robot.kDefaultPeriod);
		public static final double SPEED_CONSTRAIN_COEFFECIENT = Robot.isReal() ? 1.0 / 5.0 : 1.0 / 2.0;

		public static final Angle I_ERROR_RANGE = Degrees.of(RobotConstants.isEpsilon ? 5.0 : 5.0);

		static {
			// LOCK_CONTROLLER = new SynchronousPIDF(4.5, 0.0, 0.1, 0.0);
			// LOCK_CONTROLLER.setContinuous();
			// LOCK_CONTROLLER.setInputRange(-Math.PI, Math.PI);
			// LOCK_CONTROLLER.setMaxAbsoluteOutput(DriveConstants.kMaxAngularRate.in(RadiansPerSecond));
			LOCK_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
			LOCK_CONTROLLER.setTolerance(Degrees.of(0.5).in(Radians));
			LOCK_CONTROLLER.setIZone(I_ERROR_RANGE.in(Radians));
		}

		static {
			SmartDashboard.putData("Hub Lock Controller", LOCK_CONTROLLER);
		}
	}

	public static class FerryNeutralDynamicShootingConstants {

		public static final ProfiledPIDController LOCK_CONTROLLER = new ProfiledPIDController(
				RobotConstants.isEpsilon ? 5.5 : 5.0,
				RobotConstants.isEpsilon ? 0.0 : 0.0,
				RobotConstants.isEpsilon ? 0.0 : 0.0,
				new TrapezoidProfile.Constraints(
						DriveConstants.kMaxAngularRate.in(RadiansPerSecond),
						DriveConstants.kMaxAngularAcceleration.in(RadiansPerSecondPerSecond)),
				Robot.kDefaultPeriod);

		static {
			// LOCK_CONTROLLER = new SynchronousPIDF(9.0, 0.0, 0.5, 0.0);
			// LOCK_CONTROLLER.setContinuous();
			// LOCK_CONTROLLER.setInputRange(-Math.PI, Math.PI);
			// LOCK_CONTROLLER.setMaxAbsoluteOutput(DriveConstants.kMaxAngularRate.in(RadiansPerSecond));
			// LOCK_CONTROLLER.setTolerance(0.01);
			LOCK_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
		}
	}

	public static class FerryAllianceDynamicShootingConstants {

		public static final ProfiledPIDController LOCK_CONTROLLER = new ProfiledPIDController(
				RobotConstants.isEpsilon ? 5.5 : 5.0,
				RobotConstants.isEpsilon ? 0.0 : 0.0,
				RobotConstants.isEpsilon ? 0.0 : 0.0,
				new TrapezoidProfile.Constraints(
						DriveConstants.kMaxAngularRate.in(RadiansPerSecond),
						DriveConstants.kMaxAngularAcceleration.in(RadiansPerSecondPerSecond)),
				Robot.kDefaultPeriod);

		static {
			// LOCK_CONTROLLER = new SynchronousPIDF(9.0, 0.0, 0.5, 0.0);
			// LOCK_CONTROLLER.setContinuous();
			// LOCK_CONTROLLER.setInputRange(-Math.PI, Math.PI);
			// LOCK_CONTROLLER.setMaxAbsoluteOutput(DriveConstants.kMaxAngularRate.in(RadiansPerSecond));
			// LOCK_CONTROLLER.setTolerance(0.01);
			LOCK_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
		}
	}
}
