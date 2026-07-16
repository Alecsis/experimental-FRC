package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.bases.CameraSubsystem.Side;
import frc.lib.bases.CameraSubsystem.SimulatedCameraIOConfig;
import frc.lib.bases.CameraSubsystem.VisionConfig;
import frc.lib.io.vision.CameraIO;
import frc.lib.io.vision.limelight.AprilTagLimelightCameraIO;
import frc.lib.io.vision.photon.SublimeAprilTagPhotonCameraIO;
import frc.lib.io.vision.sim.EmptySimulatedCameraIO;
import frc.lib.io.vision.sim.photon.AprilTagSimulatedPhotonCameraIO;
import frc.lib.util.FieldLayout;
import frc.lib.util.vision.CameraPipeline;
import frc.lib.util.vision.CameraPipelineType;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;

public class CamerasConstants {

	public static class PipelineConstants {

		public static final CameraPipeline APRIL_TAG_HUB_FIELD =
				new CameraPipeline(CameraPipelineType.APRIL_TAG, "April Tag Full Field", 0);
		public static final CameraPipeline APRIL_TAG_HUB_BLUE =
				new CameraPipeline(CameraPipelineType.APRIL_TAG, "April Tag Blue Hub", 1);
		public static final CameraPipeline APRIL_TAG_HUB_RED =
				new CameraPipeline(CameraPipelineType.APRIL_TAG, "April Tag Red Hub", 2);
	}

	public static final Vector<N3> DEFAULT_STD_DEVIATION = VecBuilder.fill(0.3, 0.3, 99999999999.999999);

	public static final Vector<N3> ALIGN_STD_DEVATION = VecBuilder.fill(0.1, 0.1, 99999999999.999999);

	public static final PoseStrategy DEFAULT_APRIL_TAG_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

	public static final AprilTagFieldLayout LAYOUT = FieldLayout.kAprilTagMap;

	public static final class BackConstants {

		public static final PoseStrategy APRIL_TAG_STRATEGY = DEFAULT_APRIL_TAG_STRATEGY;

		public static final Pose3d OFFSET_FROM_CENTER = new Pose3d(
				Inches.of(-12.926376),
				Inches.of(0.008),
				Inches.of(RobotConstants.isEpsilon ? 14.375 : 14.185537),
				new Rotation3d(Degrees.of(0.0), Degrees.of(20.0), Degrees.of(180.0)));

		public static CameraIOConfig getConfig() {
			CameraIOConfig config = new CameraIOConfig();
			config.aprilTagVisionStdDevs = DEFAULT_STD_DEVIATION;
			config.robotToCameraOffset = OFFSET_FROM_CENTER;
			config.name = "limelight";
			return config;
		}

		public static final CameraIO getIO() {
			if (Robot.isReal()) {
				return new AprilTagLimelightCameraIO(getConfig());
			} else {
				if (RobotConstants.simulateVision) {
					return new AprilTagSimulatedPhotonCameraIO(
							getVisionSystemSim(), getConfig(), getSimConfig(), APRIL_TAG_STRATEGY);
				}
				return new EmptySimulatedCameraIO(getConfig());
			}
		}

		public static SimulatedCameraIOConfig getSimConfig() {
			SimulatedCameraIOConfig config = new SimulatedCameraIOConfig();
			config.calibErrorPx = 0.35;
			config.calibErrorPy = 0.25;
			config.resolutionHeightPixels = 600; // pixels
			config.resolutionWidthPixels = 800; // pixels
			config.kErrThreshold = Units.Degrees.of(25.0);
			config.maxUpdateTagDistance = Units.Meters.of(1.8);
			config.kFieldOfView = Units.Degrees.of(82.0);
			config.side = Side.FRONT;

			return config;
		}
	}

	public static final class RightConstants {

		public static final PoseStrategy APRIL_TAG_STRATEGY = DEFAULT_APRIL_TAG_STRATEGY;

		// right from intake side
		public static final Pose3d OFFSET_FROM_CENTER = new Pose3d(
				Inches.of(-2.518),
				Inches.of(-13.475),
				Inches.of(14.844).minus(Inches.of(1.6875)),
				new Rotation3d(Degrees.of(180.0), Degrees.of(0.0), Degrees.of(-90.0)));

		public static CameraIOConfig getConfig() {
			CameraIOConfig config = new CameraIOConfig();
			config.aprilTagVisionStdDevs = DEFAULT_STD_DEVIATION;
			config.robotToCameraOffset = OFFSET_FROM_CENTER;
			config.name = "right";
			return config;
		}

		public static final CameraIO getIO() {
			if (Robot.isReal()) {
				return new SublimeAprilTagPhotonCameraIO(getConfig(), APRIL_TAG_STRATEGY, LAYOUT);
			} else {
				if (RobotConstants.simulateVision) {
					return new AprilTagSimulatedPhotonCameraIO(
							getVisionSystemSim(), getConfig(), getSimConfig(), APRIL_TAG_STRATEGY);
				}
				return new EmptySimulatedCameraIO(getConfig());
			}
		}

		public static SimulatedCameraIOConfig getSimConfig() {
			SimulatedCameraIOConfig config = new SimulatedCameraIOConfig();
			config.calibErrorPx = 0.35;
			config.calibErrorPy = 0.25;
			config.resolutionHeightPixels = 600; // pixels
			config.resolutionWidthPixels = 800; // pixels
			config.kErrThreshold = Units.Degrees.of(25.0);
			config.maxUpdateTagDistance = Units.Meters.of(1.8);
			config.kFieldOfView = Units.Degrees.of(82.0);
			config.side = Side.RIGHT;

			return config;
		}
	}

	public static final class LeftConstants {

		public static final PoseStrategy APRIL_TAG_STRATEGY = DEFAULT_APRIL_TAG_STRATEGY;

		// left from intake side
		public static final Pose3d OFFSET_FROM_CENTER = new Pose3d(
				Inches.of(-2.518),
				Inches.of(13.475),
				Inches.of(14.844).minus(Inches.of(1.6875)),
				new Rotation3d(Degrees.of(180.0), Degrees.of(0.0), Degrees.of(90.0)));

		public static CameraIOConfig getConfig() {
			CameraIOConfig config = new CameraIOConfig();
			config.aprilTagVisionStdDevs = DEFAULT_STD_DEVIATION;
			config.robotToCameraOffset = OFFSET_FROM_CENTER;
			config.name = "left";
			return config;
		}

		public static final CameraIO getIO() {
			if (Robot.isReal()) {
				return new SublimeAprilTagPhotonCameraIO(getConfig(), APRIL_TAG_STRATEGY, LAYOUT);
			} else {
				if (RobotConstants.simulateVision) {
					return new AprilTagSimulatedPhotonCameraIO(
							getVisionSystemSim(), getConfig(), getSimConfig(), APRIL_TAG_STRATEGY);
				}
				return new EmptySimulatedCameraIO(getConfig());
			}
		}

		public static SimulatedCameraIOConfig getSimConfig() {
			SimulatedCameraIOConfig config = new SimulatedCameraIOConfig();
			config.calibErrorPx = 0.35;
			config.calibErrorPy = 0.25;
			config.resolutionHeightPixels = 600; // pixels
			config.resolutionWidthPixels = 800; // pixels
			config.kErrThreshold = Units.Degrees.of(25.0);
			config.maxUpdateTagDistance = Units.Meters.of(1.8);
			config.kFieldOfView = Units.Degrees.of(82.0);
			config.side = Side.LEFT;

			return config;
		}
	}

	public static VisionConfig getConfig() {
		VisionConfig config = new VisionConfig();
		config.name = "Vision";
		config.agreedTranslationUpdatesThreshold = 100;
		config.agreedTranslationUpdateEpsilon = Centimeters.of(20.0);
		config.cameras = new CameraIO[] {BackConstants.getIO()};
		return config;
	}

	public static VisionSystemSim getVisionSystemSim() {
		VisionSystemSim visionSim = new VisionSystemSim("main");
		visionSim.addAprilTags(FieldLayout.kAprilTagMap);
		return visionSim;
	}
}
