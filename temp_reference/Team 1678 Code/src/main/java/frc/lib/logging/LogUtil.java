package frc.lib.logging;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.command.TimedCommand;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.stream.DoubleStream;

public class LogUtil {
	public static final HashMap<String, StructPublisher<? extends StructSerializable>> structPublisherMap =
			new HashMap<>();
	public static final HashMap<String, StructArrayPublisher<? extends StructSerializable>> structArrayPublisherMap =
			new HashMap<>();
	public static final HashMap<String, DoublePublisher> errorTableMap = new HashMap<>();

	public static final NetworkTable baseLoggingTable =
			NetworkTableInstance.getDefault().getTable("SmartDashboard");
	public static final NetworkTable errorLoggingTable = baseLoggingTable.getSubTable("Errors");
	public static final EpochLogger epochLogger = new EpochLogger();

	public static <Serializable extends StructSerializable> Optional<Struct<Serializable>> getStruct(
			Serializable serializable) {
		try {
			Field structField = serializable.getClass().getDeclaredField("struct");
			structField.setAccessible(true);
			@SuppressWarnings("unchecked")
			Struct<Serializable> struct = (Struct<Serializable>) structField.get(serializable);
			return Optional.of(struct);
		} catch (Exception e) {
			return Optional.empty();
		}
	}

	public static <T extends StructSerializable, S extends Struct<T>> void log(
			String key, T structSerializable, S struct) {
		@SuppressWarnings("unchecked")
		StructPublisher<T> stored = (StructPublisher<T>) structPublisherMap.get(key);
		if (stored != null) {
			stored.set(structSerializable);
		} else {
			StructPublisher<T> publisher =
					baseLoggingTable.getStructTopic(key, struct).publish();
			structPublisherMap.put(key, publisher);
		}
	}

	public static <Serializable extends StructSerializable> void log(String key, Serializable serializable) {
		Optional<Struct<Serializable>> readStruct = getStruct(serializable);
		readStruct.ifPresentOrElse(
				struct -> log(key, serializable, struct),
				() -> SmartDashboard.putNumber("LogUtil/Last Unable to log struct", Timer.getFPGATimestamp()));
	}

	public static <Serializable extends StructSerializable, S extends Struct<Serializable>> void log(
			String key, Serializable serializables[], S struct) {
		@SuppressWarnings("unchecked")
		StructArrayPublisher<Serializable> stored =
				(StructArrayPublisher<Serializable>) structArrayPublisherMap.get(key);
		if (stored != null) {
			stored.set(serializables);
		} else {
			StructArrayPublisher<Serializable> publisher =
					baseLoggingTable.getStructArrayTopic(key, struct).publish();
			publisher.set(serializables);
			structArrayPublisherMap.put(key, publisher);
		}
	}

	public static <Serializable extends StructSerializable> void log(String key, Serializable[] serializables) {
		if (serializables.length > 0) {
			Optional<Struct<Serializable>> readStruct = getStruct(serializables[0]);
			readStruct.ifPresentOrElse(
					struct -> log(key, serializables, struct),
					() -> SmartDashboard.putNumber(
							"LogUtil/Last Unable to log struct array", Timer.getFPGATimestamp()));
		}
	}

	public static void timestampError(String _key) {
		DoublePublisher publisher;
		String bufferedKey = _key + " Last Occured Seconds";
		if (errorTableMap.containsKey(bufferedKey)) {
			publisher = errorTableMap.get(bufferedKey);
		} else {
			publisher = errorLoggingTable.getDoubleTopic(bufferedKey).publish();
		}
		publisher.set(Timer.getFPGATimestamp());
		errorTableMap.put(bufferedKey, publisher);
	}

	public static void recordPose2d(String key, Pose2d... poses) {
		final double[] doubleArray = Arrays.stream(poses)
				.flatMapToDouble(pose -> DoubleStream.of(
						pose.getTranslation().getX(),
						pose.getTranslation().getY(),
						pose.getRotation().getRadians()))
				.toArray();
		SmartDashboard.putNumberArray(key, doubleArray);
	}

	public static void recordPose3d(String key, Pose3d... poses) {
		final double[] doubleArray = Arrays.stream(poses)
				.flatMapToDouble(pose -> DoubleStream.of(
						pose.getTranslation().getX(),
						pose.getTranslation().getY(),
						pose.getTranslation().getZ(),
						pose.getRotation().getQuaternion().getW(),
						pose.getRotation().getQuaternion().getX(),
						pose.getRotation().getQuaternion().getY(),
						pose.getRotation().getQuaternion().getZ()))
				.toArray();
		SmartDashboard.putNumberArray(key, doubleArray);
	}

	public static void recordTrajectory(String key, Trajectory trajectory) {
		ArrayList<Pose2d> poses = new ArrayList<>();
		for (int i = 1; i < trajectory.getStates().size(); ++i) { // Don't send all poses to save performance
			poses.add(trajectory.getStates().get(i).poseMeters);
		}
		recordPose2d(key, poses.toArray(new Pose2d[0]));
	}

	public static void recordTrajectory(String key, choreo.trajectory.Trajectory<?> trajectory) {
		ArrayList<Pose2d> poses = new ArrayList<>();
		for (int i = 1; i < trajectory.samples().size(); ++i) {
			poses.add(trajectory.samples().get(i).getPose());
		}
		recordPose2d(key, poses.toArray(new Pose2d[0]));
	}

	public static void timeStart(String key) {
		epochLogger.time(key, HALUtil.getFPGATime());
		SmartDashboard.putNumber(key, 0);
	}

	public static void timeEnd(String key) {
		Time time = epochLogger.timeEnd(key, HALUtil.getFPGATime());
		SmartDashboard.putNumber(key, time.in(Seconds));
	}

	public static Command time(String key, Command command) {
		return new TimedCommand(key, command);
	}

	public static final class GcStatsCollector {
		private List<GarbageCollectorMXBean> gcBeans;
		private final long[] lastTimes;
		private final long[] lastCounts;

		public GcStatsCollector() {
			gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
			lastTimes = new long[gcBeans.size()];
			lastCounts = new long[gcBeans.size()];
		}

		public void update() {
			long accumTime = 0;
			long accumCounts = 0;
			for (int i = 0; i < gcBeans.size(); i++) {
				long gcTime = gcBeans.get(i).getCollectionTime();
				long gcCount = gcBeans.get(i).getCollectionCount();
				accumTime += gcTime - lastTimes[i];
				accumCounts += gcCount - lastCounts[i];

				lastTimes[i] = gcTime;
				lastCounts[i] = gcCount;
			}

			SmartDashboard.putBoolean("Logged Robot/Garbage Collection", accumTime > 0);
			SmartDashboard.putNumber("Logged Robot/GCTimeMS", (double) accumTime);
		}
	}
}
