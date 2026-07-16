package frc.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class TestUtil {
	/**
	 * Advances the world by 20ms per iteration.
	 * This triggers periodic() methods and updates timers.
	 */
	public static void tick(int iterations) {
		for (int i = 0; i < iterations; i++) {
			CommandScheduler.getInstance().run();
			SimHooks.stepTiming(0.020);
		}
	}

	// tests all necessary pose things
	public static void testPose(Pose2d targetPose, Pose2d currentPose) {
		assertEquals(targetPose.getX(), currentPose.getX(), 0.2, "X out of tolerance");
		assertEquals(targetPose.getY(), currentPose.getY(), 0.2, "Y out of tolerance");
		assertEquals(
				targetPose.getRotation().getDegrees(),
				currentPose.getRotation().getDegrees(),
				5.0,
				"Rotation out of tolerance");
	}

	// to test poses without causing tests to fail
	public static boolean testCurrentPose(Pose2d targetPose, Pose2d currentPose) {
		return Math.abs(targetPose.getX() - currentPose.getX()) <= 0.2
				&& Math.abs(targetPose.getY() - currentPose.getY()) <= 0.2
				&& Math.abs(targetPose.getRotation().getDegrees()
								- currentPose.getRotation().getDegrees())
						<= 5.0;
	}
}
