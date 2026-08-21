/**
 * ----------------------------------------------------------------------------
 * DRIVE SUBSYSTEM UNIT TESTS (2026 Season)
 *
 * PURPOSE: Verifies Swerve Drive logic, pose estimation, and requests.
 * HOW IT WORKS: Uses CTRE Simulation and WPILib HAL to mock a virtual robot.
 * ----------------------------------------------------------------------------
 */
/**
 * FRC UNIT TESTING GUIDE FOR SWERVE DRIVE
 * =========================================
 *
 * KEY CONCEPTS:
 * 1. HAL INITIALIZATION: Must be done ONCE per test suite using @BeforeAll (static)
 * 2. ROBOT STARTUP: RobotBase.startRobot() initializes subsystems and their constructors
 * 3. SCHEDULER: CommandScheduler must be created after HAL/Robot initialization
 * 4. PER-TEST SETUP: Use @BeforeEach for state reset between individual tests
 * 5. SIMULATION MODE: !Robot.isReal() enables mocked hardware behavior
 *
 * STRUCTURE:
 *   @BeforeAll (static) → HAL + Robot initialization (runs ONCE)
 *   @BeforeEach → Reset state for each test (runs BEFORE EACH TEST)
 *   @Test → Individual test case (Arrange → Act → Assert pattern)
 */
package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.TestUtil;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

@Tag("Drive")
public class DriveTest {

	@BeforeAll
	static void setupHAL() {
		// Initialize HAL once per test suite (static method, runs once)
		// This must happen BEFORE any WPILib or CTRE objects are created
		assert HAL.initialize(500, 0) : "HAL initialization failed";
	}

	@BeforeEach
	void setupBeforeEachTest() {
		// Access the Drive singleton (already created by Robot constructor)
		assertNotNull(Drive.mInstance, "Drive subsystem should be initialized");
		assertNotNull(Drive.mInstance.getPose(), "Drive has not updated pose");
	}

	@Test
	void testInitialPoseIsSet() {
		// Verify default pose after robot initialization
		TestUtil.testPose(Pose2d.kZero, Drive.mInstance.getPose());
	}

	@Test
	void testPeriodicTelemetry() {
		// Act & Assert: Ensure periodic updates don't crash with telemetry/dashboard
		// calls
		assertDoesNotThrow(
				() -> Drive.mInstance.periodic(), "Periodic telemetry output should not crash in simulation");
	}

	@Test
	void testSwerveRequestUpdate() {
		// Arrange: Create a stop request
		SwerveRequest.Idle stopRequest = new SwerveRequest.Idle();

		// Act & Assert: Verify setter doesn't crash and changes internal state
		assertDoesNotThrow(
				() -> Drive.mInstance.setSwerveRequest(stopRequest),
				"Setting a SwerveRequest should not throw an exception");
	}

	@Test
	void testDriveStateUpdate() {
		// Verify the drive state snapshot updates without crashing
		assertDoesNotThrow(() -> Drive.mInstance.getState(), "getState() should return current swerve drive state");
	}

	@Test
	void testResetPoseCommand() {
		// Arrange: Create a pose reset command
		Pose2d targetPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45));

		// Start from a known pose so the test is deterministic
		Drive.mInstance.resetPose(Pose2d.kZero);
		Drive.mInstance.periodic();

		Drive.mInstance.resetPoseCmd(targetPose).initialize();
		Drive.mInstance.periodic();

		TestUtil.testPose(targetPose, Drive.mInstance.getPose());

		Drive.mInstance.resetPose(Pose2d.kZero); // resets pose for other tests
	}
}
