// package frc.robot.subsystems.autoplanner;

// import static org.junit.jupiter.api.Assertions.*;

// import edu.wpi.first.hal.HAL;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.simulation.DriverStationSim;
// import edu.wpi.first.wpilibj.simulation.SimHooks;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.lib.TestUtil;
// import frc.robot.autos.AutoModeSelector;
// import frc.robot.autos.AutoModeSelector.DesiredMode;
// import frc.robot.subsystems.drive.Drive;
// import org.junit.jupiter.api.AfterEach;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Test;

// public class AutoPlannerTest {
// 	private AutoModeSelector autoModeSelector;
// 	private Drive drive;

// 	@BeforeEach
// 	void setup() {
// 		// Initialize HAL and take control of the clock
// 		assert HAL.initialize(500, 0);
// 		SimHooks.pauseTiming();

// 		// Clean slate for every test
// 		CommandScheduler.getInstance().cancelAll();

// 		drive = Drive.mInstance;

// 		autoModeSelector = new AutoModeSelector();

// 		// Setup Driver Station for Autonomous
// 		DriverStationSim.setEnabled(true);
// 		DriverStationSim.setAutonomous(true);
// 		DriverStationSim.notifyNewData();

// 		drive.resetPose(new Pose2d());
// 	}

// 	@AfterEach
// 	void tearDown() {
// 		CommandScheduler.getInstance().cancelAll();
// 		CommandScheduler.getInstance().unregisterAllSubsystems();
// 		SimHooks.resumeTiming(); // Give control back
// 	}

// 	@Test
// 	void testDriveOutMovement() {
// 		AutoPlanner autoPlanner = AutoPlanner.PURE_PURSUIT_PLANNER;

// 		for (AutoModeSelector.DesiredMode auto : AutoModeSelector.DesiredMode.values()) {
// 			// 1. Inject the mode (added to AutoModeSelector.java previously)
// 			autoModeSelector.setModeForTest(auto);

// 			// 2. Load the command
// 			Command autoCommand = autoModeSelector.getAutoAsCommand();

// 			assertNotNull(autoCommand, "The Auto Routine returned a null command.");

// 			// 3. Schedule it
// 			CommandScheduler.getInstance().schedule(autoCommand);
// 			assertTrue(autoCommand.isScheduled(), "Command failed to schedule.");

// 			// 4. Run Watchdog Loop
// 			int maxTicks = 25000;
// 			int currentTicks = 0;

// 			Pose2d targetPose = autoPlanner.getPathTrackerPose();
// 			Pose2d currentPose = drive.getPose();
// 			int posesReached = 0;

// 			while (autoCommand.isScheduled() && currentTicks < maxTicks) {
// 				TestUtil.tick(1);
// 				currentTicks++;

// 				targetPose = autoPlanner.getPathTrackerPose();

// 				currentPose = drive.getPose();

// 				// tests to see if robot has reached target pose without causing tests to fail
// 				if (TestUtil.testCurrentPose(targetPose, currentPose)) {
// 					TestUtil.testPose(targetPose, currentPose); // runs actual tests
// 					targetPose = autoPlanner.getPathTrackerPose();
// 					posesReached++; // number of target poses reached
// 				}
// 			}

// 			// ignores test if auto is DO_NOTHING
// 			if (auto != DesiredMode.DO_NOTHING) {
// 				assertTrue(posesReached >= 20); // makes sure auto was actually going places
// 			}

// 			// 5. Final Assertions
// 			TestUtil.testPose(targetPose, currentPose); // extra test to make sure auto didn't time out during movement

// 			assertFalse(currentTicks >= maxTicks, "FAIL: Auto timed out (took longer than 6 seconds).");
// 		}
// 		Drive.mInstance.resetPose(Pose2d.kZero);
// 	}
// }
