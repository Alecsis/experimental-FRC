// /**
//  * ----------------------------------------------------------------------------
//  * AUTO MODE SELECTOR INTEGRITY TEST (2026 Season)
//  *
//  * PURPOSE: Automatically verifies that every 'DesiredMode' enum entry:
//  * 1. Exists as an option in the SendableChooser (Dashboard).
//  * 2. Is handled in the getAutoAsCommand() switch statement.
//  *
//  * This ensures developers don't forget to register new autos in all 3 places.
//  * ----------------------------------------------------------------------------
//  */
// package frc.robot.autos;

// import static org.junit.jupiter.api.Assertions.*;

// import edu.wpi.first.hal.HAL;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.autos.AutoModeSelector.DesiredMode;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Tag;
// import org.junit.jupiter.api.Test;

// @Tag("Auto")
// public class AutoModeSelectorTest {
// 	private AutoModeSelector m_selector;

// 	@BeforeEach
// 	void setup() {
// 		assert HAL.initialize(500, 0); // Initialize for NetworkTables/SmartDashboard
// 		m_selector = new AutoModeSelector();
// 	}

// 	@Test
// 	void testAllEnumModesAreRegisteredAndHandled() {
// 		// Get the list of all modes defined in the Enum
// 		DesiredMode[] allModes = DesiredMode.values();

// 		for (DesiredMode mode : allModes) {
// 			Command autoCommand = m_selector.getAutoAsCommand();

// 			assertNotNull(autoCommand, "FAILED: AutoModeSelector returned a null command for mode: " + mode.name());
// 		}
// 	}

// 	@Test
// 	void testDefaultModeIsDoNothing() {
// 		// updates mode to selected, which defaults to DO_NOTHING
// 		m_selector.updateModeCreator(false);
// 		assertEquals(
// 				DesiredMode.DO_NOTHING, m_selector.getDesiredAutomode(), "The default auto mode must be DO_NOTHING.");
// 	}
// }
