// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

/** End-to-end test of the physical control-box arbitration. One test method is intentional: the
 * singleton subsystems share a JVM within a test class (see build.gradle's forkEvery note). */
class OperatorControlsIntentTest {
  private Robot robot;
  private Thread robotThread;

  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.resetData();
    SimHooks.pauseTiming();

    robot = new Robot();
    robotThread = SimRobotLoop.start(robot, "OperatorControlsIntentTest-competition");
    SimRobotLoop.awaitProgramStart();
    SimRobotLoop.registerDiagnostics(() -> SimRobotLoop.operatorBoardState(1, 6));

    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setJoystickButtonCount(1, 6);
    setControlBoxButtons();
    step(0.1);
  }

  @AfterEach
  void teardown() {
    SimRobotLoop.shutdown();
  }

  @Test
  @Timeout(30)
  void maintainedIntentsResumeAfterMomentaryOverridesAndFallbackFeedsAtSpeed() {
    Intake intake = Intake.getInstance();
    Shooter shooter = Shooter.getInstance();
    Superstructure superstructure = Superstructure.getInstance(null);

    // Button 6: physical maintained intake, and release returns to current stowed policy.
    setControlBoxButtons(6);
    step(0.1);
    assertEquals(Superstructure.SuperstructureState.INTAKING, superstructure.getSystemState());
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState());
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState());

    setControlBoxButtons();
    step(0.1);
    assertEquals(Superstructure.SuperstructureState.STOWED, superstructure.getSystemState());
    assertEquals(Intake.Roller.STOP, intake.getRollerState());
    assertEquals(700.0, shooter.getTargetRPM());

    // Button 2 and button 3 temporarily win over button 6, which resumes without re-toggling.
    setControlBoxButtons(2, 6);
    step(0.1);
    assertTrue(superstructure.getSystemState() == Superstructure.SuperstructureState.ALIGNING
        || superstructure.getSystemState() == Superstructure.SuperstructureState.SHOOTING);
    setControlBoxButtons(6);
    step(0.1);
    assertEquals(Superstructure.SuperstructureState.INTAKING, superstructure.getSystemState());
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState());
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState());

    setControlBoxButtons(3, 6);
    step(0.1);
    assertEquals(Superstructure.SuperstructureState.EJECTING, superstructure.getSystemState());
    assertEquals(Intake.Roller.EJECT, intake.getRollerState());
    assertEquals(Shooter.indexing.EJECT, shooter.getIndexerState());
    assertEquals(Shooter.Agitate.OUT, shooter.getAgitateState());
    setControlBoxButtons(6);
    step(0.1);
    assertEquals(Superstructure.SuperstructureState.INTAKING, superstructure.getSystemState());

    // Button 5 owns the special pivot routine and roller; it outranks button 6 without a storm.
    setControlBoxButtons(5, 6);
    step(0.1);
    assertEquals(Superstructure.SuperstructureState.BOUNCING, superstructure.getSystemState());
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState());
    assertTrue(intake.agitatePivot().getRequirements().contains(intake));
    step(0.6); // allow the Intake's existing post-eject jam-recovery cooldown to elapse
    assertEquals(Superstructure.SuperstructureState.BOUNCING, superstructure.getSystemState(),
        "holding the bounce switch must leave one stable bounce intent, not a cancellation storm");
    assertEquals(Intake.Roller.INTAKE, intake.getRollerState());

    setControlBoxButtons(2, 5);
    step(0.1);
    assertTrue(superstructure.getSystemState() == Superstructure.SuperstructureState.ALIGNING
        || superstructure.getSystemState() == Superstructure.SuperstructureState.SHOOTING);
    setControlBoxButtons(5);
    step(0.1);
    assertEquals(Superstructure.SuperstructureState.BOUNCING, superstructure.getSystemState());

    setControlBoxButtons(3, 5);
    step(0.1);
    assertEquals(Superstructure.SuperstructureState.EJECTING, superstructure.getSystemState());
    setControlBoxButtons(5);
    step(0.1);
    assertEquals(Superstructure.SuperstructureState.BOUNCING, superstructure.getSystemState());

    setControlBoxButtons(6);
    step(0.1);
    assertEquals(Superstructure.SuperstructureState.INTAKING, superstructure.getSystemState());
    setControlBoxButtons();
    step(0.1);
    assertEquals(Superstructure.SuperstructureState.STOWED, superstructure.getSystemState());

    // Button 1 is a fixed 1600 rotor-RPM shot: no feed while aligning, then normal index + IN.
    setControlBoxButtons(1);
    step(0.1);
    assertEquals(1600.0, shooter.getTargetRPM());
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState(),
        "fallback shot must wait for shooterAtSpeed before feeding");
    step(3.0);
    assertEquals(Shooter.indexing.INDEX, shooter.getIndexerState());
    assertEquals(Shooter.Agitate.IN, shooter.getAgitateState());

    // Releasing fallback stops feed and restores a still-physical maintained bounce/intake intent.
    setControlBoxButtons(1, 5);
    step(0.1);
    setControlBoxButtons(5);
    step(0.1);
    assertEquals(Superstructure.SuperstructureState.BOUNCING, superstructure.getSystemState());
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState());

    setControlBoxButtons(1, 6);
    step(0.1);
    setControlBoxButtons(6);
    step(0.1);
    assertEquals(Superstructure.SuperstructureState.INTAKING, superstructure.getSystemState());
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState());

    // Button 4 remains reserved: it cannot affect the normal idle/stowed state.
    setControlBoxButtons();
    step(0.1);
    setControlBoxButtons(4);
    step(0.1);
    assertEquals(Superstructure.SuperstructureState.STOWED, superstructure.getSystemState());
    assertEquals(Intake.Roller.STOP, intake.getRollerState());
    assertEquals(Shooter.indexing.STOP, shooter.getIndexerState());
  }

  private static void setControlBoxButtons(int... buttons) {
    int mask = 0;
    for (int button : buttons) {
      mask |= 1 << (button - 1);
    }
    DriverStationSim.setJoystickButtons(1, mask);
    DriverStationSim.notifyNewData();
  }

  private static void step(double seconds) {
    SimRobotLoop.step(seconds);
  }
}
