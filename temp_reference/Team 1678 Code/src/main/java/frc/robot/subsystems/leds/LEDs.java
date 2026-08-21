package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.bases.LightsSubsystem;
import frc.robot.controlboard.ControlBoardConstants;
import frc.robot.shooting.Shooting;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.vision.Cameras;

public class LEDs extends LightsSubsystem {

	public static final LEDs mInstance = new LEDs();

	public LEDs() {
		super("LEDs", LEDsConstants.getLightsIO());
	}

	@Override
	public void periodic() {

		Time lastVisionUpdateDelta =
				Seconds.of(Timer.getFPGATimestamp()).minus(Cameras.mInstance.getLastUpdatedPoseTime());
		if (lastVisionUpdateDelta.lte(Seconds.of(0.5))) {
			if (Cameras.mInstance.getPoseStable()) {
				addWantedState(LEDsConstants.STATE_VISION_POSE_STABLE);
			} else {
				addWantedState(LEDsConstants.STATE_VISION_ENABLED);
			}
		} else {
			addWantedState(LEDsConstants.STATE_VISION_DISABLED);
		}

		if (!Drive.mInstance.getGeneratedDrive().getPigeon2().isConnected()) {
			addWantedState(LEDsConstants.STATE_PIGEON_DISCONNECTED);
		}

		if (!ControlBoardConstants.mDriverController.rightTrigger().getAsBoolean()) {
			addWantedState(LEDsConstants.STATE_SIDES_EMPTY, LEDsConstants.STATE_BACK_EMPTY);
		}

		if (ControlBoardConstants.mDriverController.rightTrigger().getAsBoolean()) {
			if (Shooting.mInstance.shotVerified()) {
				addWantedState(LEDsConstants.STATE_ALLOW_SHOOTING);
			} else {
				addWantedState(LEDsConstants.STATE_NOT_ALLOW_SHOOTING);
			}
		} else if (IntakeRollers.mInstance.getSetpoint() == IntakeRollers.INTAKE) {
			addWantedState(LEDsConstants.STATE_INTAKING);
		}

		super.update();
	}
}
