package frc.robot.autos;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.midline.LeftSweep;
import frc.robot.autos.midline.LeftSweepAlternate;
import frc.robot.autos.midline.LeftSweepCloseAlternate;
import frc.robot.autos.midline.LeftSweepCutCorner;
import frc.robot.autos.midline.LeftSweepCutCornerAlternate;
import frc.robot.autos.midline.RightSweep;
import frc.robot.autos.midline.RightSweepAlternate;
import frc.robot.autos.midline.RightSweepCloseAlternate;
import frc.robot.autos.midline.RightSweepCutCorner;
import frc.robot.autos.midline.RightSweepCutCornerAlternate;
import frc.robot.autos.midline.single.LeftSingleSweep;
import frc.robot.autos.midline.single.RightSingleSweep;

public class AutoModeSelector {
	private AutoChooser mAutoChooser = new AutoChooser();

	private Pose2d startPose = new Pose2d();

	public AutoModeSelector(AutoFactory factory) {
		mAutoChooser.addRoutine("[LEFT] Double Sweep", () -> {
			LeftSweep auto = new LeftSweep(factory);
			startPose = auto.getInitialPose();
			return auto.getRoutine();
		});
		mAutoChooser.addRoutine("[LEFT] Single Sweep + Climb", () -> {
			LeftSingleSweep auto = new LeftSingleSweep(factory);
			startPose = auto.getInitialPose();
			return auto.getRoutine();
		});
		mAutoChooser.addRoutine("[RIGHT] Double Sweep", () -> {
			RightSweep auto = new RightSweep(factory);
			startPose = auto.getInitialPose();
			return auto.getRoutine();
		});
		mAutoChooser.addRoutine("[LEFT] Alternate Double Sweep", () -> {
			LeftSweepAlternate auto = new LeftSweepAlternate(factory);
			startPose = auto.getInitialPose();
			return auto.getRoutine();
		});
		mAutoChooser.addRoutine("[RIGHT] Alternate Double Sweep", () -> {
			RightSweepAlternate auto = new RightSweepAlternate(factory);
			startPose = auto.getInitialPose();
			return auto.getRoutine();
		});

		mAutoChooser.addRoutine("[LEFT] Close Alternate Double Sweep", () -> {
			LeftSweepCloseAlternate auto = new LeftSweepCloseAlternate(factory);
			startPose = auto.getInitialPose();
			return auto.getRoutine();
		});
		mAutoChooser.addRoutine("[RIGHT] Close Alternate Double Sweep", () -> {
			RightSweepCloseAlternate auto = new RightSweepCloseAlternate(factory);
			startPose = auto.getInitialPose();
			return auto.getRoutine();
		});
		mAutoChooser.addRoutine("[RIGHT] Single Sweep + Climb", () -> {
			RightSingleSweep auto = new RightSingleSweep(factory);
			startPose = auto.getInitialPose();
			return auto.getRoutine();
		});
		mAutoChooser.addRoutine("[LEFT] Cut Corner Double Sweep", () -> {
			LeftSweepCutCorner auto = new LeftSweepCutCorner(factory);
			startPose = auto.getInitialPose();
			return auto.getRoutine();
		});
		mAutoChooser.addRoutine("[RIGHT] Cut Corner Double Sweep", () -> {
			RightSweepCutCorner auto = new RightSweepCutCorner(factory);
			startPose = auto.getInitialPose();
			return auto.getRoutine();
		});
		mAutoChooser.addRoutine("[RIGHT] Alternate Cut Corner Double Sweep", () -> {
			RightSweepCutCornerAlternate auto = new RightSweepCutCornerAlternate(factory);
			startPose = auto.getInitialPose();
			return auto.getRoutine();
		});

		mAutoChooser.addRoutine("[Left] Alternate Cut Corner Double Sweep", () -> {
			LeftSweepCutCornerAlternate auto = new LeftSweepCutCornerAlternate(factory);
			startPose = auto.getInitialPose();
			return auto.getRoutine();
		});

		// mAutoChooser.addRoutine("Do Nothing", () -> new Empty(factory).getRoutine());
		SmartDashboard.putData(mAutoChooser);
	}

	public Command getSelectedCommand() {
		return mAutoChooser.selectedCommand();
	}

	public AutoChooser getAutoChooser() {
		return mAutoChooser;
	}

	public Pose2d getSelectedAutoStartingPose() {
		return startPose;
	}
}
