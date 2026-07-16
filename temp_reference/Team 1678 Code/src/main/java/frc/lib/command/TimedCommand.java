package frc.lib.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.lib.util.Stopwatch;

public class TimedCommand extends WrapperCommand {

	private final String logKey;
	private final Stopwatch stopwatch = new Stopwatch();

	public TimedCommand(String logKey, Command command) {
		super(command);
		this.logKey = logKey;
		setName("Timed " + command.getName());
		SmartDashboard.putNumber(logKey, 0);
	}

	@Override
	public void initialize() {
		super.initialize();
		stopwatch.resetAndStart();
	}

	@Override
	public void execute() {
		super.execute();
		SmartDashboard.putNumber(logKey, stopwatch.getTimeAsDouble());
	}

	@Override
	public void end(boolean interrupted) {
		super.end(interrupted);
		stopwatch.reset();
	}
}
