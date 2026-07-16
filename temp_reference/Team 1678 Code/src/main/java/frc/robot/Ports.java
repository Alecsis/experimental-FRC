package frc.robot;

import com.ctre.phoenix6.CANBus;

public enum Ports {
	INTAKE_ROLLERS_MAIN(8, RobotConstants.rio),
	INTAKE_ROLLERS_FOLLOWER(9, RobotConstants.rio),
	INTAKE_DEPLOY(10, RobotConstants.canivore1),
	HOPPER_ROLLERS(11, RobotConstants.canivore1),
	CLIMBER(12, RobotConstants.canivore1),
	FEEDER_ROLLERS(13, RobotConstants.canivore1),
	TUNNEL_ROLLERS_MAIN(14, RobotConstants.canivore1),
	TUNNEL_ROLLERS_FOLLOWER(15, RobotConstants.canivore1),
	SHOOTER_MAIN(16, RobotConstants.canivore1),
	SHOOTER_FOLLOWER_1(17, RobotConstants.canivore1),
	SHOOTER_FOLLOWER_2(18, RobotConstants.canivore1),
	SHOOTER_FOLLOWER_3(19, RobotConstants.canivore1),
	HOOD(20, RobotConstants.canivore1),

	CANDLE(21, RobotConstants.isEpsilon ? RobotConstants.rio : RobotConstants.canivore1),

	HOPPER_CANRANGE(22, RobotConstants.canivore1),
	TUNNEL_CANRANGE(23, RobotConstants.canivore1);

	public final int id;
	public final CANBus bus;

	private Ports(int id, CANBus bus) {
		this.id = id;
		this.bus = bus;
	}
}
