package frc.lib.mechviz.mechanism.handler;

public class CalculationsMechanismHandler extends Mechanism3dHandler {

	private final Runnable calculationFunctions[];

	public CalculationsMechanismHandler(Runnable... calculationFunctions) {
		this.calculationFunctions = calculationFunctions;
	}

	@Override
	public void updatePositions() {
		for (Runnable function : calculationFunctions) function.run();
	}
}
