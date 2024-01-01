package frc2023.subsystems;

import frc2023.config.constants.PortConstants;
import frc2023.config.subsystem.IntakeConfig;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.util.LiveGraph;
import frc2023.util.config.Configs;
import frc2023.util.control.ControllerOutput;
import frc2023.util.control.Spark;

public class Intake extends SubsystemBase {

	public enum RollerState {
		IDLE, INTAKE, OUTTAKE, CUSTOM, INTAKE_SLOW, OUTTAKE_SLOW,
	}

	private static Intake INSTANCE = new Intake();
	private ControllerOutput output = new ControllerOutput();
	private IntakeConfig config = Configs.get(IntakeConfig.class);

	public Spark roller = new Spark(PortConstants.intakeRollerID, "Intake Roller", true);

	private Intake() {
	}

	@Override
	public void update(Commands commands, RobotState state) {
		switch (commands.getIntakeRollerWantedState()) {
			case IDLE:
				output.setIdle();
				break;
			case INTAKE:
				//output.setPercentOutput(config.rollerPercentOutput);
				output.setPercentOutput(config.rollerPercentOutput);
				break;
			case OUTTAKE:
				output.setPercentOutput(-config.rollerPercentOutput);
				break;
			case CUSTOM:
				output.setPercentOutput(commands.intakeCustomPercentOutput);
				break;
			case OUTTAKE_SLOW:
				output.setPercentOutput(-config.rollerPercentOutput / 2);
				break;
			case INTAKE_SLOW:
				output.setPercentOutput(config.rollerPercentOutput / 1.5);
				break;
		}
		LiveGraph.add("Intake/rollerWantedOutput", output.getReference());
	}

	@Override
	public void writeHardware(RobotState state) {
		roller.setOutput(output);
	}

	@Override
	public void readHardware(RobotState state) {
		state.intakeRollerVelocity = roller.getAppliedOutput();
	}

	@Override
	public void configureHardware() {
		roller.restoreFactoryDefaults();
		roller.setSecondaryCurrentLimit(20);
		roller.setInverted(false);
	}

	public static Intake getInstance() {
		return INSTANCE;
	}
}
