package frc2023.behavior.routines.intake;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.Intake;

public class IntakeVisionRoutine extends TimeoutRoutineBase {

	@Override
	public void start(Commands commands, @ReadOnly RobotState state) {
		commands.intakeRollerWantedState = Intake.RollerState.INTAKE;
	}

	@Override
	public void stop(Commands commands, @ReadOnly RobotState state) {
		commands.intakeRollerWantedState = Intake.RollerState.IDLE;
	}

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}
}
