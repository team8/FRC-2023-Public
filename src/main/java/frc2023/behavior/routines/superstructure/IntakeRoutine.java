package frc2023.behavior.routines.superstructure;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.util.LiveGraph;

public class IntakeRoutine extends TimeoutRoutineBase {

	public IntakeRoutine(double durationSeconds) {
		super(durationSeconds);
	}

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}

	@Override
	protected void update(Commands commands, RobotState state) {
		LiveGraph.add("Routines/Intaking");
	}

	@Override
	public void onTimeout() {
		LiveGraph.add("Routines/Intaking", false);
	}
}
