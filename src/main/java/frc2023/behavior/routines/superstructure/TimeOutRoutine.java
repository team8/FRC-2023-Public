package frc2023.behavior.routines.superstructure;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;

public class TimeOutRoutine extends TimeoutRoutineBase {

	public TimeOutRoutine(double durationSeconds) {
		super(durationSeconds);
	}

	@Override
	public void update(Commands commands, RobotState state) {
	}

	public void onTimeout() {
	}

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}
}
