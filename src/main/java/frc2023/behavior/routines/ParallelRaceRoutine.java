package frc2023.behavior.routines;

import java.util.List;

import frc2023.behavior.ParallelRoutine;
import frc2023.behavior.RoutineBase;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;

public class ParallelRaceRoutine extends ParallelRoutine {

	public ParallelRaceRoutine(RoutineBase... routines) {
		super(routines);
	}

	public ParallelRaceRoutine(List<RoutineBase> routines) {
		super(routines);
	}

	@Override
	public boolean checkFinished(@ReadOnly RobotState state) {
		return mRoutines.stream().anyMatch(RoutineBase::isFinished);
	}
}
