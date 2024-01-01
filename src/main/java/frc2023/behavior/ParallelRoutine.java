package frc2023.behavior;

import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import com.esotericsoftware.minlog.Log;

import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.SubsystemBase;

/**
 * Runs all routines at the same time. Finishes when all routines are finished.
 */
public class ParallelRoutine extends MultipleRoutineBase {

	protected final LinkedList<RoutineBase> mRunningRoutines = new LinkedList<>(mRoutines);

	public ParallelRoutine(RoutineBase... routines) {
		super(routines);
	}

	public ParallelRoutine(List<RoutineBase> routines) {
		super(routines);
	}

	@Override
	public void update(Commands commands, @ReadOnly RobotState state) {
		mRunningRoutines.removeIf(runningRoutine -> {
			boolean isFinished = runningRoutine.execute(commands, state);
			if (isFinished) {
				Log.debug(getName(), String.format("Dropping routine: %s", runningRoutine.getName()));
			}
			return isFinished;
		});
	}

	@Override
	protected void stop(Commands commands, @ReadOnly RobotState state) {
		for (RoutineBase runningRoutine : mRunningRoutines) {
			runningRoutine.stop(commands, state);
		}
	}

	@Override
	public boolean checkFinished(@ReadOnly RobotState state) {
		return mRunningRoutines.isEmpty();
	}

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return RoutineManager.sharedSubsystems(mRunningRoutines);
	}
}
