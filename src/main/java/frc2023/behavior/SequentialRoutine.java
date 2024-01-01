package frc2023.behavior;

import java.util.Iterator;
import java.util.List;
import java.util.Set;

import com.esotericsoftware.minlog.Log;

import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.SubsystemBase;

/**
 * Runs routines one at a time. Finishes when the last one is finished.
 */
public class SequentialRoutine extends MultipleRoutineBase {

	private final Iterator<RoutineBase> mIterator = mRoutines.iterator();
	private RoutineBase mRunningRoutine = mIterator.next();

	public SequentialRoutine(RoutineBase... routines) {
		super(routines);
	}

	public SequentialRoutine(List<RoutineBase> routines) {
		super(routines);
	}

	@Override
	public void update(Commands commands, @ReadOnly RobotState state) {
		while (mRunningRoutine.execute(commands, state)) {
			if (!mIterator.hasNext()) {
				mRunningRoutine = null;
				break;
			}
			mRunningRoutine = mIterator.next();
			Log.info(String.format("Moving onto next routine: %s", mRunningRoutine));
		}
	}

	@Override
	protected void stop(Commands commands, @ReadOnly RobotState state) {
		if (mRunningRoutine != null) mRunningRoutine.stop(commands, state);
	}

	@Override
	public boolean checkFinished(@ReadOnly RobotState state) {
		return mRunningRoutine == null;
	}

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return RoutineManager.sharedSubsystems(mRoutines);
	}
}
