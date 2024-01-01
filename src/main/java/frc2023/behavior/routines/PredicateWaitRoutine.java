package frc2023.behavior.routines;

import java.util.Set;
import java.util.function.Predicate;

import frc2023.behavior.RoutineBase;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.SubsystemBase;

/*
 * A class whose run duration is 0 seconds; essentially runs until a certain predicate is reached.
 * Used to break up routines
 */
public abstract class PredicateWaitRoutine<T> extends RoutineBase {

	protected final Predicate<T> mPredicate;

	public PredicateWaitRoutine(Predicate<T> predicate) {
		this.mPredicate = predicate;
	}

	@Override
	public abstract boolean checkFinished(@ReadOnly RobotState state);

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return null;
	}
}
