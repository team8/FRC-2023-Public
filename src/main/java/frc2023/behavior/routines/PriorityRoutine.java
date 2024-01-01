package frc2023.behavior.routines;

import frc2023.behavior.RoutineBase;

public class PriorityRoutine implements Comparable<PriorityRoutine> {

	private RoutineBase mRoutine;
	private int mPriority;

	public PriorityRoutine(RoutineBase routine, int priority) {
		mRoutine = routine;
		mPriority = priority;
	}

	public RoutineBase getRoutine() {
		return mRoutine;
	}

	@Override
	public int compareTo(PriorityRoutine o) {
		return o.mPriority > this.mPriority ? -1 : 1;
	}
}
