package frc2023.behavior.routines.arm;

import java.util.Set;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.Pivot;
import frc2023.subsystems.SubsystemBase;

public class PivotSetPointRoutine extends TimeoutRoutineBase {

	double setpoint = 0.0;
	double timeBeforeExtend;

	public PivotSetPointRoutine(double point, double time, double timeBefore) {
		timeout = time;
		setpoint = point;
		timeBeforeExtend = timeBefore;
	}

	@Override
	public void start(Commands commands, @ReadOnly RobotState state) {
		mTimer.reset();
		mTimer.start();

	}

	@Override
	public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
		if (mTimer.get() >= timeout - timeBeforeExtend) {
			commands.wantedPivotState = Pivot.State.SET_POINT;
			commands.wantedPivotAngle = setpoint;
		} else {
			commands.wantedPivotState = Pivot.State.STOW;
		}
	}

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return Set.of(Pivot.class);
	}
}
