package frc2023.behavior.routines.arm;

import java.util.Set;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.Pivot;
import frc2023.subsystems.SubsystemBase;

public class PivotReZeroRoutineAuto extends TimeoutRoutineBase {

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}

	public PivotReZeroRoutineAuto(double time) {
		timeout = time;
	}

	@Override
	public void start(Commands commands, @ReadOnly RobotState state) {
		// Required to start the timeout timer
		mTimer.start();
		commands.wantedPivotState = Pivot.State.RE_ZERO_AUTO;
	}

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return Set.of(Pivot.class);
	}
}
