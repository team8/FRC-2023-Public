package frc2023.behavior.routines.arm;

import java.util.Set;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.Arm;
import frc2023.subsystems.Pivot;
import frc2023.subsystems.SubsystemBase;

public class ArmResetRoutine extends TimeoutRoutineBase {

	private double timeAfterPivoting;
	private boolean doneBringingArm = false;

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}

	public ArmResetRoutine(double timeAfterPivot) {
		timeAfterPivoting = timeAfterPivot;
		timeout = timeAfterPivoting + 0.6;
	}

	@Override
	public void start(Commands commands, @ReadOnly RobotState state) {
		// Required to start the timeout timer
		mTimer.start();
		commands.wantedPivotState = Pivot.State.STOW;
	}

	@Override
	public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
		if (mTimer.get() > timeAfterPivoting && !doneBringingArm) {
			doneBringingArm = true;
			commands.addWantedRoutine(new ArmMoveToPosRoutineTwo(-160, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED));
		}
	}

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return Set.of(Arm.class);
	}
}
