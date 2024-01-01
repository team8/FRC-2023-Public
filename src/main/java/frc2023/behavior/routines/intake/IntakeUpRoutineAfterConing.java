package frc2023.behavior.routines.intake;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.Intake;
import frc2023.subsystems.Pivot;

public class IntakeUpRoutineAfterConing extends TimeoutRoutineBase {

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}

	public IntakeUpRoutineAfterConing() {
		timeout = 0.3;
	}

	@Override
	public void start(Commands commands, @ReadOnly RobotState state) {
		// Required to start the timeout timer
		commands.wantedPivotState = Pivot.State.SET_POINT;
		commands.wantedPivotAngle = 62;
		commands.intakeRollerWantedState = Intake.RollerState.INTAKE;
		mTimer.start();
	}

	@Override
	public void stop(Commands commands, @ReadOnly RobotState state) {
		commands.wantedPivotState = Pivot.State.SET_POINT;
		commands.wantedPivotAngle = 82;
		commands.intakeRollerWantedState = Intake.RollerState.INTAKE_SLOW;
	}
}
