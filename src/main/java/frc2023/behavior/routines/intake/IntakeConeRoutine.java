package frc2023.behavior.routines.intake;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.behavior.routines.arm.ArmMoveToPosRoutineTwo;
import frc2023.config.subsystem.IntakeConfig;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.Arm;
import frc2023.subsystems.Intake;
import frc2023.subsystems.Pivot;
import frc2023.util.config.Configs;

public class IntakeConeRoutine extends TimeoutRoutineBase {

	IntakeConfig intakeConfig = Configs.get(IntakeConfig.class);

	public IntakeConeRoutine(double time) {
		timeout = time;
	}

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}

	@Override
	public void start(Commands commands, @ReadOnly RobotState state) {
		// Required to start the timeout timer
		commands.wantedPivotState = Pivot.State.SET_POINT;
		commands.wantedPivotAngle = 62;
		commands.addWantedRoutine(new ArmMoveToPosRoutineTwo(-160, 0.5, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED));
		commands.intakeRollerWantedState = Intake.RollerState.INTAKE;
		mTimer.start();
	}

	@Override
	public void stop(Commands commands, @ReadOnly RobotState state) {
		commands.intakeRollerWantedState = Intake.RollerState.INTAKE;
	}
}
