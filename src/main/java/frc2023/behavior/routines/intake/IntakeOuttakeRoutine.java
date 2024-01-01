package frc2023.behavior.routines.intake;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.config.subsystem.IntakeConfig;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.Intake;
import frc2023.util.config.Configs;

public class IntakeOuttakeRoutine extends TimeoutRoutineBase {

	IntakeConfig intakeConfig = Configs.get(IntakeConfig.class);

	public IntakeOuttakeRoutine(double time) {
		timeout = time;
	}

	@Override
	public void start(Commands commands, @ReadOnly RobotState state) {
		mTimer.reset();
		mTimer.start();
		commands.intakeRollerWantedState = Intake.RollerState.OUTTAKE_SLOW;
	}

	@Override
	public void stop(Commands commands, @ReadOnly RobotState state) {
		commands.intakeRollerWantedState = Intake.RollerState.IDLE;
	}

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}
}
