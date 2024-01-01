package frc2023.behavior.routines.intake;

import java.util.Set;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.subsystems.Intake;
import frc2023.subsystems.SubsystemBase;

public class RunIntakeTimedRoutine extends TimeoutRoutineBase {

	public RunIntakeTimedRoutine(double durationSeconds) {
		super(durationSeconds);
		mTimer.reset();
		mTimer.start();
	}

	@Override
	public void start(Commands commands, RobotState state) {
		super.start(commands, state);
		commands.intakeRollerWantedState = Intake.RollerState.INTAKE;
	}

	@Override
	public void stop(Commands commands, RobotState state) {
		Intake.getInstance().roller.setSecondaryCurrentLimit(20);
		commands.intakeRollerWantedState = Intake.RollerState.INTAKE_SLOW;
	}

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return Set.of(Intake.class);
	}

	@Override
	public void update(Commands commands, RobotState state) {
		// this should be between 0.1 and 0.2 seconds lol
		if (timeout / 6 < mTimer.get() && mTimer.get() < timeout / 3) {
			Intake.getInstance().roller.setSecondaryCurrentLimit(40);
		}

	}
}
