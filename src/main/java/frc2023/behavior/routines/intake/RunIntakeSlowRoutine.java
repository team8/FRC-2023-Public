package frc2023.behavior.routines.intake;

import java.util.Set;

import frc2023.behavior.routines.superstructure.TimeOutRoutine;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.subsystems.Intake;
import frc2023.subsystems.SubsystemBase;

public class RunIntakeSlowRoutine extends TimeOutRoutine {

	public RunIntakeSlowRoutine() {
		super(0.1);
		mTimer.reset();
		mTimer.start();
	}

	@Override
	public void start(Commands commands, RobotState state) {
		commands.intakeRollerWantedState = Intake.RollerState.INTAKE_SLOW;
	}

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return Set.of(Intake.class);
	}
}
