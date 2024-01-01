package frc2023.behavior.routines.arm;

import java.util.Set;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.Arm;
import frc2023.subsystems.SubsystemBase;

public class ArmReZeroRoutine extends TimeoutRoutineBase {

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}

	public ArmReZeroRoutine(double time) {
		timeout = time;
	}

	@Override
	public void start(Commands commands, @ReadOnly RobotState state) {
		// Required to start the timeout timer
		mTimer.start();
		commands.wantedArmSecondStageState = Arm.SecondStageState.RE_ZERO;
	}

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return Set.of(Arm.class);
	}
}
