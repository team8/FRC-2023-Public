package frc2023.behavior.routines.arm;

import java.util.Set;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.Arm;
import frc2023.subsystems.SubsystemBase;

/* TODO: make sure theres only one of these motherfrickers active at a time on this bot */
public class ArmMoveFirstStageRoutine extends TimeoutRoutineBase {

	Arm.FirstStageState finalFirstStageState;
	Arm.FirstStageState initialFirstStageState;

	/*
	 * Move first stage of arm
	 */
	public ArmMoveFirstStageRoutine(double time, Arm.FirstStageState finalState, Arm.FirstStageState initState) {
		timeout = time;
		finalFirstStageState = finalState;
		initialFirstStageState = initState;
	}

	@Override
	public void start(Commands commands, @ReadOnly RobotState state) {
		// Required to start the timeout timer
		mTimer.start();
		commands.wantedArmFirstStageState = initialFirstStageState;
	}

	@Override
	public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
	}

	@Override
	public void stop(Commands commands, @ReadOnly RobotState state) {
		commands.wantedArmFirstStageState = finalFirstStageState;
	}

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return mTimer.get() > timeout;
	}

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return Set.of(Arm.class);
	}

	public double getTimeout() {
		return timeout;
	}

}
