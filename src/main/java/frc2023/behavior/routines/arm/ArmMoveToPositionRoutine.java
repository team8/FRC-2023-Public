package frc2023.behavior.routines.arm;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.Arm;

public class ArmMoveToPositionRoutine extends TimeoutRoutineBase {

	double finalSecondStageAngle;
	Arm.FirstStageState finalFirstStageState;
	Arm.FirstStageState initialFirstStageState;
	ArmSecondStageTrajectoryRoutine trajRoutine;

	public ArmMoveToPositionRoutine(double endAngle, double time, Arm.FirstStageState finalState, Arm.FirstStageState initState) {
		finalSecondStageAngle = endAngle;
		timeout = time;
		finalFirstStageState = finalState;
		initialFirstStageState = initState;
		trajRoutine = new ArmSecondStageTrajectoryRoutine(endAngle, time);
	}

	public void start(Commands commands, @ReadOnly RobotState state) {
		// Required to start the timeout timer
		mTimer.start();
		commands.wantedArmFirstStageState = initialFirstStageState;
		commands.addWantedRoutine(trajRoutine);
	}

	@Override
	public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
		timeout = trajRoutine.getTimeout();
	}

	@Override
	public void stop(Commands commands, @ReadOnly RobotState state) {
		commands.wantedArmFirstStageState = finalFirstStageState;
	}

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}
}
