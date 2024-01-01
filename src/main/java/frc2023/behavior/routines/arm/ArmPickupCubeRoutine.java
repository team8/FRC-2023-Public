package frc2023.behavior.routines.arm;

import frc2023.behavior.ParallelRoutine;
import frc2023.behavior.TimeoutRoutineBase;
import frc2023.behavior.routines.intake.RunIntakeRoutine;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.subsystems.Arm;

public class ArmPickupCubeRoutine extends TimeoutRoutineBase {

	@Override
	public void start(Commands commands, RobotState state) {
		var wanted = new ParallelRoutine(
				new ArmMoveToPosRoutineTwo(-122.32, 0.5, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED),
				new PivotSetPointRoutine(-11, 0.3, 0.3),
				new RunIntakeRoutine());
		commands.addWantedRoutine(wanted);

	}

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}
}
