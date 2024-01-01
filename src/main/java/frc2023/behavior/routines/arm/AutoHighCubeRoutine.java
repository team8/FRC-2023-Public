package frc2023.behavior.routines.arm;

import frc2023.behavior.ParallelRoutine;
import frc2023.behavior.SequentialRoutine;
import frc2023.behavior.TimeoutRoutineBase;
import frc2023.behavior.routines.drive.AutoDriveRoutine;
import frc2023.behavior.routines.intake.IntakeOuttakeRoutine;
import frc2023.behavior.routines.superstructure.TimeOutRoutine;
import frc2023.config.constants.PivotConstants;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.subsystems.Arm;

public class AutoHighCubeRoutine extends TimeoutRoutineBase {

	private ParallelRoutine cubeHighRoutine;
	private ParallelRoutine cubeHighRetractRoutine;

	@Override
	public void start(Commands commands, RobotState state) {
		cubeHighRoutine = new ParallelRoutine(new ArmMoveToPosRoutineTwo(-80.0, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED), new PivotSetPointRoutine(-14.6, 0.6, 0.5));
		cubeHighRetractRoutine = new ParallelRoutine(
				new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.1, 0.1),
				new SequentialRoutine(
						new TimeOutRoutine(0.5),
						new ParallelRoutine(
								new AutoDriveRoutine(0.4, -1),
								new ArmMoveToPosRoutineTwo(-160, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED))));
		var highCubePlaceRoutine = new SequentialRoutine(cubeHighRoutine, new IntakeOuttakeRoutine(0.3), cubeHighRetractRoutine);
		commands.addWantedRoutine(highCubePlaceRoutine);

	}

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}
}
