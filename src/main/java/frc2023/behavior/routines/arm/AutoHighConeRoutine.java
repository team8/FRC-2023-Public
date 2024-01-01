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

public class AutoHighConeRoutine extends TimeoutRoutineBase {

	private ParallelRoutine coneHighRoutine;
	private ParallelRoutine coneHighRetractRoutine;

	@Override
	public void start(Commands commands, RobotState state) {
		coneHighRoutine = new ParallelRoutine(new ArmMoveSecondStageRoutine(-31, 0.6), new SequentialRoutine(new TimeOutRoutine(0.4), new ArmMoveFirstStageRoutine(0.2, Arm.FirstStageState.LOWER, Arm.FirstStageState.LOWER)), new PivotSetPointRoutine(-79, 0.3, 0.1));
		coneHighRetractRoutine = new ParallelRoutine(
				new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.3, 0.3),
				new SequentialRoutine(
						new TimeOutRoutine(0.3),
						new ParallelRoutine(
								new AutoDriveRoutine(0.65, -1),
								new ArmMoveSecondStageRoutine(-160, 0.5),
								new SequentialRoutine(
										new TimeOutRoutine(0.54),
										new ArmMoveFirstStageRoutine(0.1, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED)))));
		var highConePlaceRoutine = new SequentialRoutine(coneHighRoutine, new IntakeOuttakeRoutine(0.3), coneHighRetractRoutine);
		commands.addWantedRoutine(highConePlaceRoutine);

	}

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}
}
