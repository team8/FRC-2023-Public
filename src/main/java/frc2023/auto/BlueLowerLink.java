package frc2023.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc2023.behavior.ParallelRoutine;
import frc2023.behavior.RoutineBase;
import frc2023.behavior.SequentialRoutine;
import frc2023.behavior.routines.arm.*;
import frc2023.behavior.routines.drive.AutoDriveRoutine;
import frc2023.behavior.routines.drive.DrivePathPlannerRoutine;
import frc2023.behavior.routines.drive.ResetOdometry;
import frc2023.behavior.routines.intake.IntakeOuttakeRoutine;
import frc2023.behavior.routines.intake.RunIntakeRoutine;
import frc2023.behavior.routines.superstructure.TimeOutRoutine;
import frc2023.config.constants.PivotConstants;
import frc2023.subsystems.Arm;

// Two Game Piece auto starting from lower blue
public class BlueLowerLink extends AutoBase {

	@Override
	public RoutineBase getRoutine() {
		var firstPath = new DrivePathPlannerRoutine("BlueLowerToDNew", 4.5, 3);
		var secondPath = new DrivePathPlannerRoutine("BlueDToScore", 4.5, 3);
		var thirdPath = new DrivePathPlannerRoutine("BlueLowerScoreToC", 4.5, 3);
		var finalPath = new DrivePathPlannerRoutine("BlueLowerCToScore", 4.5, 3);

		var cubeHighRoutine = new ParallelRoutine(new ArmMoveToPosRoutineTwo(-80.0, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED), new PivotSetPointRoutine(-14.6, 0.6, 0.5));
		var cubeHighRetractRoutine = new ParallelRoutine(
				new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.1, 0.1),
				new ArmMoveToPosRoutineTwo(-160, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED));
		var cubeHighPlaceRoutine = new SequentialRoutine(cubeHighRoutine, new TimeOutRoutine(0.3), new IntakeOuttakeRoutine(0.3));

		var pickupCone = new ParallelRoutine(
				new ArmMoveToPosRoutineTwo(-139, 0.5, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED),
				new PivotSetPointRoutine(17.5, 0.5, 0.5),
				new RunIntakeRoutine());
		var pickupCone1 = new ParallelRoutine(
				new ArmMoveToPosRoutineTwo(-139, 0.5, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED),
				new PivotSetPointRoutine(17.5, 0.5, 0.5),
				new RunIntakeRoutine());

		var resetArm = new ParallelRoutine(new ArmMoveToPosRoutineTwo(-160, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED), new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.1, 0.1));
		var resetArm1 = new ParallelRoutine(new ArmMoveToPosRoutineTwo(-160, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED), new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.1, 0.1));

		var coneHighRoutine = new ParallelRoutine(new ArmMoveSecondStageRoutine(-29, 0.6), new SequentialRoutine(new TimeOutRoutine(0.4), new ArmMoveFirstStageRoutine(0.2, Arm.FirstStageState.LOWER, Arm.FirstStageState.LOWER)), new PivotSetPointRoutine(-79, 0.1, 0.1));
		var coneHighRoutine1 = new ParallelRoutine(new ArmMoveSecondStageRoutine(-29, 0.6), new SequentialRoutine(new TimeOutRoutine(0.4), new ArmMoveFirstStageRoutine(0.2, Arm.FirstStageState.LOWER, Arm.FirstStageState.LOWER)), new PivotSetPointRoutine(-79, 0.1, 0.1));
		var coneHighRetractRoutine = new ParallelRoutine(
				new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.3, 0.3),
				new SequentialRoutine(
						new TimeOutRoutine(0.3),
						new ParallelRoutine(
								new ArmMoveSecondStageRoutine(-160, 0.5),
								new SequentialRoutine(
										new TimeOutRoutine(0.54),
										new ArmMoveFirstStageRoutine(0.1, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED)))));
		var coneHighRetractRoutineDrive = new ParallelRoutine(
				new AutoDriveRoutine(0.3, 1),
				new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.3, 0.3),
				new SequentialRoutine(
						new TimeOutRoutine(0.3),
						new ParallelRoutine(
								new ArmMoveSecondStageRoutine(-160, 0.5),
								new SequentialRoutine(
										new TimeOutRoutine(0.54),
										new ArmMoveFirstStageRoutine(0.1, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED)))));
		var highConePlaceRoutine = new SequentialRoutine(coneHighRoutine, new TimeOutRoutine(0.3), new IntakeOuttakeRoutine(0.3));
		var highConePlaceRoutine1 = new SequentialRoutine(coneHighRoutine1, new TimeOutRoutine(0.3), new IntakeOuttakeRoutine(0.3));
		return new SequentialRoutine(
				new ResetOdometry(new Pose2d(1.80, 1.06, Rotation2d.fromDegrees(180))),
				new RunIntakeRoutine(),
				cubeHighPlaceRoutine,
				new ParallelRoutine(
						new SequentialRoutine(new TimeOutRoutine(0.2), cubeHighRetractRoutine),
						new ParallelRoutine(firstPath, new SequentialRoutine(new TimeOutRoutine(1.7), pickupCone))),
				//new RunIntakeRoutine(),
				new ParallelRoutine(resetArm, new SequentialRoutine(new TimeOutRoutine(0.0), new ParallelRoutine(secondPath, new SequentialRoutine(new TimeOutRoutine(1.7), highConePlaceRoutine)))),
				new ParallelRoutine(new SequentialRoutine(new TimeOutRoutine(0.65), coneHighRetractRoutine),
						new ParallelRoutine(
								thirdPath,
								new SequentialRoutine(
										new TimeOutRoutine(1.7),
										pickupCone1))),
				new ParallelRoutine(resetArm1, new SequentialRoutine(new TimeOutRoutine(0.0), new ParallelRoutine(finalPath, new SequentialRoutine(new TimeOutRoutine(1.7), highConePlaceRoutine1)))),
				coneHighRetractRoutineDrive);
	}
}
