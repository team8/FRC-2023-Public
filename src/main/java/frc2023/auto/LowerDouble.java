
package frc2023.auto;

import frc2023.behavior.ParallelRoutine;
import frc2023.behavior.RoutineBase;
import frc2023.behavior.SequentialRoutine;
import frc2023.behavior.routines.arm.ArmMoveToPosRoutineTwo;
import frc2023.behavior.routines.arm.ArmReZeroRoutine;
import frc2023.behavior.routines.arm.PivotReZeroRoutineAuto;
import frc2023.behavior.routines.arm.PivotSetPointRoutine;
import frc2023.behavior.routines.drive.BalanceChargeStationRoutine;
import frc2023.behavior.routines.drive.DrivePathPlannerRoutine;
import frc2023.behavior.routines.drive.ResetOdometryVision;
import frc2023.behavior.routines.intake.IntakeOuttakeRoutine;
import frc2023.behavior.routines.intake.RunIntakeRoutine;
import frc2023.behavior.routines.superstructure.TimeOutRoutine;
import frc2023.config.constants.PivotConstants;
import frc2023.robot.Robot;
import frc2023.robot.RobotState;
import frc2023.subsystems.Arm;

// Two Game Piece auto starting from lower blue
public class LowerDouble extends AutoBase {

	RobotState state;

	public LowerDouble(RobotState state) {
		this.state = state;
	}

	@Override
	public RoutineBase getRoutine() {
//        var firstPath =  new SequentialRoutine(
//                new DrivePathPlannerRoutine("BlueUpperToAFaceTagsPart1", 2, 3, Robot.alliance, true, this.state),
//                new DrivePathPlannerRoutine("BlueUpperToAFaceTagsPart2Better", 1, 3, Robot.alliance, false, this.state)
//        );
		var firstPath = new DrivePathPlannerRoutine("BlueLowerToDNew", 1.2, 3, Robot.alliance, false, this.state);
		var secondPath = new SequentialRoutine(
				new DrivePathPlannerRoutine("BlueDToScorePart1", 2.5, 3, Robot.alliance, false, this.state),
				new DrivePathPlannerRoutine("BlueDToScorePart2", 1, 3, Robot.alliance, true, this.state));
		var finalPath = Robot.shouldEngage ?
				new SequentialRoutine(new DrivePathPlannerRoutine("BlueUpperScoreToEngage", 3.5, 3, Robot.alliance, true, this.state), new BalanceChargeStationRoutine()) :
				new DrivePathPlannerRoutine("BlueUpperABackup", 3.5, 3, Robot.alliance, true, this.state);

		var cubeHighRoutine = new ParallelRoutine(new ArmMoveToPosRoutineTwo(-80.0, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED), new PivotSetPointRoutine(-14.6, 0.6, 0.5));
		var cubeHighPlaceRoutine = new SequentialRoutine(cubeHighRoutine, new TimeOutRoutine(0.3), new IntakeOuttakeRoutine(0.3));

		var pickupCone = new ParallelRoutine(
				new ArmMoveToPosRoutineTwo(-139, 0.5, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED),
				new PivotSetPointRoutine(17.5, 0.5, 0.5),
				new RunIntakeRoutine());

		var resetArmTwo = new ParallelRoutine(new SequentialRoutine(new TimeOutRoutine(0.2), new ArmMoveToPosRoutineTwo(-160, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED)), new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.1, 0.1));

		var coneHighRoutine = new ParallelRoutine(new ArmMoveToPosRoutineTwo(-73.0, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED), new PivotSetPointRoutine(-48, 0.6, 0.5));
		var coneHighRetractRoutine = new ParallelRoutine(
				new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.1, 0.1),
				new SequentialRoutine(
						new TimeOutRoutine(0.5),
						new ParallelRoutine(
								new ArmMoveToPosRoutineTwo(-160, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED))));
		var coneHighRetractRoutineOne = new ParallelRoutine(
				new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.1, 0.1),
				new SequentialRoutine(
						new TimeOutRoutine(0.5),
						new ParallelRoutine(
								new ArmMoveToPosRoutineTwo(-160, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED))));

		return new SequentialRoutine(
//                new ParallelRoutine(new ArmReZeroRoutine(0.1), new PivotReZeroRoutine(0.1), resetOdometry),
				new RunIntakeRoutine(),
				new ParallelRoutine(new ArmReZeroRoutine(0.1), new PivotReZeroRoutineAuto(0.1), new ResetOdometryVision()),
				cubeHighPlaceRoutine,
				new ParallelRoutine(
						firstPath,
						new SequentialRoutine(new TimeOutRoutine(1.5), pickupCone)),
				new ParallelRoutine(secondPath, resetArmTwo, new SequentialRoutine(new TimeOutRoutine(2), coneHighRoutine)),
				new TimeOutRoutine(0.3), new IntakeOuttakeRoutine(0.3),
				new ParallelRoutine(finalPath, new SequentialRoutine(new TimeOutRoutine(0.8), coneHighRetractRoutine))
		//new BalanceChargeStationRoutine()
		/*
		//new RunIntakeRoutine(),
		new ParallelRoutine(resetArm, new SequentialRoutine(new TimeOutRoutine(0.2), new ParallelRoutine(secondPath, new SequentialRoutine(new TimeOutRoutine(1.7), highConePlaceRoutine)))),
		new ParallelRoutine(new SequentialRoutine(new TimeOutRoutine(0.65), coneHighRetractRoutine), finalPath),
		new BalanceChargeStationRoutine()
		 */
		);
		//return new SequentialRoutine(new ResetOdometry(new Pose2d(1.8, 1.06, Rotation2d.fromDegrees(180))), firstPath, secondPath, finalPath);
	}
}
