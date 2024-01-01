package frc2023.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import frc2023.behavior.ParallelRoutine;
import frc2023.behavior.RoutineBase;
import frc2023.behavior.SequentialRoutine;
import frc2023.behavior.routines.arm.*;
import frc2023.behavior.routines.drive.*;
import frc2023.behavior.routines.intake.IntakeOuttakeRoutine;
import frc2023.behavior.routines.intake.RunIntakeRoutine;
import frc2023.behavior.routines.superstructure.TimeOutRoutine;
import frc2023.config.constants.PivotConstants;
import frc2023.robot.Robot;
import frc2023.robot.RobotState;
import frc2023.subsystems.Arm;

// Two Game Piece auto starting from lower blue
public class LowerScorePickup extends AutoBase {

	RobotState state;

	public LowerScorePickup(RobotState state) {
		this.state = state;
	}

	@Override
	public RoutineBase getRoutine() {
		var firstPath = new DrivePathPlannerRoutine("BlueLowerToDNew", 1.5, 3, Robot.alliance, true, this.state);
		var secondPath = new DrivePathPlannerRoutine("BlueDToScore", 1.5, 3, Robot.alliance, true, this.state);
		var finalPath = Robot.shouldEngage ?
				new SequentialRoutine(new DrivePathPlannerRoutine("BlueLowerScoreToEngage", 2.5, 3, Robot.alliance, true, this.state), new BalanceChargeStationRoutine()) :
				new DrivePathPlannerRoutine("BlueLowerABackup", 2.5, 3, Robot.alliance, true, this.state);

		Pose2d pose;
		if (Robot.alliance == DriverStation.Alliance.Blue) {
			pose = new Pose2d(1.86, 1.03, Rotation2d.fromDegrees(180));
		} else {
			pose = new Pose2d(1.86, 8 - 1.03, Rotation2d.fromDegrees(180));
		}
		var resetOdometry = new ResetOdometry(pose);

		var cubeHighRoutine = new ParallelRoutine(new ArmMoveToPosRoutineTwo(-80.0, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED), new PivotSetPointRoutine(-14.6, 0.6, 0.5));
		var cubeHighRetractRoutine = new ParallelRoutine(
				new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.1, 0.1),
				new SequentialRoutine(new TimeOutRoutine(0.2),
						new ArmMoveToPosRoutineTwo(-160, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED)));
		var cubeHighPlaceRoutine = new SequentialRoutine(cubeHighRoutine, new TimeOutRoutine(0.3), new IntakeOuttakeRoutine(0.3));

		var pickupCone = new ParallelRoutine(
				new ArmMoveToPosRoutineTwo(-139, 0.5, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED),
				new PivotSetPointRoutine(17.5, 0.5, 0.5),
				new RunIntakeRoutine());

		var resetArm = new ParallelRoutine(new ArmMoveToPosRoutineTwo(-160, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED), new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.1, 0.1));
		var resetArmTwo = new ParallelRoutine(new ArmMoveToPosRoutineTwo(-160, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED), new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.1, 0.1));

		var coneHighRoutine = new ParallelRoutine(new ArmMoveSecondStageRoutine(-29, 0.6), new SequentialRoutine(new TimeOutRoutine(0.4), new ArmMoveFirstStageRoutine(0.2, Arm.FirstStageState.LOWER, Arm.FirstStageState.LOWER)), new PivotSetPointRoutine(-79, 0.3, 0.1));
		var coneHighRetractRoutine = new ParallelRoutine(
				new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.3, 0.3),
				new SequentialRoutine(
						new TimeOutRoutine(0.3),
						new ParallelRoutine(
								new ArmMoveSecondStageRoutine(-160, 0.5),
								new SequentialRoutine(
										new TimeOutRoutine(0.54),
										new ArmMoveFirstStageRoutine(0.1, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED)))));
		var highConePlaceRoutine = new SequentialRoutine(coneHighRoutine, new TimeOutRoutine(0.3), new IntakeOuttakeRoutine(0.3));

		return new SequentialRoutine(
				new ParallelRoutine(new ArmReZeroRoutine(0.1), new PivotReZeroRoutine(0.1), resetOdometry),
				new RunIntakeRoutine(),
				cubeHighPlaceRoutine,
				new ParallelRoutine(
						new SequentialRoutine(new TimeOutRoutine(2), pickupCone),
						firstPath),
				new ParallelRoutine(secondPath, resetArmTwo, new SequentialRoutine(new TimeOutRoutine(0.7), coneHighRoutine)),
				new TimeOutRoutine(0.3), new IntakeOuttakeRoutine(0.3),
				new ParallelRoutine(finalPath, new SequentialRoutine(new TimeOutRoutine(0.8), coneHighRetractRoutine))
		//highConePlaceRoutine

//                new ParallelRoutine(secondPath, resetArmTwo, new SequentialRoutine(new TimeOutRoutine(1.0), coneHighRoutine)),
//                new TimeOutRoutine(0.3), new IntakeOuttakeRoutine(0.3),
//                new ParallelRoutine(coneHighRetractRoutine, new SequentialRoutine(new TimeOutRoutine(0.3), finalPath))

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
