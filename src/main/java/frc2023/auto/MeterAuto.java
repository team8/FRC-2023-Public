package frc2023.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc2023.behavior.RoutineBase;
import frc2023.behavior.SequentialRoutine;
import frc2023.behavior.routines.drive.DrivePathPlannerRoutine;
import frc2023.behavior.routines.drive.ResetOdometry;
import frc2023.robot.Robot;
import frc2023.robot.RobotState;

public class MeterAuto extends AutoBase {

	RobotState state;

	public MeterAuto(RobotState state) {
		this.state = state;
	}

	@Override
	public RoutineBase getRoutine() {
		var firstPath = new DrivePathPlannerRoutine("UpperMove", 1.0, 2.0, Robot.alliance, false, this.state);
		return new SequentialRoutine(
				new ResetOdometry(new Pose2d(1.79, 4.42, Rotation2d.fromDegrees(180))),
				firstPath);
	}
}
