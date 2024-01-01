package frc2023.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc2023.behavior.RoutineBase;
import frc2023.behavior.routines.drive.ResetOdometry;

public class BaseOdomAuto extends AutoBase {

	@Override
	public RoutineBase getRoutine() {
		return new ResetOdometry(new Pose2d(0, 0, new Rotation2d()));
	}
}
