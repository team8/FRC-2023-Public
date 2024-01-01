package frc2023.behavior.routines.drive;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;

import frc2023.behavior.OneUpdateRoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.subsystems.SubsystemBase;

public class ResetOdometry extends OneUpdateRoutineBase {

	private Pose2d newPose;

	public ResetOdometry(Pose2d newPose) {
		this.newPose = newPose;
	}

	@Override
	protected void updateOnce(Commands commands, RobotState state) {
		commands.resetOdometry(newPose);
	}

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return Set.of();
	}
}
