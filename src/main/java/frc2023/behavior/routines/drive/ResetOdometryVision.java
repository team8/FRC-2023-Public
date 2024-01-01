package frc2023.behavior.routines.drive;

import java.util.Set;

import frc2023.behavior.OneUpdateRoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.subsystems.SubsystemBase;

public class ResetOdometryVision extends OneUpdateRoutineBase {

	@Override
	protected void updateOnce(Commands commands, RobotState state) {
		commands.resetOdometry(state.robotPoseMeters);
	}

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return Set.of();
	}
}
