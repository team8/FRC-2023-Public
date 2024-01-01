package frc2023.behavior.routines.drive;

import edu.wpi.first.math.geometry.Translation2d;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;

public class AutoDriveRoutine extends TimeoutRoutineBase {

	private double mScaling;

	public AutoDriveRoutine(double time, double scaling) {
		timeout = time;
		mScaling = scaling;
	}

	@Override
	public void start(Commands commands, @ReadOnly RobotState state) {
		mTimer.reset();
		mTimer.start();
	}

	@Override
	public void stop(Commands commands, @ReadOnly RobotState state) {
		state.routineWantedOffset = new Translation2d();
	}

	@Override
	public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
		commands.setDriveTeleop(mScaling / timeout * (timeout - mTimer.get()), 0, 0, false, true);
	}

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}
}
