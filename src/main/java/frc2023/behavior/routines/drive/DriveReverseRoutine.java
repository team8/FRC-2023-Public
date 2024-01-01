package frc2023.behavior.routines.drive;

import edu.wpi.first.math.geometry.Translation2d;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;

public class DriveReverseRoutine extends TimeoutRoutineBase {

	private double mScaling;

	public DriveReverseRoutine(double time, double scaling) {
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
	//TODO MAKE THIS NOT TAKE AWAY CONTROL THROUGH VEDANTHS TRANSLATION THING
	public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
		double angle = state.swervePoseMeters.getRotation().getDegrees();
		angle += 90.0;
		angle = Math.toRadians(angle);
		state.routineWantedOffset = new Translation2d(mScaling / timeout * (timeout - mTimer.get()) * Math.cos(angle), mScaling / timeout * (timeout - mTimer.get()) * Math.sin(angle));
	}

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}
}
