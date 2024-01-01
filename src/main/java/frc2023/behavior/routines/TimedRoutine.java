package frc2023.behavior.routines;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.Timer;

import frc2023.behavior.RoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.SubsystemBase;

public abstract class TimedRoutine extends RoutineBase {

	protected final Timer mTimer = new Timer();
	protected double timeout;

	/**
	 * Routine that waits the specified amount of time. Does not require any subsystems.
	 */
	public TimedRoutine(double durationSeconds) {
		timeout = durationSeconds;
	}

	@Override
	public void start(Commands commands, @ReadOnly RobotState state) {
		mTimer.start();
	}

	@Override
	public boolean checkFinished(@ReadOnly RobotState state) {
		return mTimer.hasElapsed(timeout);
	}

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return new HashSet<>();
	}
}
