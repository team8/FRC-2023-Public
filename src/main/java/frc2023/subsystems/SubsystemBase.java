package frc2023.subsystems;

import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.util.Util;

public abstract class SubsystemBase {

	private final String mName;

	protected SubsystemBase() {
		mName = Util.classToJsonName(getClass());
	}

	public abstract void update(@ReadOnly Commands commands, @ReadOnly RobotState state);

	public abstract void writeHardware(@ReadOnly RobotState state);

	public abstract void readHardware(RobotState state);

	public abstract void configureHardware();

	public void logSubsystem(RobotState state) {
	}

	public void simulate(RobotState state, Commands commands) {
	}

	public void simulationInit() {
	}

	@Override
	public String toString() {
		return getName();
	}

	public String getName() {
		return mName;
	}
}
