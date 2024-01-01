package frc2023.util.service;

import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.util.Util;

public interface RobotService {

	default void start() {
	}

	default void update(@ReadOnly RobotState state, @ReadOnly Commands commands) {
	}

	default String getConfigName() {
		return Util.classToJsonName(getClass());
	}
}
