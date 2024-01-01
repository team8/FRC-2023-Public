package frc2023.util.control;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDriveOutputs {

	public SwerveModuleState[] swerveModuleStates;
	public boolean isOpenLoop;

	public SwerveDriveOutputs() {
		swerveModuleStates = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) {
			swerveModuleStates[i] = new SwerveModuleState();
		}
		isOpenLoop = true;
	}

	public SwerveDriveOutputs(SwerveModuleState[] states, boolean isOpenLoop) {
		swerveModuleStates = states;
		this.isOpenLoop = isOpenLoop;
	}
}
