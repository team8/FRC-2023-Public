package frc2023.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc2023.config.constants.RobotConstants;
import frc2023.robot.RobotState;

public class SimulatedSwerveModule extends SwerveModule {

	private static int modules = 0;

	public SimulatedSwerveModule(SwerveModuleConfig config) {
		super(config);
		this.moduleNumber = modules;
		modules++;
	}

	private SwerveModuleState state = new SwerveModuleState();

	private double position = 0.0;

	@Override
	public SwerveModuleState setDesiredState(RobotState state, SwerveModuleState desiredState, boolean isOpenLoop) {
		super.setDesiredState(state, desiredState, isOpenLoop);
		this.state = desiredState;
		return desiredState;
	}

	@Override
	public SwerveModuleState getState() {
		return state;
	}

	@Override
	public SwerveModulePosition getPosition() {
		position += state.speedMetersPerSecond * RobotConstants.period;
		return new SwerveModulePosition(position, state.angle);
	}

	@Override
	public void writeHardware() {
		super.writeHardware();
	}
}
