package frc2023.behavior.routines.arm;

import java.util.Set;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.config.subsystem.ArmConfig;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.Arm;
import frc2023.subsystems.SubsystemBase;
import frc2023.util.config.Configs;

/* TODO: make sure theres only one of these motherfrickers active at a time on this bot */
public class ArmSecondStageTrajectoryRoutine extends TimeoutRoutineBase {

	double endPos;
	ArmConfig mConfig = Configs.get(ArmConfig.class);
	TrapezoidProfile mProfile;
	private double direction;

	public ArmSecondStageTrajectoryRoutine(double endAngle, double time) {
		endPos = endAngle;
		timeout = time;
	}

	@Override
	public void start(Commands commands, @ReadOnly RobotState state) {
		// Required to start the timeout timer
		mTimer.start();
		commands.wantedArmSecondStageState = Arm.SecondStageState.TRAJECTORY;
		super.start(commands, state);
		double velocity = getMaxVelocityFromConstraints(Math.abs(endPos - state.armSecondStageAngleDegrees), mConfig.armSecondStageMaxAcceleration, timeout) * Math.signum(-endPos + state.armSecondStageAngleDegrees);
		if (Double.isNaN(velocity) || Math.abs(velocity) > mConfig.armSecondStageMaxVelocity) {
			velocity = mConfig.armSecondStageMaxVelocity * Math.signum(endPos - state.armSecondStageAngleDegrees);
			timeout = Math.abs(getTimeoutFromConstraints(Math.abs(endPos - state.armSecondStageAngleDegrees), mConfig.armSecondStageMaxAcceleration, velocity));
		}
		direction = Math.signum(endPos - state.armSecondStageAngleDegrees);
		mProfile = new TrapezoidProfile(
				new TrapezoidProfile.Constraints(Math.abs(velocity), mConfig.armSecondStageMaxAcceleration),
				new TrapezoidProfile.State(state.armSecondStageAngleDegrees, state.armSecondStageVelocity),
				new TrapezoidProfile.State(endPos, 0.0));
		TrapezoidProfile.State initState = mProfile.calculate(0.0);
		commands.armSecondStageWantedPositionDegrees = initState.position;
		commands.armSecondStageWantedVelocityDegreesPerSecond = initState.velocity;
	}

	@Override
	public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
		TrapezoidProfile.State currentWantedState = mProfile.calculate(mTimer.get());
		commands.armSecondStageWantedVelocityDegreesPerSecond = -currentWantedState.velocity;
		commands.armSecondStageWantedPositionDegrees = currentWantedState.position;
	}

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}

	private double getMaxVelocityFromConstraints(double distance, double acceleration, double time) {
		double a = -time + Math.sqrt(Math.pow(time, 2) - 4 * (distance / acceleration));
		double b = (-2 / acceleration);
		return a / b;
	}

	public double getTimeout() {
		return timeout;
	}

	private double getTimeoutFromConstraints(double distance, double acceleration, double velocity) {
		double a = (distance - (Math.pow(velocity, 2) / acceleration)) / velocity;
		double b = (2 * velocity) / acceleration;
		return a + b;
	}

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return Set.of(Arm.class);
	}
}
