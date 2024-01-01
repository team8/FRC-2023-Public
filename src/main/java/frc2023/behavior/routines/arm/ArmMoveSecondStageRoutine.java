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
public class ArmMoveSecondStageRoutine extends TimeoutRoutineBase {

	double endPos;
	ArmConfig mConfig = Configs.get(ArmConfig.class);
	TrapezoidProfile mProfile;
	private double direction;
	double finalSecondStageAngle;

	public ArmMoveSecondStageRoutine(double endAngle, double time) {
		endPos = endAngle;
		timeout = time;
	}

	@Override
	public void start(Commands commands, @ReadOnly RobotState state) {
		// Required to start the timeout timer
		System.out.println("brutha");
		mTimer.start();
		commands.wantedArmSecondStageState = Arm.SecondStageState.POSITION;
		super.start(commands, state);
		commands.wantedArmPosition = endPos;
	}

	@Override
	public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
	}

	@Override
	public void stop(Commands commands, @ReadOnly RobotState state) {
	}

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return mTimer.get() > timeout;
	}

	private double getMaxVelocityFromConstraints(double distance, double acceleration, double time) {
		double a = -time + Math.sqrt(Math.pow(time, 2) - 4 * (distance / acceleration));
		System.out.println("dist: " + distance + " equation: " + (Math.pow(time, 2) - 4 * (distance / acceleration)));
		System.out.println("a " + a);
		double b = (-2 / acceleration);
		System.out.println("b " + b);
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
