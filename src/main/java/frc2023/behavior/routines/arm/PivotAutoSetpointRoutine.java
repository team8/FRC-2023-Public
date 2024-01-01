package frc2023.behavior.routines.arm;

import java.util.Set;

import edu.wpi.first.wpilibj.Timer;

import frc2023.behavior.RoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.Pivot;
import frc2023.subsystems.SubsystemBase;

public class PivotAutoSetpointRoutine extends RoutineBase {

	private double mSetpoint;
	private final double ERROR = 0.5;
	private Timer mTimer = new Timer();

	public PivotAutoSetpointRoutine(double setpoint) {
		mSetpoint = setpoint;
		mTimer.reset();

	}

	@Override
	protected void start(Commands commands, @ReadOnly RobotState state) {
		mTimer.start();
		commands.wantedPivotState = Pivot.State.SET_POINT;
		commands.wantedPivotAngle = mSetpoint;
	}

	@Override
	protected void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
	}

	@Override
	public boolean checkFinished(RobotState state) {
		if (Math.abs(mSetpoint - state.pivotCurrentAngleDegrees) <= ERROR || mTimer.get() > 2.5) {
			return true;
		}
		return false;
	}

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return Set.of(Pivot.class);
	}
}
