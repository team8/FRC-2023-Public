package frc2023.behavior.routines.drive;

import static frc2023.config.constants.DriveConstants.autoBalanceAcceptableError;
import static frc2023.config.constants.DriveConstants.autoBalanceTimer;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import frc2023.behavior.RoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.Robot;
import frc2023.robot.RobotState;
import frc2023.subsystems.Drive;
import frc2023.subsystems.SubsystemBase;
import frc2023.util.LiveGraph;

public class BalanceChargeStationRoutine extends RoutineBase {

	private final PIDController pidP = new PIDController(0.013, 0, 0);
	private final PIDController pidR = new PIDController(0.013, 0, 0);
	private final Timer timer = new Timer();
	private boolean balanced = false;

	@Override
	protected void start(Commands commands, RobotState state) {
		pidP.setSetpoint(0);
		pidR.setSetpoint(0);
	}

	@Override
	protected void update(Commands commands, RobotState state) {
		LiveGraph.add("Routines/BalanceChargeStationRoutine/pitch", state.drivePitch.getDegrees());
		LiveGraph.add("Routines/BalanceChargeStationRoutine/roll", state.driveRoll.getDegrees());
		double oP = pidP.calculate(state.drivePitch.getDegrees());
		double oR = pidR.calculate(state.driveRoll.getDegrees());
		boolean larger = Math.abs(oP) > Math.abs(oR);
		double output;
		//TODO eliminate the if statement
		if (Robot.alliance == DriverStation.Alliance.Blue) {
			System.out.println("WE ARE ON BLUE");
			output = larger ? -oP : -oR;
		} else {
			System.out.println("WE ARE ON RED");
			output = larger ? -oP : -oR;
		}
		LiveGraph.add("Routines/BalanceChargeStationRoutine/output", output);
		if (!balanced) commands.setDriveTeleop(output, 0, 0, true, true);
		else commands.setDriveTeleop(0, 0, 0, true, true);
	}

	@Override
	public boolean checkFinished(RobotState state) {
		if (Math.abs(state.drivePitch.getDegrees()) <= autoBalanceAcceptableError && Math.abs(state.driveRoll.getDegrees()) <= autoBalanceAcceptableError) {
			timer.start();
			balanced = true;
			LiveGraph.add("Routines/BalanceChargeStationRoutine/acceptable", true);
		} else {
			timer.reset();
			timer.stop();
			balanced = false;
			LiveGraph.add("Routines/BalanceChargeStationRoutine/acceptable", false);
		}
		LiveGraph.add("Routines/BalanceChargeStationRoutine/on", timer.hasElapsed(autoBalanceTimer));
		return timer.hasElapsed(autoBalanceTimer);
	}

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return Set.of(Drive.class);
	}

}
