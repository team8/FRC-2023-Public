package frc2023.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;

import frc2023.config.constants.PivotConstants;
import frc2023.config.constants.PortConstants;
import frc2023.config.subsystem.PivotConfig;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.util.LiveGraph;
import frc2023.util.config.Configs;
import frc2023.util.control.ControllerOutput;
import frc2023.util.control.Spark;

public class Pivot extends SubsystemBase {

	private AnalogInput pivotPotentiometer = new AnalogInput(PortConstants.pivotPotentiometerID);
	private Spark pivotSpark = new Spark(PortConstants.pivotMotorID, "Pivot Motor");
	private static Pivot INSTANCE = new Pivot();
	private ControllerOutput mPivotOutput = new ControllerOutput();

	private PivotConfig mPivotConfig = Configs.get(PivotConfig.class);

	private Timer mZeroTimer = new Timer();

	public enum State {
		RE_ZERO, STOW, NEUTRAL, PARALLEL, MANUAL, SET_POINT, RE_ZERO_AUTO
	}

	@Override
	public void update(Commands commands, RobotState state) {
		LiveGraph.add("Pivot output current", pivotSpark.getOutputCurrent());

		switch (commands.wantedPivotState) {
			case STOW:
				mPivotOutput.setTargetPositionProfiled(PivotConstants.pivotStowAngle, mPivotConfig.arbitraryPercentOutput, mPivotConfig.targetingGains);
				break;
			case NEUTRAL:
				mPivotOutput.setIdle();
				break;
			case MANUAL:
				mPivotOutput.setPercentOutput(commands.wantedPivotPercentOutput);
				break;
			case SET_POINT:
				mPivotOutput.setTargetPositionProfiled(commands.wantedPivotAngle, mPivotConfig.arbitraryPercentOutput, mPivotConfig.targetingGains);
				break;
			case RE_ZERO:
				/*
				//0 at straight along, negative when downwards + when upwards
				double potentiometerDiff = state.pivotCurrentPotentiometerPosition - PivotConstants.pivotPotentiometerZero;
				double angle = ((-potentiometerDiff) / PivotConstants.pivotPotentiometerTicksInFullRotation) * 360.0;
				pivotSpark.getEncoder().setPosition(angle);
				
				*/
				LiveGraph.add("Pivot/ZeroTimer", mZeroTimer.get());
				if (mZeroTimer.get() == 0) {
					mZeroTimer.start();
					mPivotOutput.setPercentOutput(0.4);
					commands.wantedPivotState = State.RE_ZERO;
				} else {
					if (mZeroTimer.hasElapsed(1)) {
						pivotSpark.getEncoder().setPosition(150);
						commands.wantedPivotState = State.STOW;
						mZeroTimer.stop();
						mZeroTimer.reset();
					} else {
						mPivotOutput.setPercentOutput(0.4);
						commands.wantedPivotState = State.RE_ZERO;
					}
				}
				break;
			case RE_ZERO_AUTO:
				/*
				//0 at straight along, negative when downwards + when upwards
				double potentiometerDiff = state.pivotCurrentPotentiometerPosition - PivotConstants.pivotPotentiometerZero;
				double angle = ((-potentiometerDiff) / PivotConstants.pivotPotentiometerTicksInFullRotation) * 360.0;
				pivotSpark.getEncoder().setPosition(angle);
				
				*/
				LiveGraph.add("Pivot/ZeroTimer", mZeroTimer.get());
				if (mZeroTimer.get() == 0) {
					mZeroTimer.start();
					mPivotOutput.setPercentOutput(0.4);
					commands.wantedPivotState = State.RE_ZERO;
				} else {
					if (mZeroTimer.hasElapsed(0.1)) {
						pivotSpark.getEncoder().setPosition(150);
						commands.wantedPivotState = State.STOW;
						mZeroTimer.stop();
						mZeroTimer.reset();
					} else {
						mPivotOutput.setPercentOutput(0.4);
						commands.wantedPivotState = State.RE_ZERO;
					}
				}
				break;
			case PARALLEL:
				double angleGround = state.armFirstStageAngleDegrees + state.armSecondStageAngleDegrees;
				LiveGraph.add("angle TO GROUND PIVOT", angleGround);
				mPivotOutput.setTargetPositionProfiled(-angleGround, mPivotConfig.arbitraryPercentOutput, mPivotConfig.targetingGains);
				break;
		}
		LiveGraph.add("pivotPotDiff", ((state.pivotCurrentPotentiometerPosition - PivotConstants.pivotPotentiometerZero) / PivotConstants.pivotPotentiometerTicksInFullRotation) * 360.0);
	}

	@Override
	public void writeHardware(RobotState state) {

		pivotSpark.setOutput(mPivotOutput);
	}

	@Override
	public void readHardware(RobotState state) {
		state.pivotCurrentPotentiometerPosition = pivotPotentiometer.getValue() - 2048;
		LiveGraph.add("pivot pot pos", state.pivotCurrentPotentiometerPosition);
		state.pivotCurrentAngleDegrees = pivotSpark.getEncoder().getPosition();
		LiveGraph.add("pivot ANGLE", state.pivotCurrentAngleDegrees);
		LiveGraph.add("pivot gear", pivotSpark.getEncoder().getPositionConversionFactor());
	}

	@Override
	public void configureHardware() {
		pivotSpark.restoreFactoryDefaults();
		pivotSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);
		pivotSpark.setInverted(false);
		pivotSpark.getEncoder().setVelocityConversionFactor(PivotConstants.pivotVelocityConversionFactor);
		pivotSpark.getEncoder().setPositionConversionFactor(PivotConstants.pivotPositionConversionFactor);
	}

	public static Pivot getInstance() {
		return INSTANCE;
	}
}
