package frc2023.robot;

import static frc2023.config.constants.DriveConstants.*;
import static frc2023.config.constants.PortConstants.*;
import static frc2023.util.Util.handleDeadBand;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import frc2023.behavior.ParallelRoutine;
import frc2023.behavior.RoutineBase;
import frc2023.behavior.SequentialRoutine;
import frc2023.behavior.routines.PriorityRoutine;
import frc2023.behavior.routines.arm.*;
import frc2023.behavior.routines.drive.*;
import frc2023.behavior.routines.intake.IntakeOuttakeRoutine;
import frc2023.behavior.routines.intake.RunIntakeRoutine;
import frc2023.behavior.routines.intake.RunIntakeSlowRoutine;
import frc2023.behavior.routines.intake.RunIntakeTimedRoutine;
import frc2023.behavior.routines.superstructure.ScoreRoutine;
import frc2023.behavior.routines.superstructure.TimeOutRoutine;
import frc2023.config.constants.PivotConstants;
import frc2023.subsystems.*;
import frc2023.util.LiveGraph;
import frc2023.util.input.Joystick;
import frc2023.util.input.XboxController;

/**
 * Used to produce {@link Commands}'s from human input. Should only be used in robot package.
 */
public class OperatorInterface {

	public static final double DEAD_BAND = 0.075;
	public static final double DEAD_BAND_DRIVER = 0.05;
	double offsetX;
	double offsetY;
	boolean coneWanted = false;
	boolean highCone = false;
	boolean midCone = false;
	Pose2d targetPose = new Pose2d();
	String wantedHeight = "high";

	HashMap<Integer, Pose2d> scoringPositions = new HashMap<>();
	int wantedIdx;
	private VisionDriveRoutine driveRoutine;
	private RoutineBase visionArmDown;

	private final XboxController operatorXboxController = new XboxController(operatorXbox);
	private final XboxController driverXboxController = new XboxController(driverXbox);
	private final Joystick operatorKeyboard = new Joystick(2);
	private boolean fieldRelative = true;
	ParallelRoutine armUpRoutine;
	Timer intakeStopWheelsTimer = new Timer();
	/*
	 * 0: Rezero
	 * 1: Scoring
	 * 2: Retracting After Scoring
	 * 3: Double Substation
	 * 4: Retract Double Substation
	 * 5: Pickup Cone
	 * 6: Pickup Cube
	 * 7: Return Cube/Cone
	 * 8: Single Substation
	 */
	PriorityQueue<PriorityRoutine> wantedRoutines = new PriorityQueue<>();
	RoutineBase lastWanted = null;

	boolean doVision = true;

	public void setConeWanted() {
		coneWanted = true;
	}

	public OperatorInterface() {
		/*
		 * 0: Single Substation
		 * 1-9: Scoring Grid
		 * 10: Double Substation
		 */
		if (Robot.alliance == DriverStation.Alliance.Red) {
			scoringPositions.put(0, new Pose2d(2.95, 7.5, new Rotation2d(Math.PI / 2)));
			scoringPositions.put(1, new Pose2d(14.69, 0.64, new Rotation2d()));
			scoringPositions.put(2, new Pose2d(14.69, 1.06, new Rotation2d()));
			scoringPositions.put(3, new Pose2d(14.69, 1.69, new Rotation2d(0.02)));
			scoringPositions.put(4, new Pose2d(14.69, 2.19, new Rotation2d()));
			scoringPositions.put(5, new Pose2d(14.69, 2.71, new Rotation2d()));
			scoringPositions.put(6, new Pose2d(14.69, 3.27, new Rotation2d(0.03)));
			scoringPositions.put(7, new Pose2d(14.69, 3.84, new Rotation2d(0.04)));
			scoringPositions.put(8, new Pose2d(14.69, 4.36, new Rotation2d()));
			scoringPositions.put(9, new Pose2d(14.69, 4.89, new Rotation2d(0.03)));
			scoringPositions.put(10, new Pose2d(0, 0, new Rotation2d()));
		} else if (Robot.alliance == DriverStation.Alliance.Blue) {
			scoringPositions.put(0, new Pose2d(13.59, 7.5, new Rotation2d(Math.PI / 2)));
			scoringPositions.put(1, new Pose2d(1.83, 0.64, new Rotation2d(Math.PI + 0.05)));
			scoringPositions.put(2, new Pose2d(1.83, 1.04, new Rotation2d(Math.PI)));
			scoringPositions.put(3, new Pose2d(1.83, 1.69, new Rotation2d(Math.PI)));
			scoringPositions.put(4, new Pose2d(1.83, 2.22, new Rotation2d(Math.PI)));
			scoringPositions.put(5, new Pose2d(1.83, 2.71, new Rotation2d(Math.PI)));
			scoringPositions.put(6, new Pose2d(1.83, 3.26, new Rotation2d(Math.PI)));
			scoringPositions.put(7, new Pose2d(1.83, 3.84, new Rotation2d(Math.PI)));
			scoringPositions.put(8, new Pose2d(1.83, 4.36, new Rotation2d(Math.PI)));
			scoringPositions.put(9, new Pose2d(1.83, 4.91, new Rotation2d(Math.PI - 0.03)));
			scoringPositions.put(10, new Pose2d(15.42, 7.33, new Rotation2d()));
		}
	}

	/**
	 * Modifies commands based on operator input devices.
	 */
	void updateCommands(Commands commands, @ReadOnly RobotState state) {
		if (wantedRoutines.size() != 0) {
			if (commands.routinesWanted.contains(lastWanted)) {

			} else {
				commands.addWantedRoutine(wantedRoutines.element().getRoutine());
				lastWanted = wantedRoutines.element().getRoutine();
				wantedRoutines.clear();
			}
		}
		updateDriveCommands(commands, state);
		updateArmCommands(commands, state);
		updateIntakeCommands(commands, state);
	}

	private void updateDriveCommands(Commands commands, RobotState state) {
		commands.shouldClearCurrentRoutines = false;
		double multiplier;
		if (driverXboxController.getLeftTrigger()) {
			multiplier = 0.25;
		} else {
			multiplier = 1;
		}

		if (!state.drivePathFollowing) {
			commands.setDriveTeleop(
					-multiplier * handleDeadBand(driverXboxController.getLeftY(), DEAD_BAND_DRIVER, true) + state.routineWantedOffset.getY(),
					-multiplier * handleDeadBand(driverXboxController.getLeftX(), DEAD_BAND_DRIVER, true) + state.routineWantedOffset.getX(),
					multiplier * -handleDeadBand(driverXboxController.getRightX(), DEAD_BAND_DRIVER, true),
					true, false);
		}
		int joystickPov = (int) (operatorKeyboard.getY() * 127 + 0.5);
		wantedIdx = -1;
		if (joystickPov >= 15 && joystickPov <= 25) {
			//high
			wantedHeight = "high";
			wantedIdx = joystickPov - 15;
		} else if (joystickPov >= 31 && joystickPov <= 39) {
			//mid
			wantedHeight = "mid";
			wantedIdx = joystickPov - 30;
		} else if (joystickPov >= 46 && joystickPov <= 54) {
			//low
			wantedHeight = "low";
			wantedIdx = joystickPov - 45;
		}

		if (Robot.isSimulation()) {
			wantedIdx = 5;
			wantedHeight = "high";
		}

		if (wantedIdx != 0 && wantedIdx != 10 && Robot.alliance == DriverStation.Alliance.Blue) {
			wantedIdx = 10 - wantedIdx;
		}
		if (wantedIdx == 2 || wantedIdx == 5 || wantedIdx == 8) {
			coneWanted = false;
		} else {
			coneWanted = true;
		}

		if (driverXboxController.getRightBumperPressed() || driverXboxController.getRightTriggerPressed()) {
			if (!state.drivePathFollowing) {

				LiveGraph.add("Robot/wanted", wantedIdx);
				if (wantedIdx < 0 || wantedIdx > 10) {
					return;
				}

				targetPose = scoringPositions.get(wantedIdx);
				if (wantedHeight.equals("mid") || wantedHeight.equals("low")) {
					targetPose = targetPose.transformBy(new Transform2d(new Translation2d(), new Rotation2d(0.05)));
				}
				ArrayList<Pose2d> waypoints = new ArrayList<>(List.of(state.robotPoseMeters, targetPose));
				boolean substation = (wantedIdx == 0 || wantedIdx == 10);

				driveRoutine = new VisionDriveRoutine(commands, state, waypoints, 3.0, 3.0,
						wantedIdx, substation, wantedHeight);
				var armUpRoutine = new SequentialRoutine(new TimeOutRoutine(driveRoutine.getInitialLineupTime()), getArmRoutine(true));
				visionArmDown = getArmRoutine(false);
				var finalVisionRoutine = new SequentialRoutine(new ParallelRoutine(driveRoutine, armUpRoutine), new ParallelRoutine(new SequentialRoutine(new TimeOutRoutine(0.5), new IntakeOuttakeRoutine(1.0)), new SequentialRoutine(new TimeOutRoutine(0.75), visionArmDown)));

				RoutineBase doubleSub = (new ParallelRoutine(
						new ArmMoveToPosRoutineTwo(-66.95, 0.5, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED),
						new PivotSetPointRoutine(-57.64, 0.5, 0.4),
						new RunIntakeRoutine()));

				if (!substation) {
					wantedRoutines.add(new PriorityRoutine(finalVisionRoutine, 0));
				} else {
					if (wantedIdx == 0) {
						wantedRoutines.add(new PriorityRoutine(driveRoutine, 0));
					} else if (wantedIdx == 10) {
						wantedRoutines.add(new PriorityRoutine(new ParallelRoutine(doubleSub, driveRoutine), 0));
					}

				}
			}

			state.drivePathFollowing = true;
		}
		if (driverXboxController.getRightBumperReleased()) {
			state.drivePathFollowing = false;
			commands.shouldClearCurrentRoutines = true;
			commands.addWantedRoutine(new ResetOdometry(new Pose2d(state.swervePoseMeters.getX(), state.swervePoseMeters.getY(), state.swervePoseMeters.getRotation())));
		}
		if (driverXboxController.getAButtonPressed()) {
			commands.addWantedRoutine(new ResetOdometry(new Pose2d()));
			if (state.simulated) SimulatedRobot.odometry = new Pose2d();
		}
		if (driverXboxController.getYButtonPressed()) {
			commands.addWantedRoutine(new ResetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(180.0))));
		}
		if (driverXboxController.getBButtonPressed()) {
			commands.addWantedRoutine(new ResetOdometry(state.robotPoseMeters));
		}
	}

	private void updateArmCommands(Commands commands, RobotState state) {
		LiveGraph.add("robot/FIRST BUTTON PRESSED", operatorKeyboard.getRawButtonPressed(1));
		// Lighting for Cones
		if (coneWanted && !state.drivePathFollowing) {
			commands.lightingWantedState = Lighting.State.CONE;
		} else if (!coneWanted && !state.drivePathFollowing) {
			commands.lightingWantedState = Lighting.State.CUBE;
		}

		if (driverXboxController.getXButtonPressed()) {
			var wanted = new ParallelRoutine(new ArmReZeroRoutine(0.2), new PivotReZeroRoutine(0.2));
			wantedRoutines.add(new PriorityRoutine(wanted, 0));
		}

		// manual arm controlled by up/down arrows on keyboard
		if ((operatorKeyboard.getX() * 127) != 0) {
			commands.armWantedPercentOutput = 0.2 * (operatorKeyboard.getX() / Math.abs(operatorKeyboard.getX()));
			commands.wantedArmSecondStageState = Arm.SecondStageState.PERCENT_OUTPUT;
		} else if (commands.wantedArmSecondStageState == Arm.SecondStageState.PERCENT_OUTPUT) {
			commands.wantedArmSecondStageState = Arm.SecondStageState.STATIONARY;
			commands.wantedPivotPercentOutput = 0.0;
		}

		if (commands.shouldMoveArmVision) {
			commands.shouldMoveArmVision = false;
		}

		if (operatorKeyboard.getRawButton(1)) {
			//reset arm
			var wanted = new ParallelRoutine(
					new ArmMoveToPosRoutineTwo(-150, 0.5, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED),
					new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.5, 0.5));
			// reset has the highest priority
			wantedRoutines.add(new PriorityRoutine(wanted, 0));
		}

		//manual pivot controlled with left/right arrows on keyboard
		if (operatorKeyboard.getZ() * 127 != 0) {
			commands.wantedPivotPercentOutput = -0.25 * (operatorKeyboard.getZ() / Math.abs(operatorKeyboard.getZ()));
			commands.wantedPivotState = Pivot.State.MANUAL;
		} else if (commands.wantedPivotState != Pivot.State.RE_ZERO && commands.wantedPivotState != Pivot.State.STOW && commands.wantedPivotState != Pivot.State.SET_POINT) {
			commands.wantedPivotState = Pivot.State.SET_POINT;
			commands.wantedPivotAngle = state.pivotCurrentAngleDegrees;
		}

		if (driverXboxController.getLeftBumperPressed()) {
			commands.addWantedRoutine(getArmRoutine(true));
		} else if (driverXboxController.getLeftBumperReleased()) {
			commands.addWantedRoutine(getArmRoutine(false));
		}
	}

	private void updateIntakeCommands(Commands commands, RobotState state) {
		//double substation rotations

		Translation2d frontLeftLocation = new Translation2d((wheelBase / 2.0), trackWidth / 2.0);
		Translation2d frontRightLocation = new Translation2d((wheelBase / 2.0), -trackWidth / 2.0);
		Translation2d backLeftLocation = new Translation2d((-wheelBase / 2.0), trackWidth / 2.0);
		Translation2d backRightLocation = new Translation2d((-wheelBase / 2.0), -trackWidth / 2.0);

		swerveKinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

		// double substation
		if (operatorKeyboard.getRawButtonPressed(7)) {
			//cube from double substation
			var wantedRoutine = (new ParallelRoutine(
					new ArmMoveToPosRoutineTwo(-66.95, 0.5, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED),
					new PivotSetPointRoutine(-57.64, 0.5, 0.4),
					new RunIntakeRoutine()));
			wantedRoutines.add(new PriorityRoutine(wantedRoutine, 3));
			intakeStopWheelsTimer.reset();
			intakeStopWheelsTimer.stop();
		} else if (operatorKeyboard.getRawButtonReleased(7)) {
			var wanted = new ParallelRoutine(
					new SequentialRoutine(
							new ScoreRoutine(0.4),
							new ArmMoveToPosRoutineTwo(-160.0, 0.5, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED)),
					new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.3, 0.3),
					new RunIntakeSlowRoutine(),
					new DriveReverseRoutine(0.6, 0.5));
			wantedRoutines.add(new PriorityRoutine(wanted, 4));
		}

		//cone intake
		if (operatorKeyboard.getRawButtonPressed(2)) {
			var wanted = new ParallelRoutine(
					new ArmMoveToPosRoutineTwo(-139, 0.5, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED),
					new PivotSetPointRoutine(2.5, 0.5, 0.5),
					new RunIntakeRoutine());
			wantedRoutines.add(new PriorityRoutine(wanted, 5));
		}

		//cube intake
		if (operatorKeyboard.getRawButtonPressed(6)) {
			var wanted = new ParallelRoutine(
					new ArmMoveToPosRoutineTwo(-120.5, 0.5, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED),
					new PivotSetPointRoutine(-11, 0.3, 0.3),
					new RunIntakeRoutine());
			wantedRoutines.add(new PriorityRoutine(wanted, 6));
		}

		if (operatorKeyboard.getRawButtonReleased(2) || operatorKeyboard.getRawButtonReleased(6) || operatorKeyboard.getRawButtonReleased(3)) {
			var wanted = new ParallelRoutine(
					new RunIntakeTimedRoutine(0.6),
					new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.3, 0.3),
					new SequentialRoutine(new TimeOutRoutine(0.2), new ArmMoveToPosRoutineTwo(-150, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED)));
			wantedRoutines.add(new PriorityRoutine(wanted, 7));
		}

		//substation pos
		if (operatorKeyboard.getRawButtonPressed(3)) {
			var wantedRoutine = new ParallelRoutine(
					new PivotSetPointRoutine(111, 0.1, 0.1),
					new RunIntakeRoutine());
			wantedRoutines.add(new PriorityRoutine(wantedRoutine, 8));
		}

		//operating intake in substation
		LiveGraph.add("robot/button 14", operatorKeyboard.getRawButton(14));
		if (operatorKeyboard.getRawButtonPressed(14)) {
			commands.intakeRollerWantedState = Intake.RollerState.INTAKE;
		} else if (operatorKeyboard.getRawButtonReleased(14)) {
			commands.intakeRollerWantedState = Intake.RollerState.INTAKE_SLOW;
		}

		if (operatorKeyboard.getRawButtonPressed(15)) {
			commands.intakeRollerWantedState = Intake.RollerState.OUTTAKE;
		} else if (operatorKeyboard.getRawButtonReleased(15)) {
			commands.intakeRollerWantedState = Intake.RollerState.IDLE;
		}
	}

	public void resetPeriodic(Commands commands) {
	}

	public void reset(Commands commands) {
		commands.routinesWanted.clear();
	}

	// placing means you are either placing or pulling back
	public RoutineBase getArmRoutine(boolean placing) {

		if (placing) {
			maxSpeed = 2.0;
			var coneHighRoutine = new ParallelRoutine(new ArmMoveToPosRoutineTwo(-73.0, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED), new PivotAutoSetpointRoutine(-39));

			var cubeHighRoutine = new ParallelRoutine(new ArmMoveToPosRoutineTwo(-80.0, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED), new PivotAutoSetpointRoutine(-34));
			var cubeMidRoutine = new ParallelRoutine(new ArmMoveToPosRoutineTwo(-145, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED), new PivotAutoSetpointRoutine(111));
			var coneMidRoutine = new ParallelRoutine(
					new ArmMoveToPosRoutineTwo(-67.6, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED),
					new SequentialRoutine(new TimeOutRoutine(0.1), new PivotAutoSetpointRoutine(-104.7)));
			var lowRoutine = new ParallelRoutine(new ParallelRoutine(new ArmMoveToPosRoutineTwo(-160.0, 0.5, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED), new PivotSetPointRoutine(90.0, 0.6, 0.6)));
			highCone = false;
			midCone = false;
			if (coneWanted) {
				if (wantedHeight.equals("high")) {
					armUpRoutine = coneHighRoutine;
					highCone = true;
				} else if (wantedHeight.equals("mid")) {
					armUpRoutine = coneMidRoutine;
					midCone = true;
				} else {
					armUpRoutine = lowRoutine;
				}
			} else {
				if (wantedHeight.equals("high")) {
					armUpRoutine = coneHighRoutine;
				} else if (wantedHeight.equals("mid")) {
					armUpRoutine = cubeMidRoutine;
				} else {
					armUpRoutine = lowRoutine;
				}
			}
			return armUpRoutine;

		} else {
			maxSpeed = 4.0;
			RoutineBase armDownRoutine;
			if (wantedHeight.equals("high") || highCone) {
				if (highCone) {
					armDownRoutine = new ParallelRoutine(
							new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.1, 0.1),
							new SequentialRoutine(
									new TimeOutRoutine(1),
									new ParallelRoutine(
											new ArmMoveToPosRoutineTwo(-150, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED))));
				} else {
					armDownRoutine = new ParallelRoutine(
							new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.1, 0.1),
							new SequentialRoutine(
									new TimeOutRoutine(1),
									new ParallelRoutine(
											new ArmMoveToPosRoutineTwo(-150, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED))));
				}
			} else {
				if (midCone) {
					armDownRoutine = new ParallelRoutine(
							new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.1, 0.1),
							new SequentialRoutine(
									new TimeOutRoutine(0.6),
									new ParallelRoutine(
											new ArmMoveToPosRoutineTwo(-150, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED))));
				} else {
					armDownRoutine = new ParallelRoutine(
							new PivotSetPointRoutine(PivotConstants.pivotStowAngle, 0.1, 0.1),
							new SequentialRoutine(
									new TimeOutRoutine(0.2),
									new ArmMoveToPosRoutineTwo(-150, 0.6, Arm.FirstStageState.RAISED, Arm.FirstStageState.RAISED)));
				}
			}
			return armDownRoutine;
		}
	}
}
