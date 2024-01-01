package frc2023.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;

import frc2023.behavior.RoutineBase;
import frc2023.subsystems.Arm;
import frc2023.subsystems.Drive;
import frc2023.subsystems.Intake;
import frc2023.subsystems.Lighting;
import frc2023.subsystems.Pivot;
import frc2023.util.LiveGraph;

/**
 * Commands represent what we want the robot to be doing.
 */
public class Commands {

	/* Routines */
	public List<RoutineBase> routinesWanted = new ArrayList<>();
	public List<Class<? extends RoutineBase>> wantedCancels = new ArrayList<>();
	public boolean shouldClearCurrentRoutines;
	/* Robot */
	public boolean shouldResetOdometry = false;
	public Pose2d resetOdometry = new Pose2d();
	/* Drive */
	private Drive.State driveWantedState = Drive.State.NEUTRAL;
	public double xOffset;
	public double yOffset;
	/* Intake */
	public Intake.RollerState intakeRollerWantedState = Intake.RollerState.IDLE;
	public double intakeCustomPercentOutput;
	// teleop
	private double driveWantedTranslation, driveWantedStrafe, driveWantedRotation;
	private boolean driveFieldRelativeWanted, driveOpenLoopWanted;
	// auto
	private Trajectory driveWantedTrajectory;
	private PathPlannerTrajectory driveWantedPP;
	/* Vision */
	public boolean visionLEDsWanted;
	public PathPlannerTrajectory wantedVisionTrajectory;

	/* Lighting */
	public Lighting.State lightingWantedState = Lighting.State.DO_NOTHING;

	/* Arm */
	public Arm.FirstStageState wantedArmFirstStageState = Arm.FirstStageState.RAISED;
	public Arm.SecondStageState wantedArmSecondStageState = Arm.SecondStageState.RE_ZERO;
	public double armSecondStageWantedVelocityDegreesPerSecond;
	public double armSecondStageWantedPositionDegrees;
	public double armTempWantedVel;
	public boolean shouldMoveArmVision = false;
	public double armVisionMoveTime = 0.0;
	public double wantedArmPosition;
	public double armWantedPercentOutput;

	/* Pivot */
	public Pivot.State wantedPivotState = Pivot.State.NEUTRAL;
	public double wantedPivotPercentOutput;
	public double wantedPivotAngle;

	public void addWantedRoutines(RoutineBase... wantedRoutines) {
		for (RoutineBase wantedRoutine : wantedRoutines) {
			addWantedRoutine(wantedRoutine);
		}
	}

	public synchronized void addWantedRoutine(RoutineBase wantedRoutine) {
		routinesWanted.add(wantedRoutine);
	}

	public synchronized void stopRoutine(Class<? extends RoutineBase> wantedRoutine) {
		wantedCancels.add(wantedRoutine);
	}

	public synchronized void stopRoutine(Class<? extends RoutineBase>... wantedRoutines) {
		for (var wantedRoutine : wantedRoutines) {
			stopRoutine(wantedRoutine);
		}
	}

	@Override
	public String toString() {
		var log = new StringBuilder();
		log.append("Wanted routines: ");
		for (RoutineBase routine : routinesWanted) {
			log.append(routine).append(" ");
		}
		return log.append("\n").toString();
	}

	public void copyTo(Commands other) {
		other.shouldClearCurrentRoutines = shouldClearCurrentRoutines;
		other.routinesWanted.addAll(routinesWanted);
	}

	public Drive.State getDriveWantedState() {
		return driveWantedState;
	}

	public double getDriveWantedTranslation() {
		return driveWantedTranslation;
	}

	public PathPlannerTrajectory getDriveWantedPPTrajectory() {
		return driveWantedPP;
	}

	public PathPlannerTrajectory getDriveWantedVisionTrajectory() {
		return wantedVisionTrajectory;
	}

	public double getDriveWantedStrafe() {
		return driveWantedStrafe;
	}

	public double getDriveWantedRotation() {
		return driveWantedRotation;
	}

	public boolean isDriveFieldRelativeWanted() {
		return driveFieldRelativeWanted;
	}

	public boolean isDriveOpenLoopWanted() {
		return driveOpenLoopWanted;
	}

	public Trajectory getDriveWantedTrajectory() {
		return driveWantedTrajectory;
	}

	public void setDriveTeleop() {
		setDriveTeleop(0.0, 0.0, 0.0, true, false);
	}

	public void setDriveVision() {
		driveWantedState = Drive.State.VISION_ALIGN;
	}

	public void setDriveTrajectoryVision() {
		driveWantedState = Drive.State.VISION_ALIGN_TRAJ;
	}

	public void setDriveTeleop(double translation, double strafe, double rotation, boolean isFieldRelative, boolean isOpenLoop) {
		driveWantedState = Drive.State.TELEOP;
		driveWantedTranslation = translation;
		driveWantedRotation = rotation;
		driveWantedStrafe = strafe;
		driveFieldRelativeWanted = isFieldRelative;
		driveOpenLoopWanted = isOpenLoop;
	}

	public void setDriveVision(double translation, double strafe, double rotation, boolean isFieldRelative, boolean isOpenLoop, double xOff, double yOff) {
		driveWantedState = Drive.State.VISION_ALIGN;
		driveWantedTranslation = translation;
		driveWantedRotation = rotation;
		driveWantedStrafe = strafe;
		driveFieldRelativeWanted = isFieldRelative;
		driveOpenLoopWanted = isOpenLoop;
		xOffset = xOff;
		yOffset = yOff;
	}

	public void setDrivePathFollowing(Trajectory trajectory) {
		driveWantedState = Drive.State.PATH_FOLLOWING;
		driveWantedTrajectory = trajectory;
	}

	public void setDrivePPFollowing(PathPlannerTrajectory trajectory) {
		driveWantedState = Drive.State.PATH_FOLLOWING;
		driveWantedPP = trajectory;
	}

	public void resetOdometry(Pose2d pose) {
		shouldResetOdometry = true;
		resetOdometry = pose;
	}

	/* Intake */
	public Intake.RollerState getIntakeRollerWantedState() {
		return intakeRollerWantedState;
	}

	public void reset() {
		shouldResetOdometry = false;
	}

	public void log() {
		LiveGraph.add("Drive/driveWantedTranslation", driveWantedTranslation);
		LiveGraph.add("Drive/driveWantedStrafe", driveWantedStrafe);
		LiveGraph.add("Drive/driveWantedRotation", driveWantedRotation);
		LiveGraph.add("Drive/driveFieldRelativeWanted", driveFieldRelativeWanted);
		LiveGraph.add("Drive/driveOpenLoopWanted", driveOpenLoopWanted);
	}
}
