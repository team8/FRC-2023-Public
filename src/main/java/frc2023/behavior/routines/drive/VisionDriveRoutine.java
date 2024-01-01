package frc2023.behavior.routines.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import frc2023.behavior.RoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.Robot;
import frc2023.robot.RobotState;
import frc2023.subsystems.Drive;
import frc2023.subsystems.Lighting;
import frc2023.subsystems.SubsystemBase;
import frc2023.util.LiveGraph;
import frc2023.util.VisionTrajectoryBuilder;

public class VisionDriveRoutine extends RoutineBase {

	private PathPlannerTrajectory trajectoryOne;
	private PathPlannerTrajectory trajectoryTwo;
	private PathPlannerTrajectory trajectoryFinal;
	private Pose2d endpoint;
	private boolean switched;
	private double mError = Double.MAX_VALUE;
	private double mRotationError = Double.MAX_VALUE;

	private Timer mTimer = new Timer();

	// runs trajectory to lineup point
	// resets robot pose in between
	// runs second part of trajectory which lines up all the way based on the closer camera

	public VisionDriveRoutine(Commands commands, RobotState state, ArrayList<Pose2d> waypoints, double maxAcceleration, double maxVelocity, int wantedIdx, boolean substation, String height) {

		int blueMult = (Robot.alliance == DriverStation.Alliance.Blue) ? 1 : 0;
		double endPointMod = 0;
		if (height.equals("low")) endPointMod = 0.15;

		ArrayList<Pose2d> waypointsCopy = new ArrayList<>(waypoints);
		Pose2d originalEndpoint = waypoints.get(1);
		endpoint = waypoints.get(1);
		originalEndpoint = new Pose2d(originalEndpoint.getX() - Math.pow(-1, blueMult) * endPointMod, originalEndpoint.getY(), originalEndpoint.getRotation());
		if (!substation) {
			waypointsCopy.set(1, new Pose2d(originalEndpoint.getX() - Math.pow(-1, blueMult) * 0.1, originalEndpoint.getY(), originalEndpoint.getRotation()));
		} else {
			/*
			Rotation2d rot;
			if (Robot.alliance == DriverStation.Alliance.Red) {
				rot = new Rotation2d(Math.PI);
			} else {
				rot = new Rotation2d();
			}
			
			 */
			waypointsCopy.set(1, new Pose2d(originalEndpoint.getX(), originalEndpoint.getY(), new Rotation2d()));
		}
		// add conditional for left / right cam here
		VisionTrajectoryBuilder builderOne = new VisionTrajectoryBuilder(waypointsCopy, maxVelocity, maxAcceleration, state, substation);
		trajectoryOne = builderOne.getSimpleTrajectory(state);
		ArrayList<Pose2d> secondWaypoints = new ArrayList<>(List.of(waypointsCopy.get(1), originalEndpoint));
		System.out.println(originalEndpoint.getRotation().toString());
		VisionTrajectoryBuilder builderTwo = new VisionTrajectoryBuilder(secondWaypoints, maxVelocity, maxAcceleration, state, substation);
		trajectoryTwo = builderTwo.getSimpleTrajectory(state);
		mTimer.reset();

		//TODO: tune this value
		trajectoryFinal = builderOne.add(builderTwo, 0.6);
	}

	@Override
	protected void start(Commands commands, @ReadOnly RobotState state) {
		// Required to start the timeout timer
		mTimer.start();
		commands.wantedVisionTrajectory = trajectoryFinal;
		commands.setDriveTrajectoryVision();
		LiveGraph.add("Robot/visionEndpose", commands.getDriveWantedVisionTrajectory().getEndState().poseMeters);
		commands.lightingWantedState = Lighting.State.LINE_UP;
	}

	@Override
	protected void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
		LiveGraph.add("Robot/visionEndpose", commands.getDriveWantedVisionTrajectory().getEndState().poseMeters);
		mError = Math.sqrt(Math.pow((state.robotPoseMeters.getX() - endpoint.getX()), 2) + Math.pow((state.robotPoseMeters.getY() - endpoint.getY()), 2));
		mRotationError = Math.abs(state.robotPoseMeters.getRotation().getRadians() - endpoint.getRotation().getRadians());
		LiveGraph.add("Robot/visionError", mError);
		LiveGraph.add("Robot/visionEndpoint", endpoint);
	}

	@Override
	protected void stop(Commands commands, @ReadOnly RobotState state) {
		commands.lightingWantedState = Lighting.State.ALIGN;
		state.drivePathFollowing = false;
	}

	@Override
	public boolean checkFinished(RobotState state) {
		if ((mTimer.get() > trajectoryFinal.getTotalTimeSeconds() || (mError <= 0.05 && mRotationError <= 0.05))) {
			return true;
		} else return false;
	}

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return Set.of(Drive.class);
	}

	public double getInitialLineupTime() {
		return trajectoryOne.getTotalTimeSeconds();
	}

}
