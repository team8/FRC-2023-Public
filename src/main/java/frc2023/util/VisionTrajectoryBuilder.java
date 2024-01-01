package frc2023.util;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;

import frc2023.robot.Robot;
import frc2023.robot.RobotState;

public class VisionTrajectoryBuilder {

	private PathPlannerTrajectory traj;
	private ArrayList<Pose2d> mWaypoints;
	private PathConstraints constraints;
	private RobotState mState;
	private boolean mSubstation;
	private final DriverStation.Alliance alliance = Robot.alliance;
	private final double lowerXRedCharge = 11.2;
	private final double upperXRedCharge = 13.3;

	public VisionTrajectoryBuilder(ArrayList<Pose2d> waypoints, double maxVelocity, double maxAcceleration, RobotState state, boolean substation) {

		constraints = new PathConstraints(maxVelocity, maxAcceleration);
		mWaypoints = waypoints;
		mState = state;
		mSubstation = substation;

	}

	// combines two connected paths by removing the point common to them
	// vOverride is the velocity to be taken at that point
	public PathPlannerTrajectory add(VisionTrajectoryBuilder other, double vOverride) {
		List<PathPoint> myPoints = this.getPoints();
		List<PathPoint> otherPoints = other.getPoints();
		otherPoints.remove(0);

		ArrayList<PathPoint> finalPoints = new ArrayList<>(myPoints);
		finalPoints.addAll(otherPoints);
		PathPoint inter = finalPoints.get(myPoints.size() - 1);
		PathPoint newInter = new PathPoint(inter.position, inter.heading, inter.holonomicRotation, vOverride);
		finalPoints.set(myPoints.size() - 1, newInter);

		return PathPlanner.generatePath(constraints, finalPoints);
	}

	public PathPlannerTrajectory getTrajectory() {
		List<PathPoint> pts;
		int blueMult = (alliance == DriverStation.Alliance.Blue) ? 1 : 0;
		//14 is edge of charge station
		boolean pastChargeStation = false;
		if (alliance == DriverStation.Alliance.Blue) {
			pastChargeStation = mWaypoints.get(0).getX() <= Math.pow(-1, blueMult) * (upperXRedCharge - 16.54 * blueMult);
		} else {
			pastChargeStation = mWaypoints.get(0).getX() >= upperXRedCharge;
		}
		if (pastChargeStation)
			//pts = createPathMarkers(mWaypoints, simpleToPathPoints(mWaypoints, mState));
			pts = simpleToPathPoints(mWaypoints, mState);
		else {
			pts = advancedToPathPoints(mWaypoints, mState);
		}

		if (mSubstation) pts = simpleToPathPoints(mWaypoints, mState);

		traj = PathPlanner.generatePath(constraints, pts);
		return traj;
	}

	public List<PathPoint> getPoints() {
		List<PathPoint> pts;
		int blueMult = (alliance == DriverStation.Alliance.Blue) ? 1 : 0;
		//14 is edge of charge station
		boolean pastChargeStation = false;
		if (alliance == DriverStation.Alliance.Blue) {
			pastChargeStation = mWaypoints.get(0).getX() <= Math.pow(-1, blueMult) * (upperXRedCharge - 16.54 * blueMult);
		} else {
			pastChargeStation = mWaypoints.get(0).getX() >= upperXRedCharge;
		}
		if (pastChargeStation)
			//pts = createPathMarkers(mWaypoints, simpleToPathPoints(mWaypoints, mState));
			pts = simpleToPathPoints(mWaypoints, mState);
		else {
			pts = advancedToPathPoints(mWaypoints, mState);
		}

		if (mSubstation) pts = simpleToPathPoints(mWaypoints, mState);

		return pts;
	}

	public PathPlannerTrajectory getSimpleTrajectory(RobotState state) {
		Rotation2d heading = new Rotation2d();
		if (alliance == DriverStation.Alliance.Blue) heading = new Rotation2d(Math.PI);
		PathPoint start = new PathPoint(
				new Translation2d(mWaypoints.get(0).getX(), mWaypoints.get(0).getY()),
				heading, mWaypoints.get(0).getRotation(), 0);
		PathPoint end = new PathPoint(
				new Translation2d(mWaypoints.get(1).getX(), mWaypoints.get(1).getY()),
				heading, mWaypoints.get(1).getRotation());

		List<PathPoint> pathPoints = new ArrayList<>(List.of(start, end));
		return PathPlanner.generatePath(constraints, pathPoints);

	}

	// "simple" because it only takes start point and endpoint
	private List<PathPoint> simpleToPathPoints(ArrayList<Pose2d> waypoints, RobotState state) {
		List<PathPoint> pathPoints = new ArrayList<>();

		int blueMult = (alliance == DriverStation.Alliance.Blue) ? 1 : 0;

		Rotation2d heading = new Rotation2d();
		if (alliance == DriverStation.Alliance.Blue) heading = new Rotation2d(Math.PI);

		if (waypoints.size() != 2) {
			throw new IllegalArgumentException("Error in TrajectoryBuilder. You must pass only a start point and an end point in the" +
					" waypoint ArrayList.");
		}

		Translation2d translation = new Translation2d(waypoints.get(0).getX(), waypoints.get(0).getY());
		PathPoint point;

		boolean pastChargeStation = false;
		if (alliance == DriverStation.Alliance.Blue) {
			pastChargeStation = mWaypoints.get(0).getX() <= Math.pow(-1, blueMult) * (upperXRedCharge - 16.54 * blueMult);
		} else {
			pastChargeStation = mWaypoints.get(0).getX() >= upperXRedCharge;
		}

		if (pastChargeStation) point = new PathPoint(translation, heading.rotateBy(new Rotation2d(Math.PI)), waypoints.get(0).getRotation(), state.driveVelocity);
		else point = new PathPoint(translation, heading, waypoints.get(0).getRotation(), state.driveVelocity);

		pathPoints.add(point);
		translation = new Translation2d(waypoints.get(1).getX(), waypoints.get(1).getY());
		point = new PathPoint(translation, heading, waypoints.get(1).getRotation());
		pathPoints.add(point);

		return pathPoints;
	}

	// "advanced" because it builds a trajectory with intermediate points (to avoid charge station etc)
	private List<PathPoint> advancedToPathPoints(ArrayList<Pose2d> waypoints, RobotState state) {

		int blueMult = (alliance == DriverStation.Alliance.Blue) ? 1 : 0;

		Rotation2d heading = new Rotation2d();
		if (alliance == DriverStation.Alliance.Blue) heading = new Rotation2d(Math.PI);

		List<PathPoint> pathPoints = new ArrayList<>();

		Pose2d startPoint = waypoints.get(0);
		Pose2d endPoint = waypoints.get(waypoints.size() - 1);

		PathPoint startPathPoint;
		PathPoint endPathPoint;

		Translation2d startTranslation = startPoint.getTranslation();

		boolean pastChargeStation = false;
		if (alliance == DriverStation.Alliance.Blue) {
			pastChargeStation = mWaypoints.get(0).getX() <= Math.pow(-1, blueMult) * (upperXRedCharge - 16.54 * blueMult);
		} else {
			pastChargeStation = mWaypoints.get(0).getX() >= upperXRedCharge;
		}

		if (pastChargeStation) startPathPoint = new PathPoint(startTranslation, heading.rotateBy(new Rotation2d(Math.PI)), waypoints.get(0).getRotation(), state.driveVelocity);
		else startPathPoint = new PathPoint(startTranslation, heading, waypoints.get(0).getRotation(), state.driveVelocity);

		Translation2d endTranslation = endPoint.getTranslation();
		endPathPoint = new PathPoint(endTranslation, heading, endPoint.getRotation());

		// if startpoint is next to charge station and not clear of it
		// First clear charge station along x, then drive to behind, then park
		// if startpoint is clear and behind dcharge station
		// figure out which way is fastest
		// clear charge station along y and clear charge station along x ( go to corner)
		// move into community, drive to behind, park

		boolean intersectingChargeStation = false;

		if (alliance == DriverStation.Alliance.Blue) {
			intersectingChargeStation = startPoint.getX() <= Math.pow(-1, blueMult) * (lowerXRedCharge - 16.54 * blueMult) && startPoint.getX() >= Math.pow(-1, blueMult) * (upperXRedCharge - 16.54 * blueMult);
		} else {
			intersectingChargeStation = startPoint.getX() >= lowerXRedCharge && startPoint.getX() <= upperXRedCharge;
		}

		if (intersectingChargeStation || startPoint.getY() <= 1.2 || startPoint.getY() >= 4.3) {
			double clearanceY = state.robotPoseMeters.getY();
			/*
			if (startPoint.getY() > 2.8) {
				// on upper side of charge station
				clearanceY = 4.6;
			} else {
				// on lower side of charge station
				clearanceY = 0.8;
			}
			 */
			// figure out the angle beetween startpoint and clearance point
			// arctan (delta y / delta x)
			double clearanceHeading = Math.atan2(startPoint.getY() - clearanceY, startPoint.getX() - Math.pow(-1, blueMult) * (upperXRedCharge + 0.0 - 16.54 * blueMult)) + Math.PI;
			// change heading of start point
			startPathPoint = new PathPoint(startPathPoint.position, new Rotation2d(clearanceHeading), startPathPoint.holonomicRotation, startPathPoint.velocityOverride);
			// TODO: the x value for this point should be 14 ... doesn't work on our field tho : - )
			PathPoint chargeStationClearancePoint = new PathPoint(new Translation2d(Math.pow(-1, blueMult) * (upperXRedCharge + 0.0 - 16.54 * blueMult), clearanceY), new Rotation2d(clearanceHeading), endPoint.getRotation());
			PathPoint alignmentPoint = new PathPoint(new Translation2d(endPoint.getX() - Math.pow(-1, blueMult) * 0.5, endPoint.getY()), heading, endPoint.getRotation());

			pathPoints.add(startPathPoint);
			pathPoints.add(chargeStationClearancePoint);
			pathPoints.add(alignmentPoint);
			pathPoints.add(endPathPoint);
		} else {
			// we do something very similar for way past charge station
			double clearanceY;
			double directionMult;
			if (startPoint.getY() > 2.8) {
				// on upper side of charge station
				clearanceY = 4.6;
				directionMult = 1;
				if (startPoint.getY() >= 5.1) directionMult = -1;
			} else {
				// on lower side of charge station
				clearanceY = 0.8;
				directionMult = -1;
				if (startPoint.getY() <= 0.4) directionMult = 1;
			}
			// change heading of start point
			startPathPoint = new PathPoint(startPathPoint.position, new Rotation2d(directionMult * Math.PI / 2), startPathPoint.holonomicRotation, startPathPoint.velocityOverride);
			PathPoint chargeStationYClearancePoint = new PathPoint(new Translation2d(Math.pow(-1, blueMult) * (lowerXRedCharge - 16.54 * blueMult), clearanceY), heading, endPoint.getRotation());
			// TODO: the x value for this point should be 14 ... doesn't work on our field tho : - )
			PathPoint chargeStationXClearancePoint = new PathPoint(new Translation2d(Math.pow(-1, blueMult) * (upperXRedCharge + 0 - 16.54 * blueMult), clearanceY), heading, endPoint.getRotation());
			PathPoint alignmentPoint = new PathPoint(new Translation2d(endPoint.getX() - Math.pow(-1, blueMult) * 0.5, endPoint.getY()), heading, endPoint.getRotation());

			pathPoints.add(startPathPoint);
			pathPoints.add(chargeStationYClearancePoint);
			pathPoints.add(chargeStationXClearancePoint);
			pathPoints.add(alignmentPoint);
			pathPoints.add(endPathPoint);

		}

		return pathPoints;
	}

}
