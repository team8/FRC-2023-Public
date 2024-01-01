package frc2023.behavior.routines.drive;

import java.util.*;
import java.util.function.Predicate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.config.constants.DriveConstants;
import frc2023.config.subsystem.DriveConfig;
import frc2023.robot.Commands;
import frc2023.robot.ReadOnly;
import frc2023.robot.RobotState;
import frc2023.subsystems.Drive;
import frc2023.subsystems.SubsystemBase;
import frc2023.util.LiveGraph;
import frc2023.util.config.Configs;

public class DrivePathRoutine extends TimeoutRoutineBase {
	//todo: fix all timed routines

	private static final double drivePathError = 0.1;

	private static final DriveConfig config = Configs.get(DriveConfig.class);
	private final String name;
	protected static final double timeoutMultiplier = 1.1;
	private final List<Pose2d> poses;
	private List<Translation2d> waypoints;
	private double maxVelocityMetersPerSecond = config.pathVelocityMetersPerSecond,
			maxAccelerationMetersPerSecondSq = config.pathAccelerationMetersPerSecondSquared;
	private double startingVelocityMetersPerSecond, endingVelocityMetersPerSecond;
	private List<TrajectoryConstraint> constraints = new ArrayList<>();
	private boolean shouldReversePath, driveInReverse;
	protected Trajectory trajectory;

	/**
	 * @param poses Points to move towards from current pose. No initial pose needs to be supplied.
	 */
	public DrivePathRoutine(String name, Pose2d... poses) {
		this(name, Arrays.asList(poses));
	}

	/**
	 * @see #DrivePathRoutine(String, Pose2d...)
	 */
	public DrivePathRoutine(String name, List<Pose2d> poses) {
		this.poses = poses;
		this.timeout = 3;
		this.name = name;
	}

	public DrivePathRoutine startingVelocity(double velocityMetersPerSecond) {
		startingVelocityMetersPerSecond = velocityMetersPerSecond;
		return this;
	}

	public DrivePathRoutine addConstraint(TrajectoryConstraint constraint) {
		constraints.add(constraint);
		return this;
	}

	public DrivePathRoutine endingVelocity(double velocityMetersPerSecond) {
		endingVelocityMetersPerSecond = velocityMetersPerSecond;
		return this;
	}

	public DrivePathRoutine setMovement(double velocityMetersPerSecond, double accelerationMetersPerSecondPerSecond) {
		maxVelocityMetersPerSecond = velocityMetersPerSecond;
		maxAccelerationMetersPerSecondSq = accelerationMetersPerSecondPerSecond;
		return this;
	}

	public DrivePathRoutine limitWhen(double maxVelocityMetersPerSecond, Predicate<Pose2d> predicate) {
		constraints.add(new TrajectoryConstraint() {

			@Override
			public double getMaxVelocityMetersPerSecond(Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond) {
				if (predicate.test(poseMeters)) {
					return maxVelocityMetersPerSecond;
				}
				return Double.POSITIVE_INFINITY;
			}

			@Override
			public MinMax getMinMaxAccelerationMetersPerSecondSq(Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond) {
				return new MinMax(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
			}
		});
		return this;
	}

	public DrivePathRoutine waypoints(List<Translation2d> waypoints) {
		this.waypoints = waypoints;
		return this;
	}

	/**
	 * Robot will try to drive in reverse while traversing the path. Does not reverse the path itself.
	 */
	public DrivePathRoutine driveInReverse() {
		driveInReverse = true;
		return this;
	}

	/**
	 * Reverse points in the path. Does not make the robot drive in reverse.
	 */
	public DrivePathRoutine reversePath() {
		shouldReversePath = true;
		return this;
	}

	/**
	 * Reverses the path and attempts to drive it backwards. Useful for getting a robot back to its
	 * starting position after running a path.
	 */
	public DrivePathRoutine reverse() {
		driveInReverse();
		reversePath();
		return this;
	}

	public void generateTrajectory(Pose2d startingPose) {
		if (trajectory == null) {
			var posesWithStart = new LinkedList<>(poses);
			if (shouldReversePath) {
				Collections.reverse(posesWithStart);
			}
			posesWithStart.addFirst(startingPose);
			var trajectoryConfig = DriveConstants.getTrajectoryConfig(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);
			trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(1.6));
			trajectoryConfig.setReversed(driveInReverse);
			trajectoryConfig.setStartVelocity(startingVelocityMetersPerSecond);
			trajectoryConfig.setEndVelocity(endingVelocityMetersPerSecond);
			trajectoryConfig.addConstraints(constraints);
			trajectory = waypoints == null ?
					TrajectoryGenerator.generateTrajectory(posesWithStart, trajectoryConfig) :
					TrajectoryGenerator.generateTrajectory(posesWithStart.getFirst(), waypoints, posesWithStart.getLast(),
							trajectoryConfig);
			timeout = trajectory.getTotalTimeSeconds() * timeoutMultiplier;
		} else {
			throw new IllegalStateException("Trajectory already generated!");
		}
	}

	public Trajectory getTrajectory() {
		return trajectory;
	}

	@Override
	public void start(Commands commands, @ReadOnly RobotState state) {
		// Required to start the timeout timer
		super.start(commands, state);
		generateTrajectory(state.robotPoseMeters);
	}

	@Override
	public Set<Class<? extends SubsystemBase>> getRequiredSubsystems() {
		return Set.of(Drive.class);
	}

	@Override
	public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
		commands.setDrivePathFollowing(trajectory);
	}

	@Override
	public boolean checkIfFinishedEarly(@ReadOnly RobotState state) {
//		Pose2d finalPose = mTrajectory.getStates().get(mTrajectory.getStates().size() - 1).poseMeters;
		// TODO: possibly implement to see if we are within a tolerance of the end early
//		return isNull(finalPose) ? false : Math.abs(state.drivePoseMeters.getX() - finalPose.getX()) < kDrivePathError && Math.abs(state.drivePoseMeters.getY() - finalPose.getY()) < kDrivePathError;
		return false;
	}

	public void update(RobotState state, Commands commands) {
		commands.setDrivePathFollowing(trajectory);
		LiveGraph.add("Routines/" + name);
	}
}
