package frc2023.behavior.routines.drive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

import frc2023.behavior.TimeoutRoutineBase;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;

public class DrivePathPlannerRoutine extends TimeoutRoutineBase {

	private final String pathName;
	private final double maxVelocity, maxAcceleration;
	private PathPlannerTrajectory trajectory;
	private RobotState state;
	private boolean useVision;
	protected static final double timeoutMultiplier = 1.1;
	private PPHolonomicDriveController controller = new PPHolonomicDriveController(
			new PIDController(0.8, 0, 0),
			new PIDController(0.8, 0, 0),
			new PIDController(1.5, 0, 0));

	public DrivePathPlannerRoutine(String pathName, double maxVelocity, double maxAcceleration) {
		this.pathName = pathName;
		this.maxVelocity = maxVelocity;
		this.maxAcceleration = maxAcceleration;
		//red side
		trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath(pathName, maxVelocity, maxAcceleration), DriverStation.Alliance.Blue);
		//blue side
		//trajectory = PathPlanner.loadPath(pathName, maxVelocity, maxAcceleration);
		timeout = trajectory.getTotalTimeSeconds() * timeoutMultiplier;
	}

	public DrivePathPlannerRoutine(String pathName, double maxVelocity, double maxAcceleration, boolean useVision, RobotState state) {
		this.pathName = pathName;
		this.maxVelocity = maxVelocity;
		this.maxAcceleration = maxAcceleration;
		this.state = state;
		this.useVision = useVision;
		//red side
		trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath(pathName, maxVelocity, maxAcceleration), DriverStation.Alliance.Blue);
		//blue side
		//trajectory = PathPlanner.loadPath(pathName, maxVelocity, maxAcceleration);
		timeout = trajectory.getTotalTimeSeconds() * timeoutMultiplier;
	}

	public DrivePathPlannerRoutine(String pathName, double maxVelocity, double maxAcceleration, DriverStation.Alliance alliance, boolean useVision, RobotState state) {
		this.pathName = pathName;
		this.maxVelocity = maxVelocity;
		this.maxAcceleration = maxAcceleration;
		this.state = state;
		this.useVision = useVision;
		//red side
		trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath(pathName, maxVelocity, maxAcceleration), alliance);
		//blue side
		//trajectory = PathPlanner.loadPath(pathName, maxVelocity, maxAcceleration);
		timeout = trajectory.getTotalTimeSeconds() * timeoutMultiplier;
	}

	@Override
	protected void update(Commands commands, RobotState state) {
		if (this.state != null) {
			this.state.useAutoVision = this.useVision;
		}
		commands.setDrivePPFollowing(trajectory);
	}

	@Override
	public void start(Commands commands, RobotState state) {
		super.start(commands, state);
	}

	@Override
	public boolean checkIfFinishedEarly(RobotState state) {
		return false;
	}

	public Pose2d getInitialPose() {
		return trajectory.getInitialPose();
	}
}
