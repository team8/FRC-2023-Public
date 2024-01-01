package frc2023.subsystems.controllers.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;

import frc2023.config.constants.DriveConstants;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.subsystems.Drive;

public class VisionTrajectoryController extends Drive.DriveController {

	private PPHolonomicDriveController controller = new PPHolonomicDriveController(
			new PIDController(8.5, 0, 0),
			new PIDController(8.5, 0, 0),
			// at 15.5 overshoot by abt 3 cm
			new PIDController(5.2, 0, 0));
	private Trajectory trajectory;
	private final Timer timer = new Timer();

	public VisionTrajectoryController() {
		timer.start();
	}

	@Override
	public void updateSignal(Commands commands, RobotState state) {
		PathPlannerTrajectory wantedTrajectory = commands.getDriveWantedVisionTrajectory();
		if (trajectory != wantedTrajectory) {
			// We want our update function to define every state associated with
			// this controller, so we must take care to handle these internal states to
			// avoid errors and state bleeding.
			trajectory = wantedTrajectory;
			timer.reset();
		}
		PathPlannerTrajectory.State targetPose = wantedTrajectory.sample(timer.get());
		//System.out.println("NEXT TARGET: " + targetPose);
		//System.out.println("LAST TARGET: " + wantedTrajectory.sample(wantedTrajectory.getTotalTimeSeconds()));
		//System.out.println(targetPose.poseMeters);
		// TODO: ensure the rotation is correct
		if (state.simulated) state.robotPoseMeters = state.visionPose;
		ChassisSpeeds adjustedSpeeds = controller.calculate(
				state.robotPoseMeters, (PathPlannerTrajectory.PathPlannerState) targetPose);

		outputs.swerveModuleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(adjustedSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(outputs.swerveModuleStates, DriveConstants.maxSpeed);
		outputs.isOpenLoop = false;
	}

}
