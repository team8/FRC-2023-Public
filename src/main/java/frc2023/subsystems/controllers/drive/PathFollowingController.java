package frc2023.subsystems.controllers.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

import frc2023.config.constants.DriveConstants;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.subsystems.Drive;
import frc2023.util.LiveGraph;

public class PathFollowingController extends Drive.DriveController {

	private HolonomicDriveController controller = new HolonomicDriveController(
			// 1.25 0.1
			new PIDController(0.75, 0, 0.0), new PIDController(0.75, 0, 0.0),
			new ProfiledPIDController(3.0, 0, 0,
					new TrapezoidProfile.Constraints(7.36, 6.14)));
	private Trajectory trajectory;
	private final Timer timer = new Timer();

	public PathFollowingController() {
		timer.start();
	}

	@Override
	public void updateSignal(Commands commands, RobotState state) {
		Trajectory wantedTrajectory = commands.getDriveWantedTrajectory();
		if (trajectory != wantedTrajectory) {
			// We want our update function to define every state associated with
			// this controller, so we must take care to handle these internal states to
			// avoid errors and state bleeding.
			trajectory = wantedTrajectory;
			var thetaController = new ProfiledPIDController(4.0, 0, 0,
					new TrapezoidProfile.Constraints(7.36, 6.14));
			thetaController.enableContinuousInput(-Math.PI, Math.PI);

			controller = new HolonomicDriveController(
					// 1.25 0.1
					new PIDController(0.75, 0, 0.0),
					new PIDController(0.75, 0, 0.0),
					thetaController);
			timer.reset();
		}
		Trajectory.State targetPose = wantedTrajectory.sample(timer.get());
		LiveGraph.add("Auto/targetPose", targetPose.poseMeters);
		LiveGraph.add("Auto/targetVel", targetPose.velocityMetersPerSecond);
		LiveGraph.add("Auto/realVel", Math.hypot(DriveConstants.swerveKinematics.toChassisSpeeds(state.driveSwerveModuleStates).vxMetersPerSecond, DriveConstants.swerveKinematics.toChassisSpeeds(state.driveSwerveModuleStates).vyMetersPerSecond));
		// TODO: ensure the rotation is correct
		ChassisSpeeds adjustedSpeeds = controller.calculate(
				state.robotPoseMeters,
				targetPose,
				targetPose.poseMeters.getRotation());

		LiveGraph.add("Autos/targetPos", targetPose.poseMeters);
		LiveGraph.add("Autos/targetRotation", targetPose.poseMeters.getRotation().getDegrees());
		LiveGraph.add("Autos/rotation", state.robotPoseMeters.getRotation().getDegrees());
		outputs.swerveModuleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(adjustedSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(outputs.swerveModuleStates, DriveConstants.maxSpeed);
		outputs.isOpenLoop = false;
	}
}
