package frc2023.subsystems.controllers.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import frc2023.config.constants.DriveConstants;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.subsystems.Drive;
import frc2023.util.LiveGraph;

public class PPFollowingController extends Drive.DriveController {

	private PPHolonomicDriveController controller = new PPHolonomicDriveController(
			new PIDController(6, 0, 0),
			new PIDController(6, 0, 0),
			new PIDController(2, 0, 0));
	private Trajectory trajectory;
	private final Timer timer = new Timer();

	public PPFollowingController() {
		timer.start();
	}

	@Override
	public void updateSignal(Commands commands, RobotState state) {
		PathPlannerTrajectory wantedTrajectory = commands.getDriveWantedPPTrajectory();
		if (trajectory != wantedTrajectory) {
			// We want our update function to define every state associated with
			// this controller, so we must take care to handle these internal states to
			// avoid errors and state bleeding.
			trajectory = wantedTrajectory;
			timer.reset();
		}
		var targetPose = wantedTrajectory.sample(timer.get());
		//targetPose = transformStateForAlliance((PathPlannerTrajectory.PathPlannerState)targetPose, DriverStation.Alliance.Red);
//		if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
//			targetPose = Flippy.flipTranslation(targetPose);
//		}
		/*targetPose.poseMeters = Flippy.flipTranslation(targetPose.poseMeters);
		targetPose.velocityMetersPerSecond = -targetPose.velocityMetersPerSecond;
		//targetPose.poseMeters = new Pose2d(targetPose.poseMeters.getX(), targetPose.poseMeters.getY(), Rotation2d.fromDegrees(targetPose.poseMeters.getRotation().getDegrees() + 180));
		targetPose.curvatureRadPerMeter = targetPose.curvatureRadPerMeter;*/

		LiveGraph.add("Auto/targetPose", targetPose.poseMeters);
		LiveGraph.add("Auto/targetVel", targetPose.velocityMetersPerSecond);
		LiveGraph.add("Auto/realVel", Math.hypot(DriveConstants.swerveKinematics.toChassisSpeeds(state.driveSwerveModuleStates).vxMetersPerSecond, DriveConstants.swerveKinematics.toChassisSpeeds(state.driveSwerveModuleStates).vyMetersPerSecond));
		// TODO: ensure the rotation is correct
		ChassisSpeeds adjustedSpeeds;
		if (state.useAutoVision) {
			adjustedSpeeds = controller.calculate(
					state.robotPoseMeters, (PathPlannerTrajectory.PathPlannerState) targetPose);
		} else {
			adjustedSpeeds = controller.calculate(
					state.swervePoseMeters, (PathPlannerTrajectory.PathPlannerState) targetPose);
		}
		//LiveGraph.add("Autos/targetPos", targetPose.poseMeters);
		LiveGraph.add("Autos/targetRotation", ((PathPlannerTrajectory.PathPlannerState) targetPose).holonomicRotation);
		LiveGraph.add("Autos/rotation", state.robotPoseMeters.getRotation().getDegrees());
		outputs.swerveModuleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(adjustedSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(outputs.swerveModuleStates, DriveConstants.maxSpeed);
		outputs.isOpenLoop = false;
	}

	public static PathPlannerTrajectory.PathPlannerState transformStateForAlliance(PathPlannerTrajectory.PathPlannerState state, DriverStation.Alliance alliance) {
		if (alliance == DriverStation.Alliance.Red) {
			PathPlannerTrajectory.PathPlannerState transformedState = new PathPlannerTrajectory.PathPlannerState();
			Translation2d transformedTranslation = new Translation2d(16.5 - state.poseMeters.getX(), state.poseMeters.getY());
			Rotation2d transformedHeading = state.poseMeters.getRotation().times(1.0D);
			Rotation2d transformedHolonomicRotation = Rotation2d.fromDegrees(state.holonomicRotation.getDegrees() + 180);
			transformedState.timeSeconds = state.timeSeconds;
			transformedState.velocityMetersPerSecond = state.velocityMetersPerSecond;
			transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
			transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
			transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
			transformedState.holonomicRotation = transformedHolonomicRotation;
			transformedState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
			//transformedState.curveRadius = -state.curveRadius;
			transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;
			//transformedState.deltaPos = state.deltaPos;
			return transformedState;
		} else {
			return state;
		}
	}
}
