package frc2023.subsystems.controllers.drive;

import java.util.ArrayList;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

import frc2023.config.constants.DriveConstants;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.subsystems.Drive;
import frc2023.util.LiveGraph;

public class VisionPathFollowingController extends Drive.DriveController {

	private HolonomicDriveController controller = new HolonomicDriveController(
			// 1.25 0.1
			new PIDController(0.75, 0, 0.0), new PIDController(0.75, 0, 0.0),
			new ProfiledPIDController(3.0, 0, 0,
					new TrapezoidProfile.Constraints(7.36, 6.14)));
	private final Timer timer = new Timer();
	private int pointer = 0;

	public VisionPathFollowingController() {
		timer.start();
		pointer = 0;
	}

	@Override
	public void updateSignal(Commands commands, RobotState state) {
		ArrayList<Pose2d> targets = new ArrayList<>();
		var realPose = new Pose2d(state.visionPose.getX(), state.visionPose.getY(), state.robotPoseMeters.getRotation());
		var rotationTarget = new Pose2d(state.visionPose.getX(), state.visionPose.getY(), new Rotation2d(Math.PI));
		var thetaController = new ProfiledPIDController(7.5, 0, 0,
				new TrapezoidProfile.Constraints(500000000.6, 10000.14));
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		controller = new HolonomicDriveController(
				// 1.25 0.1
				new PIDController(1.5, 0, 0.0),
				new PIDController(1.5, 0, 0.0),
				thetaController);

		var rotationTolerance = (new Pose2d(10.0, 10.0, new Rotation2d(0.18)));
		var driveTolerance = (new Pose2d(0.75, 0.1, new Rotation2d(10.0)));

		ChassisSpeeds adjustedSpeeds;

		if (pointer == 0) {
			adjustedSpeeds = controller.calculate(realPose, rotationTarget, 0.0, rotationTarget.getRotation());
			if (withinTolerance(rotationTolerance, realPose, rotationTarget)) {
				if (pointer < targets.size()) pointer++;
			}
		} else {
			/*
			if (state.haveVision) adjustedSpeeds = controller.calculate(state.visionPose, targets.get(pointer-1), 0.7, new Rotation2d());
			else
				adjustedSpeeds = controller.calculate(realPose, rotationTarget, 0.0, rotationTarget.getRotation());
			 */
			Pose2d noRotationPose = new Pose2d(state.robotPoseMeters.getX(), state.robotPoseMeters.getY(), new Rotation2d());
			adjustedSpeeds = controller.calculate(noRotationPose, targets.get(pointer - 1), 0.7, new Rotation2d());

			if (withinTolerance(driveTolerance, state.visionPose, targets.get(pointer - 1))) {
				if (pointer < targets.size() - 1) pointer++;
			}

		}
		//LiveGraph.add("Robot/nextTarget", target);
		/*
		if (withinTolerance(tolerance, state.visionPose, targets.get(pointer))) {
			if (pointer < targets.size()-1)
			pointer++;
		}
		 */
		outputs.swerveModuleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(adjustedSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(outputs.swerveModuleStates, DriveConstants.maxSpeed);
		outputs.isOpenLoop = false;
		LiveGraph.add("Robot/visionPose", state.visionPose);

	}

	private boolean withinTolerance(Pose2d tolerance, Pose2d position, Pose2d target) {
		System.out.println(pointer);
		System.out.println("X DIFF: " + Math.abs(position.getX() - target.getX()));
		System.out.println("Y DIFF: " + Math.abs(position.getY() - target.getY()));
		System.out.println("ROT DIFF: " + Math.abs(position.getRotation().getRadians() - target.getRotation().getRadians()));
		if (Math.abs(position.getX() - target.getX()) > tolerance.getX()) return false;
		if (Math.abs(position.getY() - target.getY()) > tolerance.getY()) return false;
		if (Math.abs(position.getRotation().getRadians() - target.getRotation().getRadians()) > tolerance.getRotation().getRadians()) return false;
		return true;

	}
}
