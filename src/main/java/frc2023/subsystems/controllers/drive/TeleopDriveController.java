package frc2023.subsystems.controllers.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc2023.config.constants.DriveConstants;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.subsystems.Drive;
import frc2023.util.LiveGraph;

public class TeleopDriveController extends Drive.DriveController {

	private Translation2d previousTranslation = new Translation2d();
	private double velocity;

	@Override
	public void updateSignal(Commands commands, RobotState state) {
		Translation2d translation = new Translation2d(
				-commands.getDriveWantedTranslation(),
				-commands.getDriveWantedStrafe()).times(DriveConstants.maxSpeed);
		double rotation = commands.getDriveWantedRotation();
		rotation = rotation * rotation * Math.signum(rotation);
		rotation *= DriveConstants.maxAngularVelocity;
		LiveGraph.add("Drive/wantedRotation", rotation);
		LiveGraph.add("Drive/Field relative", commands.isDriveFieldRelativeWanted());

//		var acc = translation.minus(previousTranslation).getNorm() * RobotConstants.period;
//		if (acc >= DriveConstants.maxAcceleration) {
//			translation = translation
//					.div(translation.getNorm())
//					.times(DriveConstants.maxAcceleration);
//		}
//		previousTranslation = translation;

		state.driveVelocity = velocity;
		SwerveModuleState[] states = drive(
				translation, rotation,
				commands.isDriveFieldRelativeWanted(),
				state.swervePoseMeters.getRotation());
		outputs.swerveModuleStates = states;
		outputs.isOpenLoop = commands.isDriveOpenLoopWanted();
	}

	private SwerveModuleState[] drive(Translation2d translation, double rotation, boolean fieldRelative, Rotation2d yaw) {
		var wantedSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
				translation.getX(),
				translation.getY(),
				rotation,
				yaw) :
				new ChassisSpeeds(
						translation.getX(),
						translation.getY(),
						rotation);
		SwerveModuleState[] swerveModuleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(wantedSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.maxSpeed);

		velocity = Math.sqrt(Math.pow(wantedSpeeds.vxMetersPerSecond, 2) + Math.pow(wantedSpeeds.vyMetersPerSecond, 2));

		LiveGraph.add("Drive/Wanted speed x", wantedSpeeds.vxMetersPerSecond);
		LiveGraph.add("Drive/Wanted speed y", wantedSpeeds.vyMetersPerSecond);
		LiveGraph.add("Drive/Wanted speed omega", wantedSpeeds.omegaRadiansPerSecond);

		return swerveModuleStates;
	}
}
