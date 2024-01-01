package frc2023.subsystems.controllers.drive;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.Timer;

import frc2023.config.constants.DriveConstants;
import frc2023.config.subsystem.DriveConfig;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.subsystems.Drive;
import frc2023.util.CircularBuffer;
import frc2023.util.LiveGraph;
import frc2023.util.config.Configs;

public class VisionAlignController extends Drive.DriveController {

	private final double errorMargin = 0.0;

	boolean stop = false;

	private HolonomicDriveController controller = new HolonomicDriveController(
			new PIDController(0.0, 0, 0), new PIDController(0.0, 0, 0),
			new ProfiledPIDController(10.0, 0, 0,
					new TrapezoidProfile.Constraints(6.28, 3.14)));
	private HolonomicDriveController controllerPID = new HolonomicDriveController(
			new PIDController(1.0, 0, 0), new PIDController(1.0, 0, 0),
			new ProfiledPIDController(10.0, 0, 0,
					new TrapezoidProfile.Constraints(6.28, 3.14)));

	private ProfiledPIDController controller2 = new ProfiledPIDController(1.5, 0, 0.0, new TrapezoidProfile.Constraints(1.0, 2.0));
	private ProfiledPIDController angleController2 = new ProfiledPIDController(20.0, 0, 0,
			new TrapezoidProfile.Constraints(6.28, 3.14));
	private double offsetX = 0.0;
	private double offsetY = 1.0;
	private boolean aligned = false;
	int count = 0;

	Pose2d pidPose = null;
	double pidResetVisionCount = 0;
	CircularBuffer<Pair<Double, Transform3d>> yAngleBuffer = new CircularBuffer<>(11);
	CircularBuffer<Transform3d> closestAngleFilter = new CircularBuffer<>(10);
	CircularBuffer<Double> ambiguityFilter = new CircularBuffer<>(10);
	Transform3d currentClosestTransform = new Transform3d();
	Translation2d translationDirection = new Translation2d();
	private double lastGoodAngleCorrection;
	private final Timer timer = new Timer();
	Translation2d translation = new Translation2d();

	private double p = 0.0;
	private double maxVel = 0.5;
	double translationTimeout;
	double rotationalTimeout;

	private double kRecalcError = 20.0;
	private Trajectory currentTrajectory = null;
	private Trajectory angleTrajectory = null;
	Pose2d finalPose = null;
	double finalAngle = 0;
	double originalAngle = 0;
	private static final DriveConfig config = Configs.get(DriveConfig.class);

	public VisionAlignController() {
		timer.start();
	}

	@Override
	public void updateSignal(Commands commands, RobotState state) {
		/*
		 * I suspect that AprilTags will be more precise than vision tape so I will initially align to the april tag and
		 * then make an adjustment to align with the cone pole.
		 */
		// align directly in front of april tag
		ChassisSpeeds currentSpeeds = DriveConstants.swerveKinematics.toChassisSpeeds(
				state.driveSwerveModuleStates);
		LiveGraph.add("vx", currentSpeeds.vxMetersPerSecond);
		LiveGraph.add("vy", currentSpeeds.vyMetersPerSecond);
		LiveGraph.add("Vision/ModuleStates/1", state.driveSwerveModuleStates[0].speedMetersPerSecond);
		LiveGraph.add("Vision/ModuleStates/2", state.driveSwerveModuleStates[1].speedMetersPerSecond);
		LiveGraph.add("Vision/ModuleStates/3", state.driveSwerveModuleStates[2].speedMetersPerSecond);
		LiveGraph.add("Vision/ModuleStates/4", state.driveSwerveModuleStates[3].speedMetersPerSecond);
		if (commands.xOffset == 1.0) {
			if (commands.yOffset == 0.0) {
				offsetX = -0.6;
				offsetY = 0.9;
			} else if (commands.yOffset == -1.0) {
				offsetX = -0.6;
				offsetY = 0.9;
			} else {
				offsetX = -0.6;
				offsetY = 0.9;
			}
		} else if (commands.xOffset == 0.0) {
			if (commands.yOffset == 0.0) {
				offsetX = -0.05;
				offsetY = 1.2;
			} else if (commands.yOffset == -1.0) {
				offsetX = -0.1;
				offsetY = 1.15;
			} else {
				offsetX = -0.1;
				offsetY = 1.1;
			}
		} else {
			if (commands.yOffset == 0.0) {
				offsetX = 0.5;
				offsetY = 0.90;
			} else if (commands.yOffset == -1.0) {
				offsetX = 0.5;
				offsetY = 0.9;
			} else {
				offsetX = 0.45;
				offsetY = 0.85;
			}
		}

		if (!state.visionCameraTargetPose.isEmpty()) {
			Transform3d cameraToTarget = state.visionCameraTargetPose.get();
			Transform3d cameraToTargetBad = state.visionCameraTargetPoseBad.get();
			Transform3d targetToCameraBad = cameraToTargetBad.inverse();
			Transform3d targetToCamera = cameraToTarget.inverse();
			if (yAngleBuffer.size() >= 1 && ambiguityFilter.size() >= 1) {
				double scoreGood = state.aprilTagAmbiguity;
				double yawLast = yAngleBuffer.get(yAngleBuffer.size() - 1).getSecond().getRotation().getZ();
				double yawCurrent = targetToCamera.getRotation().getZ();
				double ambigLast = ambiguityFilter.get(ambiguityFilter.size() - 1);
				scoreGood += (scoreGood - ambigLast);
				if (scoreGood > 1.0) {
					targetToCamera = targetToCameraBad;
				} else {
					ambiguityFilter.add(state.aprilTagAmbiguity);
				}
			} else {
				ambiguityFilter.add(state.aprilTagAmbiguity);
			}

			/*if(targetToCameraBad.getRotation().getY() + 9.0 < targetToCamera.getRotation().getY() + 9.0){
				yAngleBuffer.add(new Pair<>(Math.toDegrees(targetToCameraBad.getRotation().getY()), targetToCameraBad));
			}else{
				yAngleBuffer.add(new Pair<>(Math.toDegrees(targetToCamera.getRotation().getY()), targetToCamera));
			}*/
			yAngleBuffer.add(new Pair<>(Math.toDegrees(targetToCamera.getRotation().getY()), targetToCamera));

			int currentLowest = 0;
			double currentLowestDiff = Double.MAX_VALUE;
			for (int i = 0; i < yAngleBuffer.size(); i++) {
				if (Math.abs(yAngleBuffer.get(i).getFirst() + 9.0) < currentLowestDiff) {
					currentLowestDiff = Math.abs(yAngleBuffer.get(i).getFirst() + 9.0);
					currentLowest = i;
				}
			}
			int realBest = 0;
			for (int i = yAngleBuffer.size() - 1; i >= 0; i--) {
				if (Math.abs(yAngleBuffer.get(i).getFirst() - yAngleBuffer.get(currentLowest).getFirst()) < 6.0) {
					realBest = i;
					break;
				}
			}
			LiveGraph.add("Vision/currentTransform/xpos", targetToCamera.getX());
			LiveGraph.add("Vision/currentTransform/ypos", targetToCamera.getY());
			LiveGraph.add("Vision/currentTransform/zpos", targetToCamera.getZ());
			LiveGraph.add("Vision/currentTransform/yaw", Math.toDegrees(targetToCamera.getRotation().getZ()));
			LiveGraph.add("Vision/currentTransform/pitch", Math.toDegrees(targetToCamera.getRotation().getY()));
			LiveGraph.add("Vision/currentTransform/roll", Math.toDegrees(targetToCamera.getRotation().getX()));
			closestAngleFilter.add(yAngleBuffer.get(realBest).getSecond());

			currentClosestTransform = yAngleBuffer.get(currentLowest).getSecond();
			LiveGraph.add("Vision/closestTransform/x", currentClosestTransform.getX());
			LiveGraph.add("Vision/closestTransform/y", currentClosestTransform.getY());
			LiveGraph.add("Vision/closestTransform/z", currentClosestTransform.getZ());
			LiveGraph.add("Vision/closestTransform/yaw", Math.toDegrees(currentClosestTransform.getRotation().getZ()));
			LiveGraph.add("Vision/closestTransform/pitch", Math.toDegrees(currentClosestTransform.getRotation().getY()));
			LiveGraph.add("Vision/closestTransform/roll", Math.toDegrees(currentClosestTransform.getRotation().getX()));
			double robotRelativeAngle = Math.atan(currentClosestTransform.inverse().getY() / currentClosestTransform.inverse().getX());
			double phi = state.robotPoseMeters.getRotation().getDegrees();
			double posDiffX = offsetX - currentClosestTransform.getY();
			double posDiffY = offsetY - currentClosestTransform.getX();
			LiveGraph.add("Vision/phi", phi);
			double theta = Math.toDegrees(robotRelativeAngle);
			LiveGraph.add("Vision/theta", theta);
			double epsilon = Math.toDegrees(Math.atan(currentClosestTransform.getX() / (Math.signum(-posDiffX) * Math.abs(currentClosestTransform.getY()))));
			LiveGraph.add("Vision/epsilon", epsilon);
			double angleToRotate = phi + theta + epsilon;
			LiveGraph.add("Vision/total", angleToRotate);
			LiveGraph.add("Vision/ambiguity", state.aprilTagAmbiguity);
			// System.out.println(cameraToTarget.getRotation().getZ());
			//pretty much this works kind of like a P controller, it should slow down as it approaches the target position
			translation = new Translation2d(1, 0);

			double angle = -Math.atan(posDiffY / posDiffX);
			if (posDiffX < 0) {
				// angle = -((2 * Math.PI) - angle);
			}
			//angleToRotate = 180 - angleToRotate;
			double dist = Math.hypot(posDiffX, posDiffY);
			LiveGraph.add("Vision/distToOffset/x", posDiffX);
			LiveGraph.add("Vision/distToOffset/y", posDiffY);
			LiveGraph.add("Vision/angleOffsetNew", Math.toDegrees(angle));
			LiveGraph.add("Vision/angleOffsetOrig", angleToRotate);
			//  angleToRotate = 180 - angleToRotate;
			angleToRotate = angleToRotate;
			angleToRotate += Math.toDegrees(angle);
			Translation2d temp = new Translation2d(Math.cos(Math.toRadians(angleToRotate)), Math.sin(Math.toRadians(angleToRotate)));
			LiveGraph.add("Vision/tempTrans/x", temp.getX());
			LiveGraph.add("Vision/tempTrans/y", temp.getY());
			LiveGraph.add("Vision/angleTotalOffset", angleToRotate);
			//angleToRotate = 180-angleToRotate;
			translation = translation.rotateBy(Rotation2d.fromDegrees(angleToRotate));
			translationDirection = translation;

			LiveGraph.add("Vision/translation/x", translation.getX());
			LiveGraph.add("Vision/translation/y", translation.getY());
			translation = translation.times(dist);
			// translation.rotateBy(Rotation2d.fromDegrees(-angleToRotate));
			//translation = new Translation2d(-translation.getX(), translation.getY());
			//translation.rotateBy(Rotation2d.fromDegrees(angleToRotate));
			//translation = new Translation2d(-translation.getX(), translation.getY());
			translation = translation.plus(state.robotPoseMeters.getTranslation());
			double zAngle = currentClosestTransform.getRotation().getZ();
			if ((finalPose == null) || translation.getDistance(finalPose.getTranslation()) > kRecalcError || yAngleBuffer.size() < 5 || (timer.get() > translationTimeout && count < 2)) {
				pidPose = null;
				pidResetVisionCount = 0;
				count++;
				Translation2d diff = translation.minus(state.robotPoseMeters.getTranslation());
				double angleHeading = Math.toDegrees(Math.atan2(diff.getY(), diff.getX()));
				//  translation = translation.div(translation.getNorm());
				Pose2d finalPos2d = new Pose2d(translation.getX(), translation.getY(), Rotation2d.fromDegrees(angleHeading));

				//finalPos2d = new Pose2d(Math.cos(Math.toRadians(angleHeading)), Math.sin(Math.toRadians(angleHeading)), Rotation2d.fromDegrees(angleHeading));
				finalPose = finalPos2d;
				//Pose2d finalRotationPose = new Pose2d(state.robotPoseMeters.getTranslation(), Rotation2d.fromDegrees(state.robotPoseMeters.getRotation().getDegrees() + zAngle));
				double currentVel;
				if (currentTrajectory != null) {
					currentVel = currentTrajectory.sample(timer.get()).velocityMetersPerSecond;
				} else {
					currentVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
				}

				//angleTrajectory = generateTrajectory(finalRotationPose, state.robotPoseMeters, 0.0);
				//rotationalTimeout = angleTrajectory.getTotalTimeSeconds();
				try {
					currentTrajectory = generateTrajectory(finalPose, new Pose2d(state.robotPoseMeters.getTranslation(), Rotation2d.fromDegrees(angleHeading)), currentVel);
					finalAngle = Math.toDegrees(Math.signum(zAngle) * ((Math.PI) - Math.abs(zAngle)));
					originalAngle = state.robotPoseMeters.getRotation().getDegrees();
					translationTimeout = currentTrajectory.getTotalTimeSeconds();
					commands.shouldMoveArmVision = true;
					commands.armVisionMoveTime = translationTimeout;
					timer.reset();
				} catch (Exception E) {

				}

				System.out.println("RESET");
				System.out.println("RESET");
				System.out.println("RESET");
			}

			/* if(timer.get() > translationTimeout && dist > errorMargin && !stop){
					double velocity = Math.abs(controller2.calculate(dist));
					double heading = Math.toRadians(angleToRotate);
					double rotation = finalAngle;
					rotation += originalAngle;
					rotation = Math.toRadians(rotation);
					ChassisSpeeds adjustedSpeeds = controller.calculate(
							state.robotPoseMeters,
							currentTrajectory.sample(translationTimeout),
							Rotation2d.fromDegrees(rotation)
					);
					LiveGraph.add("pid velocity", velocity);
					ChassisSpeeds speed = new ChassisSpeeds(velocity * translationDirectionOnly.getY(), velocity * translationDirectionOnly.getX(), 0.0);
					SwerveModuleState[] swerveModuleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(speed);
					outputs.swerveModuleStates = swerveModuleStates;
				}else if((dist < errorMargin || stop) && timer.get() > translationTimeout){
					stop = true;
					double rotation = finalAngle + originalAngle;
			
					if( Math.abs(Rotation2d.fromDegrees(rotation).minus( state.robotPoseMeters.getRotation()).getDegrees()) % 360 > 1.0){
						ChassisSpeeds adjustedSpeeds = controller.calculate(
								state.robotPoseMeters,
								currentTrajectory.sample(translationTimeout),
								Rotation2d.fromDegrees(rotation)
						);
						adjustedSpeeds.vyMetersPerSecond = 0;
						adjustedSpeeds.vxMetersPerSecond = 0;
						SwerveModuleState[] swerveModuleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(adjustedSpeeds);
						outputs.swerveModuleStates = swerveModuleStates;
					}else{
						SwerveModuleState[] states = drive(new Translation2d(), 0, true, state.robotPoseMeters.getRotation());
						outputs.swerveModuleStates = states;
				// }
			
				}*/
		}
		if (currentTrajectory != null && yAngleBuffer.size() > 5) {
			LiveGraph.add("Vision/finalPos", finalPose);
			LiveGraph.add("Vision/timeout", translationTimeout);
			LiveGraph.add("Vision/timer", timer.get());
			Trajectory.State targetPose = currentTrajectory.sample(timer.get());
			//Trajectory.State targetRotation = angleTrajectory.sample(timer.get());
			LiveGraph.add("TargetPose", targetPose.poseMeters);
			// TODO: ensure the rotation is correct
			LiveGraph.add("targetVel", targetPose.velocityMetersPerSecond);
			LiveGraph.add("targetVelx", targetPose.velocityMetersPerSecond * targetPose.poseMeters.getRotation().getCos());
			LiveGraph.add("targetVely", targetPose.velocityMetersPerSecond * targetPose.poseMeters.getRotation().getSin());
			LiveGraph.add("timer/translation", Math.min((timer.get() / translationTimeout), 1.0));
			double zAngle = currentClosestTransform.getRotation().getZ();
			double currentFinalAngle = Math.toDegrees(Math.signum(zAngle) * ((Math.PI) - Math.abs(zAngle)));
			LiveGraph.add("currentToTurn", finalAngle + originalAngle);
			LiveGraph.add("detectedToTurn", currentFinalAngle + state.robotPoseMeters.getRotation().getDegrees());
			/* if(Math.abs((finalAngle + originalAngle) - (state.robotPoseMeters.getRotation().getDegrees() + currentFinalAngle)) > 3.0){
					finalAngle = currentFinalAngle;
					double timeLeft = translationTimeout - timer.get();
					double delta = (finalAngle)/(timeLeft);
					originalAngle = state.robotPoseMeters.getRotation().getDegrees() - delta * (translationTimeout - timeLeft);
				}*/
			double rotation = finalAngle * Math.min((timer.get() / translationTimeout), 1.0);

			LiveGraph.add("finalAngle", finalAngle);
			rotation += originalAngle;
			LiveGraph.add("rotation", rotation);
			ChassisSpeeds adjustedSpeeds;
			if (timer.get() <= translationTimeout) {
				adjustedSpeeds = controller.calculate(
						state.robotPoseMeters,
						targetPose,
						Rotation2d.fromDegrees(rotation));
				adjustedSpeeds = new ChassisSpeeds(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond);
				//adjustedSpeeds = new ChassisSpeeds(0.0,0,0);
			} else {
				if (pidResetVisionCount < 5) {
					double posDiffX = offsetX - currentClosestTransform.getY();
					double posDiffY = offsetY - currentClosestTransform.getX();
					pidResetVisionCount += 1;

					if (Math.abs(state.robotPoseMeters.getTranslation().minus(translation).getNorm()) < 0.5) {
						pidPose = new Pose2d(translation.getX(), translation.getY(), state.robotPoseMeters.getRotation());
						pidResetVisionCount += 1;
					} else if (pidPose == null) {
						pidPose = state.robotPoseMeters;
					}
					//originalAngle = state.robotPoseMeters.getRotation().getDegrees();
					//finalAngle = Math.toDegrees(Math.signum(zAngle) * ((Math.PI) - Math.abs(zAngle)));
				}
				LiveGraph.add("piodPose", pidPose);
				adjustedSpeeds = controllerPID.calculate(
						state.robotPoseMeters,
						pidPose,
						0.0,
						Rotation2d.fromDegrees(rotation));
				//finalAngle += commands.getDriveWantedRotation() * 0.3;
				Translation2d translationNew = new Translation2d(
						-commands.getDriveWantedTranslation(),
						-commands.getDriveWantedStrafe()).times(DriveConstants.maxSpeed * 0.25);
				double rot = commands.getDriveWantedRotation();
				rot = rot * rot * Math.signum(rot);
				rot *= DriveConstants.maxAngularVelocity * 0.25;
				LiveGraph.add("Drive/wantedRotation", rot);
				LiveGraph.add("Drive/Field relative", commands.isDriveFieldRelativeWanted());
				adjustedSpeeds = DriveConstants.swerveKinematics.toChassisSpeeds(drive(
						translationNew, rot,
						commands.isDriveFieldRelativeWanted(),
						state.robotPoseMeters.getRotation()));
				// if(Math.abs(additional.vxMetersPerSecond) > 0.01 || Math.abs(additional.vyMetersPerSecond) > 0.01|| Math.abs(additional.omegaRadiansPerSecond) > 0.01){
				// adjustedSpeeds = additional;
				// }else{
				//    }
				// adjustedSpeeds = new ChassisSpeeds(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, -adjustedSpeeds.omegaRadiansPerSecond);
				//ChassisSpeeds newSpeeds = controller.calculate(state.robotPoseMeters, pidPose, 0.0, state.robotPoseMeters.getRotation())
				//double velocity = Math.abs(controller2.calculate(dist));
				//System.out.println(velocity);
				//LiveGraph.add("pid velocity", velocity);
				//adjustedSpeeds = new ChassisSpeeds(-velocity * translationDirection.getY(), -velocity * translationDirection.getX(), -adjustedSpeeds.omegaRadiansPerSecond);
				//adjustedSpeeds = new ChassisSpeeds(additional.vxMetersPerSecond, additional.vyMetersPerSecond, -adjustedSpeeds.omegaRadiansPerSecond);
			}
			LiveGraph.add("adjusted vx", adjustedSpeeds.vxMetersPerSecond);
			LiveGraph.add("adjusted vyy", adjustedSpeeds.vyMetersPerSecond);
			LiveGraph.add("adjusted ange", adjustedSpeeds.omegaRadiansPerSecond);
			SwerveModuleState[] swerveModuleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(adjustedSpeeds);
			LiveGraph.add("Vision/ModuleStatesGenerated/1", swerveModuleStates[0].speedMetersPerSecond);
			LiveGraph.add("Vision/ModuleStatesGenerated/2", swerveModuleStates[1].speedMetersPerSecond);
			LiveGraph.add("Vision/ModuleStatesGenerated/3", swerveModuleStates[2].speedMetersPerSecond);
			LiveGraph.add("Vision/ModuleStatesGenerated/4", swerveModuleStates[3].speedMetersPerSecond);
			//SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 0.5);
			outputs.swerveModuleStates = swerveModuleStates;
			//SwerveModuleState[] states = drive(new Translation2d(), 0, true, state.robotPoseMeters.getRotation());
			//outputs.swerveModuleStates = states;
			outputs.isOpenLoop = false;
		} else if (timer.get() <= translationTimeout) {
			SwerveModuleState[] states = drive(new Translation2d(), 0, true, state.robotPoseMeters.getRotation());
			outputs.swerveModuleStates = states;
		} else {
			Translation2d translationNew = new Translation2d(
					-commands.getDriveWantedTranslation(),
					-commands.getDriveWantedStrafe()).times(DriveConstants.maxSpeed * 0.3);
			double rot = commands.getDriveWantedRotation();
			rot = rot * rot * Math.signum(rot);
			rot *= DriveConstants.maxAngularVelocity * 0.4;
			LiveGraph.add("Drive/wantedRotation", rot);
			LiveGraph.add("Drive/Field relative", commands.isDriveFieldRelativeWanted());
			outputs.swerveModuleStates = (drive(
					translationNew, rot,
					commands.isDriveFieldRelativeWanted(),
					state.robotPoseMeters.getRotation()));
		}

		// translation = translation.times(out);

		//SwerveModuleState[] states = drive(new Translation2d(0, 0), 0, true, state.robotPoseMeters.getRotation());
		//  outputs.swerveModuleStates = states;
		//outputs.isOpenLoop = commands.isDriveOpenLoopWanted();
	}

	public Trajectory generateTrajectory(Pose2d finalPose, Pose2d startPose, double currentVelocity) {
		var posesWithStart = new ArrayList<Pose2d>();
		posesWithStart.add(startPose);
		posesWithStart.add(finalPose);
		var trajectoryConfig = DriveConstants.getTrajectoryConfig(0.6, 2.0);
		trajectoryConfig.addConstraint(new CentripetalAccelerationConstraint(1.6));
		trajectoryConfig.setStartVelocity(currentVelocity);
		trajectoryConfig.setEndVelocity(0.0);
		Trajectory traj = TrajectoryGenerator.generateTrajectory(posesWithStart, trajectoryConfig);
		return traj;
	}

	private SwerveModuleState[] drive(Translation2d translation, double rotation, boolean fieldRelative, Rotation2d yaw) {
		SwerveModuleState[] swerveModuleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(
				fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
						translation.getX(),
						translation.getY(),
						rotation,
						yaw) :
						new ChassisSpeeds(
								translation.getX(),
								translation.getY(),
								rotation));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.maxSpeed);

		return swerveModuleStates;
	}
}
