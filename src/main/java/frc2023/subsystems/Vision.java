package frc2023.subsystems;

import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc2023.config.constants.VisionConstants;
import frc2023.robot.Commands;
import frc2023.robot.Robot;
import frc2023.robot.RobotState;
import frc2023.robot.SimulatedRobot;
import frc2023.util.LiveGraph;

public class Vision extends SubsystemBase {

	private static final Vision INSTANCE = new Vision();

	private final PhotonCamera aprilTagCamera = new PhotonCamera(VisionConstants.aprilTagCameraName);
	private final PhotonCamera tapeCamera = new PhotonCamera(VisionConstants.tapeCameraName);
	private VisionLEDMode LEDState = VisionLEDMode.kOff;

	private final HashMap<Long, Pose3d> tags = new HashMap<>();

	//Forward Camera
	// testCamera -> leftCamera
	// other one -> rightCamera
	// z is left/right camera x is depth in robot
	boolean useLeft = false;
	private PhotonCamera leftCam = new PhotonCamera("newCamera");
	private PhotonCamera rightCam = new PhotonCamera("oldCamera");
	double leftCameraOffset = -0.25;
	double rightCameraOffset = 0.25;
	Transform3d robotToCamLeft = new Transform3d(new Translation3d(0.4 /* depth in robot */, leftCameraOffset /* camera offset */, 0.22/* vertical offset */), new Rotation3d(0, Math.PI / 12, Math.PI / 4 - 0.06));
	Transform3d robotToCamRight = new Transform3d(new Translation3d(0.4 /* depth in robot */, rightCameraOffset /* camera offset */, 0.22/* vertical offset */), new Rotation3d(0, Math.PI / 12, -Math.PI / 4 + 0.02));
	// Construct PhotonPoseEstimator
	AprilTagFieldLayout aprilTagFieldLayout;
	{
		try {
			String color = Robot.alliance == DriverStation.Alliance.Blue ? "blue" : "red";
			String fieldLayoutPath = Paths.get(Filesystem.getDeployDirectory().toString(), "2023-chargedup-" + color + ".json").toString();
			//TODO: implement this!!
			aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
			aprilTagFieldLayout = new AprilTagFieldLayout(fieldLayoutPath);
		} catch (Exception e) {
			System.out.println("NO FILE NO FILE NO FILE BAD BAD BAD !!! !!! NO FILE NO FILE !!! ");
			e.printStackTrace();
		}
	}

	PhotonPoseEstimator photonPoseEstimatorLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, leftCam, robotToCamLeft);
	PhotonPoseEstimator photonPoseEstimatorRight = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, rightCam, robotToCamRight);

	private Vision() {
	}

	@Override
	public void update(Commands commands, RobotState state) {
		boolean rightCamOverride = false;
		boolean leftCamOverride = false;

		Transform3d bestLeft = new Transform3d();
		Transform3d bestRight = new Transform3d();

		photonPoseEstimatorRight.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP);
		if (rightCam.hasTargets()) {
			PhotonPipelineResult latestResult = rightCam.getLatestResult();
			if (latestResult.getBestTarget() != null) {
				bestRight = latestResult.getBestTarget().getBestCameraToTarget();
			} else {
				bestRight = null;
			}
			List<PhotonTrackedTarget> res;
			res = latestResult.getTargets();
			if (res.size() == 2) {
				if ((res.get(0).getFiducialId() == 5 || res.get(1).getFiducialId() == 5)) {
					photonPoseEstimatorRight.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
					rightCamOverride = true;
				} else {
					photonPoseEstimatorRight.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP);
				}
			}
		}
		photonPoseEstimatorLeft.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP);
		if (leftCam.hasTargets()) {
			PhotonPipelineResult latestResult = leftCam.getLatestResult();
			if (latestResult.getBestTarget() != null) {
				bestLeft = latestResult.getBestTarget().getBestCameraToTarget();
			} else {
				bestLeft = null;
			}
			List<PhotonTrackedTarget> res;
			res = latestResult.getTargets();
			if (res.size() == 2) {
				if ((res.get(0).getFiducialId() == 4 || res.get(1).getFiducialId() == 4)) {
					photonPoseEstimatorLeft.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
					leftCamOverride = true;
				} else {
					photonPoseEstimatorLeft.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP);
				}

			}
		}
		Optional<EstimatedRobotPose> fetchedPoseLeft = photonPoseEstimatorLeft.update();
		Optional<EstimatedRobotPose> fetchedPoseRight = photonPoseEstimatorRight.update();
		Pose2d leftPose = null;
		Pose2d rightPose = null;
		Pose2d finalPose = null;

		boolean bothWorking = fetchedPoseRight.isPresent() && fetchedPoseLeft.isPresent() && !leftCamOverride && !rightCamOverride;

		// best tag distance
		double bestXDistance = Double.MAX_VALUE;
		double bestYDistance = Double.MAX_VALUE;

		if (fetchedPoseLeft.isPresent() && !leftCamOverride) {
			if (bestLeft != null) {
				double leastDistanceX = bestLeft.getX();
				double leastDistanceY = bestLeft.getY();
				if (leastDistanceX < bestXDistance) {
					bestXDistance = leastDistanceX;
				}
				if (leastDistanceY < bestYDistance) {
					bestYDistance = leastDistanceY;
				}
			}

			leftPose = fetchedPoseLeft.get().estimatedPose.toPose2d();
			if (!bothWorking) {
				finalPose = leftPose;
			}
			LiveGraph.add("Robot/leftCamPose", leftPose);
		}
		LiveGraph.add("Robot/leftCamPoseOn", leftCam.hasTargets());
		if (fetchedPoseRight.isPresent() && !rightCamOverride) {
			if (bestRight != null) {
				double leastDistanceX = bestRight.getX();
				double leastDistanceY = bestRight.getY();
				if (leastDistanceX < bestXDistance) {
					bestXDistance = leastDistanceX;
				}
				if (leastDistanceY < bestYDistance) {
					bestYDistance = leastDistanceY;
				}
			}

			rightPose = fetchedPoseRight.get().estimatedPose.toPose2d();
			if (!bothWorking) {
				finalPose = rightPose;
			}
			LiveGraph.add("Robot/rightCamPose", rightPose);
		}
		LiveGraph.add("Robot/rightCamPoseOn", rightCam.hasTargets());

		if (bothWorking) {
			double avgX = (rightPose.getX() + leftPose.getX()) / 2;
			double avgY = (rightPose.getY() + leftPose.getY()) / 2;
			double avgRotation = (Math.abs(rightPose.getRotation().getRadians()) + Math.abs(leftPose.getRotation().getRadians())) / 2;
			finalPose = new Pose2d(avgX, avgY, new Rotation2d(avgRotation));
		}

		if (finalPose != null) {
			Vector<N3> visionStdDevs = VecBuilder.fill(Math.pow(bestXDistance, 2), Math.pow(bestYDistance, 2), 0.01);
			state.robotPoseEstimator.addVisionMeasurement(finalPose, WPIUtilJNI.now() * 1.0e-6, visionStdDevs);
			state.haveVision = true;
		} else {
			state.haveVision = false;
		}

	}

	@Override
	public void writeHardware(RobotState state) {
		aprilTagCamera.setLED(VisionLEDMode.kOn);
	}

	@Override
	public void readHardware(RobotState state) {
		if (LEDState == VisionLEDMode.kOn) {
			var tapeResults = tapeCamera.getLatestResult();

			state.visionHasTapeTargets = tapeResults.hasTargets();
			state.visionTapeTargets = tapeResults.targets;
		}

	}

	@Override
	public void simulate(RobotState state, Commands commands) {
		state.tagTarget = 1;
		state.robotPoseMeters = SimulatedRobot.odometry;
		state.visionPose = state.robotPoseMeters;
		LiveGraph.add("Robot/visionPose", state.visionPose);

	}

	@Override
	public void logSubsystem(RobotState state) {
	}

	@Override
	public void configureHardware() {
		aprilTagCamera.setLED(VisionLEDMode.kOn);
	}

	public static Vision getInstance() {
		return INSTANCE;
	}
}
