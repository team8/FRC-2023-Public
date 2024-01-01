package frc2023.robot;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.photonvision.targeting.PhotonTrackedTarget;

import frc2023.config.constants.DriveConstants;
import frc2023.util.LiveGraph;
import frc2023.util.Util;

/**
 * Holds the current physical state of the robot from our sensors.
 */
@SuppressWarnings ("squid:ClassVariableVisibilityCheck")
public class RobotState {

	public enum GamePeriod {
		AUTO, TELEOP, TESTING, DISABLED
	}

	public RobotState(Pose2d initialPosition) {
		this.robotPoseMeters = initialPosition;
		this.swervePoseMeters = initialPosition;

		for (int i = 0; i < 4; i++) {
			driveSwerveModuleStates[i] = new SwerveModuleState();
			driveModulePositions[i] = new SwerveModulePosition();
		}

		robotPoseEstimator = new SwerveDrivePoseEstimator(
				DriveConstants.swerveKinematics, driveYaw,
				driveModulePositions, robotPoseMeters,
				new Matrix<>(VecBuilder.fill(0.01, 0.01, 0.01)),
				new Matrix<>(VecBuilder.fill(0.1, 0.1, 0.1)));

		swervePoseEstimator = new SwerveDrivePoseEstimator(
				DriveConstants.swerveKinematics, driveYaw,
				driveModulePositions, swervePoseMeters,
				new Matrix<>(VecBuilder.fill(0.01, 0.01, 0.01)),
				new Matrix<>(VecBuilder.fill(0, 0, 0)));
	}

	public static final String kLoggerTag = Util.classToJsonName(RobotState.class);

	/* Robot */
	public Pose2d robotPoseMeters;
	public Pose2d swervePoseMeters;
	public SwerveDrivePoseEstimator robotPoseEstimator;
	public SwerveDrivePoseEstimator swervePoseEstimator;
	public Pose2d swerveDriveOdometry;
	public boolean simulated = false;

	/* Drive */
	public SwerveModuleState[] driveSwerveModuleStates = new SwerveModuleState[4];
	public SwerveModulePosition[] driveModulePositions = new SwerveModulePosition[4];
	public double[] driveGyroAngles = new double[3],
			driveAngularVelocities = new double[3];
	public Rotation2d driveYaw = new Rotation2d(),
			driveYawAngularVelocity = new Rotation2d(),
			drivePitch = new Rotation2d(),
			driveRoll = new Rotation2d();
	public boolean driveIsGyroReady = false;
	public Translation2d routineWantedOffset = new Translation2d(0, 0);
	public double driveVelocity;

	/* Vision */
	public boolean visionHasTapeTargets;
	public List<PhotonTrackedTarget> visionTapeTargets;
	public int visionFoundTargetID;
	public Pose2d visionPose = new Pose2d();
	public boolean haveVision;
	public long tagTarget;
	public boolean drivePathFollowing = false;
	public boolean useLeftCamera = false;

	public boolean useAutoVision = true;

	public double aprilTagAmbiguity;
	public Optional<Transform3d> visionCameraTargetPose = Optional.empty();
	public Optional<Transform3d> visionCameraTargetPoseBad = Optional.empty();
	public Optional<Double> yawToTarget = Optional.empty();

	/* Arm */
	public double armFirstStageAngleDegrees;
	public double armSecondStageAngleDegrees;
	public double armSecondStageVelocity;
	public double armSecondStageAbsoluteAngleDegrees;

	/* Pivot */
	public double pivotCurrentPotentiometerPosition;
	public double pivotCurrentAngleDegrees;

	/* Intake */
	public double intakeRollerVelocity;
	public double intakeX, intakeY, intakeW, intakeH;

	/* Game and Field */
	public GamePeriod gamePeriod = GamePeriod.DISABLED;
	public String gameData;

	public enum Piece {
		CUBE, CONE
	}

	public Piece heldPiece;

	public void calculations() {
		this.robotPoseMeters = robotPoseEstimator.getEstimatedPosition();
		this.swervePoseMeters = swervePoseEstimator.getEstimatedPosition();
	}

	public void log() {
		/* Robot */
		LiveGraph.add("Robot/robotPoseMeters", robotPoseMeters);
		LiveGraph.add("Robot/swervePoseMeters", swervePoseMeters);

		/* Drive */
		LiveGraph.add("Drive/driveModuleStates", driveSwerveModuleStates);
		LiveGraph.add("Drive/driveGyroAngle", driveYaw);
	}
}
