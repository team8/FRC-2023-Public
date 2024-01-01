package frc2023.subsystems;

import static frc2023.config.constants.DriveConstants.*;
import static frc2023.robot.SimulatedRobot.lastModuleState;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.*;

import frc2023.config.constants.PortConstants;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.robot.SimulatedRobot;
import frc2023.subsystems.controllers.drive.*;
import frc2023.subsystems.swerve.SimulatedSwerveModule;
import frc2023.subsystems.swerve.SwerveModule;
import frc2023.util.LiveGraph;
import frc2023.util.control.SwerveDriveOutputs;

public class Drive extends SubsystemBase {

	public enum State {
		NEUTRAL, TELEOP, PATH_FOLLOWING, VISION_ALIGN, VISION_ALIGN_TRAJ
	}

	public static abstract class DriveController {

		protected SwerveDriveOutputs outputs = new SwerveDriveOutputs();

		public final SwerveDriveOutputs update(Commands commands, RobotState state) {
			updateSignal(commands, state);
			return outputs;
		}

		public abstract void updateSignal(Commands commands, RobotState state);
	}

	private Drive() {
	}

	private static final Drive INSTANCE = new Drive();

	private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

	private DriveController controller = new IdleController();
	private State state = State.NEUTRAL;
	// front left, front right, back left, back right
	private SwerveModule[] swerveModules = new SwerveModule[] { new SwerveModule(FLConfig), new SwerveModule(FRConfig), new SwerveModule(BLConfig), new SwerveModule(BRConfig)
	};

	public SwerveModule[] getSwerveModules() {
		return swerveModules;
	}

	private Pigeon2 gyro = new Pigeon2(PortConstants.driveGyroID);

	private SwerveDriveOdometry mOdometry;

	@Override
	public void update(Commands commands, RobotState state) {
		double[] gyroAngles = new double[3];
		gyro.getYawPitchRoll(gyroAngles);
		Rotation2d driveYaw = Rotation2d.fromDegrees(gyroAngles[0]);

		// Update the pose
		state.swerveDriveOdometry = mOdometry.update(driveYaw,
				new SwerveModulePosition[] { swerveModules[0].getPosition(), swerveModules[1].getPosition(), swerveModules[2].getPosition(), swerveModules[3].getPosition()
				});

		SwerveDriveOutputs outputs = controller.outputs;
		State wantedState = commands.getDriveWantedState();
		boolean isNewState = this.state != wantedState;
		this.state = wantedState;
		if (isNewState) {
			switch (wantedState) {
				case TELEOP:
					controller = new TeleopDriveController();
					break;
				case PATH_FOLLOWING:
					controller = new PPFollowingController();
					break;
				case VISION_ALIGN:
					controller = new VisionPathFollowingController();
					break;
				case VISION_ALIGN_TRAJ:
					controller = new VisionTrajectoryController();
					break;
				case NEUTRAL:
				default:
					controller = new IdleController();
			}
		}

		outputs = controller.update(commands, state);
		for (int i = 0; i < 4; i++) {
			LiveGraph.add("Teleop/ModuleStates/" + Integer.toString(i), state.driveSwerveModuleStates[i].speedMetersPerSecond);
			outputs.swerveModuleStates[i] = swerveModules[i].setDesiredState(state, outputs.swerveModuleStates[i], outputs.isOpenLoop);
		}
		LiveGraph.add("Drive/swerveWantedStates", outputs.swerveModuleStates);

		if (commands.shouldResetOdometry) {
			//state.robotPoseMeters = commands.resetOdometry;
			mOdometry.resetPosition(driveYaw,
					new SwerveModulePosition[] { swerveModules[0].getPosition(), swerveModules[1].getPosition(), swerveModules[2].getPosition(), swerveModules[3].getPosition()
					}, commands.resetOdometry);
			state.swervePoseMeters = commands.resetOdometry;
			//state.robotPoseEstimator.resetPosition(state.driveYaw, state.driveModulePositions, commands.resetOdometry);
			state.swervePoseEstimator.resetPosition(state.driveYaw, state.driveModulePositions, commands.resetOdometry);
		}
		LiveGraph.add("Robot/swerveodometry", state.swerveDriveOdometry);
	}

	@Override
	public void logSubsystem(RobotState state) {
		var pose = state.robotPoseMeters;
		var swerveModuleStates = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) {
			swerveModuleStates[i] = swerveModules[i].logModule();
		}
		LiveGraph.add("Drive/swerveModuleAbsoluteStates", swerveModuleStates);
	}

	@Override
	public void writeHardware(RobotState state) {
		for (var module : swerveModules) {
			module.writeHardware();
		}
	}

	@Override
	public void readHardware(RobotState state) {
		for (int i = 0; i < 4; i++) {
			state.driveSwerveModuleStates[i] = swerveModules[i].getState();
			state.driveModulePositions[i] = swerveModules[i].getPosition();
		}

		state.driveIsGyroReady = gyro.getUpTime() > 0;
		double[] gyroAngles = new double[3], gyroAngularVelocities = new double[3];
		if (state.driveIsGyroReady) {
			gyro.getYawPitchRoll(gyroAngles);
			state.driveYaw = Rotation2d.fromDegrees(gyroAngles[0]);
			state.drivePitch = Rotation2d.fromDegrees(gyroAngles[1]);
			state.drivePitch = Rotation2d.fromDegrees(gyroAngles[2]);
			state.driveGyroAngles = gyroAngles;
			gyro.getRawGyro(gyroAngularVelocities);
			state.driveYawAngularVelocity = Rotation2d.fromDegrees(gyroAngularVelocities[2]);
			state.driveAngularVelocities = gyroAngularVelocities;
			LiveGraph.add("Drive/yawPitchRoll", gyroAngles);
			LiveGraph.add("Drive/driveYaw", state.driveYaw.getDegrees() % 360);
			LiveGraph.add("Drive/driveYawVelocity", state.driveYawAngularVelocity.getDegrees());

			// need to update with time because inside of update, MathShareStore.getTimestamp() throws an error
			// poseestimator.update() not working as of wpilib 2023.4.3
			state.robotPoseEstimator.updateWithTime(WPIUtilJNI.now() * 1.0e-6, state.driveYaw, state.driveModulePositions);
			state.swervePoseEstimator.updateWithTime(WPIUtilJNI.now() * 1.0e-6, state.driveYaw, state.driveModulePositions);
		}
	}

	@Override
	public void configureHardware() {
		for (var module : swerveModules) {
			module.configureHardware();
		}
		double[] gyroAngles = new double[3];
		gyro.getYawPitchRoll(gyroAngles);
		Rotation2d driveYaw = Rotation2d.fromDegrees(gyroAngles[0]);
		mOdometry = new SwerveDriveOdometry(
				kinematics, driveYaw,
				new SwerveModulePosition[] { swerveModules[0].getPosition(), swerveModules[1].getPosition(), swerveModules[2].getPosition(), swerveModules[3].getPosition()
				}, new Pose2d(0, 0, new Rotation2d())

		);
	}

	@Override
	public void simulationInit() {
		swerveModules = new SwerveModule[] { new SimulatedSwerveModule(FLConfig), new SimulatedSwerveModule(FRConfig), new SimulatedSwerveModule(BLConfig), new SimulatedSwerveModule(BRConfig)
		};
	}

	@Override
	public void simulate(RobotState state, Commands commands) {
		if (commands.shouldResetOdometry) {
			SimulatedRobot.odometry = commands.resetOdometry;
			state.robotPoseEstimator.resetPosition(state.driveYaw, state.driveModulePositions, commands.resetOdometry);
			state.swervePoseEstimator.resetPosition(state.driveYaw, state.driveModulePositions, commands.resetOdometry);
		}

		State wantedState = commands.getDriveWantedState();
		boolean isNewState = this.state != wantedState;
		this.state = wantedState;
		SwerveDriveOutputs outputs = controller.outputs;
		if (isNewState) {
			switch (wantedState) {
				case TELEOP:
					controller = new TeleopDriveController();
					break;
				case PATH_FOLLOWING:
					controller = new PPFollowingController();
					break;
				case VISION_ALIGN:
					controller = new VisionPathFollowingController();
					break;
				case VISION_ALIGN_TRAJ:
					controller = new VisionPathFollowingController();
					break;
				case NEUTRAL:
				default:
					controller = new IdleController();
			}
		}
		outputs = controller.update(commands, state);
		// updates swervemodules
		for (int i = 0; i < 4; i++) {
			outputs.swerveModuleStates[i] = swerveModules[i].setDesiredState(state, outputs.swerveModuleStates[i], outputs.isOpenLoop);
		}

		for (int i = 0; i < 4; i++) {
			state.driveSwerveModuleStates[i] = swerveModules[i].getState();
			state.driveModulePositions[i] = swerveModules[i].getPosition();
		}

		var pigeonSim = gyro.getSimCollection();

		SwerveModulePosition[] measuredStatesDiff = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) {
			measuredStatesDiff[i] = new SwerveModulePosition(
					state.driveModulePositions[i].distanceMeters - lastModuleState[i].distanceMeters,
					state.driveModulePositions[i].angle);
			lastModuleState[i] = state.driveModulePositions[i];
		}
		var twist = kinematics.toTwist2d(measuredStatesDiff);

		SimulatedRobot.odometry = SimulatedRobot.odometry.exp(twist);

		LiveGraph.add("Simulation/odometry", SimulatedRobot.odometry);

		pigeonSim.setRawHeading(SimulatedRobot.odometry.getRotation().getDegrees());
	}

	public static Drive getInstance() {
		return INSTANCE;
	}
}
