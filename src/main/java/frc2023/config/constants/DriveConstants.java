package frc2023.config.constants;

import static frc2023.config.constants.PortConstants.*;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;

import frc2023.config.subsystem.DriveConfig;
import frc2023.subsystems.swerve.SwerveModule;
import frc2023.util.config.Configs;

public class DriveConstants {

	public static final double trackWidth = Units.inchesToMeters(21.73);
	public static final double wheelBase = Units.inchesToMeters(21.73);

	public static final double autoBalanceAcceptableError = 5;
	public static final double autoBalanceTimer = 5;

	public static Translation2d frontLeftLocation = new Translation2d((wheelBase / 2.0), trackWidth / 2.0);
	public static Translation2d frontRightLocation = new Translation2d((wheelBase / 2.0), -trackWidth / 2.0);
	public static Translation2d backLeftLocation = new Translation2d((-wheelBase / 2.0), trackWidth / 2.0);
	public static Translation2d backRightLocation = new Translation2d((-wheelBase / 2.0), -trackWidth / 2.0);

	public static double maxSpeed = 5.4, maxAcceleration = 0.4;
	public static final double wheelRadius = (Units.metersToInches(0.96) / Units.metersToInches(1) * Units.inchesToMeters(4.0)) / 2,
			wheelCircumference = wheelRadius * 2 * Math.PI,
			wheelDistance = 10;

	public static final double driveGearRatio = (6.12); //8.14:1
	public static final double angleGearRatio = ((150.0 / 7.0)); //21.4285714286:1
	public static final double maxAngularVelocity = 3.14 * 2;
	public static final TalonFXConfiguration swerveAngleFXConfig, swerveDriveFXConfig;
	public static SwerveDriveKinematics swerveKinematics;

	public static final SwerveModule.SwerveModuleConfig FLConfig = new SwerveModule.SwerveModuleConfig(
			driveFLTurnID, driveFLDriveID, driveFLEncoderID,
			false, true, 358.5 - 2.27, false),
			FRConfig = new SwerveModule.SwerveModuleConfig(
					driveFRTurnID, driveFRDriveID, driveFREncoderID,
					false, true, 351.211 + 180.72 - 180.36, false),
			BLConfig = new SwerveModule.SwerveModuleConfig(
					driveBLTurnID, driveBLDriveID, driveBLEncoderID,
					false, true, 5.537 + 176.326 - 357.02, false),
			BRConfig = new SwerveModule.SwerveModuleConfig(
					driveBRTurnID, driveBRDriveID, driveBREncoderID,
					false, true, 358.945 + 2.609 - 181.02 - 180, false);

	public static TrajectoryConfig getTrajectoryConfig(double maxPathVelocityMetersPerSecond, double maxPathAccelerationMetersPerSecondSquared) {
		return new TrajectoryConfig(maxPathVelocityMetersPerSecond, maxPathAccelerationMetersPerSecondSquared)
				.setKinematics(swerveKinematics);
	}

	static {
		var config = Configs.get(DriveConfig.class);
		swerveAngleFXConfig = new TalonFXConfiguration();
		swerveAngleFXConfig.slot0.kP = config.angleGains.p;
		swerveAngleFXConfig.slot0.kI = config.angleGains.i;
		swerveAngleFXConfig.slot0.kD = config.angleGains.d;
		swerveAngleFXConfig.slot0.kF = config.angleGains.f;
		swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

		swerveDriveFXConfig = new TalonFXConfiguration();
		swerveDriveFXConfig.slot0.kP = config.driveGains.p;
		swerveDriveFXConfig.slot0.kI = config.driveGains.i;
		swerveDriveFXConfig.slot0.kD = config.driveGains.d;
		swerveDriveFXConfig.slot0.kF = config.driveGains.f;
		swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

		swerveKinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
	}
}
