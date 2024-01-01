package frc2023.subsystems.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc2023.config.constants.DriveConstants;
import frc2023.config.subsystem.DriveConfig;
import frc2023.robot.RobotState;
import frc2023.util.LiveGraph;
import frc2023.util.config.Configs;
import frc2023.util.control.CTREModuleState;
import frc2023.util.control.Falcon;
import frc2023.util.math.Conversions;

public class SwerveModule {

	private DriveConfig configs = Configs.get(DriveConfig.class);

	private static int modules = 0;

	public static class SwerveModuleConfig {

		public int angleMotorID;
		public int driveMotorID;
		public int angleEncoderID;
		public boolean inverted;
		public boolean angleInverted;
		public double angleOffset;
		public boolean coast;

		public SwerveModuleConfig(int angleMotorID, int driveMotorID, int angleEncoderID, boolean inverted, boolean angleInverted, double angleOffset, boolean coast) {
			this.angleMotorID = angleMotorID;
			this.driveMotorID = driveMotorID;
			this.angleEncoderID = angleEncoderID;
			this.inverted = inverted;
			this.angleInverted = angleInverted;
			this.angleOffset = angleOffset;
			this.coast = coast;
		}
	}

	protected Falcon angleMotor;
	protected Falcon driveMotor;
	protected CANCoder angleEncoder;

	protected int moduleNumber = 0;
	protected boolean isOpenLoop;
	protected SwerveModuleState desiredState;
	protected double angle;
	protected double lastAngle;

	protected boolean inverted;
	protected boolean angleInverted;
	protected boolean coast;
	protected double angleOffset;

	private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(configs.driveKS, configs.driveKV, configs.driveKA);

	public SwerveModule(SwerveModuleConfig config) {
		this.inverted = config.inverted;
		this.angleInverted = config.angleInverted;
		this.angleOffset = config.angleOffset;
		this.coast = config.coast;
		angleMotor = new Falcon(config.angleMotorID, "Angle Motor", "Awesome Sauce");
		driveMotor = new Falcon(config.driveMotorID, "Drive Motor", "Awesome Sauce");
		angleEncoder = new CANCoder(config.angleEncoderID, "Awesome Sauce");
		this.moduleNumber = modules;
		modules++;
	}

	public SwerveModuleState setDesiredState(RobotState state, SwerveModuleState desiredState, boolean isOpenLoop) {
		this.desiredState = CTREModuleState.optimize(desiredState, state.driveSwerveModuleStates[moduleNumber].angle);
		this.isOpenLoop = isOpenLoop;

		this.angle = (Math.abs(this.desiredState.speedMetersPerSecond) <= (DriveConstants.maxSpeed * 0.01)) ?
				lastAngle :
				this.desiredState.angle.getDegrees();
		this.lastAngle = this.angle;
		return this.desiredState;
	}

	public SwerveModuleState logModule() {
		double velocity = Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), DriveConstants.wheelCircumference, DriveConstants.driveGearRatio);
		Rotation2d angle = Rotation2d.fromDegrees((angleEncoder.getAbsolutePosition() - angleOffset) % 360);

		LiveGraph.add("Drive/Swerve Module " + moduleNumber + "/absEncoderPosition",
				angleEncoder.getAbsolutePosition());
		LiveGraph.add("Drive/Swerve Module " + moduleNumber + "/encoderPosition",
				Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), DriveConstants.angleGearRatio));
		LiveGraph.add("Drive/Swerve Module " + moduleNumber + "/setPoint", this.angle);
		return new SwerveModuleState(velocity, angle);
	}

	public SwerveModuleState getState() {
		double velocity = Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), DriveConstants.wheelCircumference, DriveConstants.driveGearRatio);
		Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), DriveConstants.angleGearRatio));

		return new SwerveModuleState(velocity, angle);
	}

	public SwerveModulePosition getPosition() {
		double position = Conversions.falconToDegrees(driveMotor.getSelectedSensorPosition(), DriveConstants.driveGearRatio) / 360 * DriveConstants.wheelCircumference;
		Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), DriveConstants.angleGearRatio));
		return new SwerveModulePosition(position, angle);
	}

	public void writeHardware() {
		if (isOpenLoop) {
			double percentOutput = desiredState.speedMetersPerSecond / DriveConstants.maxSpeed;
			driveMotor.set(ControlMode.PercentOutput, percentOutput);
		} else {
			double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, DriveConstants.wheelCircumference, DriveConstants.driveGearRatio);
			LiveGraph.add("Teleop/VelocityFalcon/" + Integer.toString(moduleNumber), velocity);
			driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
		}
		angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, DriveConstants.angleGearRatio));
	}

	public void configureHardware() {
		/* Angle Motor Config */
		angleMotor.configFactoryDefault();
		angleMotor.configAllSettings(DriveConstants.swerveAngleFXConfig);
		angleMotor.setNeutralMode(NeutralMode.Coast);
		angleEncoder.configFactoryDefault();
		double absolutePosition = Conversions.degreesToFalcon(((angleEncoder.getAbsolutePosition()) - angleOffset) % 360,
				DriveConstants.angleGearRatio);
		var code = angleMotor.setSelectedSensorPosition(-absolutePosition);
		angleMotor.setInverted(angleInverted);
		if (code != ErrorCode.OK) {
			System.out.println("ERROR IN MODULE " + moduleNumber);
		} else {
			System.out.println("Configured module " + moduleNumber);
		}

		/* Drive Motor Config */
		driveMotor.configFactoryDefault();
		driveMotor.configAllSettings(DriveConstants.swerveDriveFXConfig);
		driveMotor.setInverted(inverted);
		driveMotor.setNeutralMode(coast ? NeutralMode.Coast : NeutralMode.Brake);
		driveMotor.setSelectedSensorPosition(0);

		lastAngle = 0;
	}
}
