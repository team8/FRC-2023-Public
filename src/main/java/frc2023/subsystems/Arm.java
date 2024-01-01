package frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.*;

import frc2023.config.constants.ArmConstants;
import frc2023.config.constants.PortConstants;
import frc2023.config.subsystem.ArmConfig;
import frc2023.robot.Commands;
import frc2023.robot.RobotState;
import frc2023.util.LiveGraph;
import frc2023.util.config.Configs;
import frc2023.util.control.Falcon;
import frc2023.util.control.TimedSolenoid;

public class Arm extends SubsystemBase {

	double zeroPos = 0.0;
	private static Arm INSTANCE = new Arm();
	private boolean mFirstStageExtended = false;
	private ArmConfig mConfig = Configs.get(ArmConfig.class);

	private Falcon secondStageMaster = new Falcon(PortConstants.armSecondStageMotorSlaveID, "Arm Second Stage Master");
	private TimedSolenoid firstStageSolenoid = new TimedSolenoid(21, PneumaticsModuleType.CTREPCM, PortConstants.armFirstStageSolenoidID, mConfig.armFirstStagePistonExtensionTime, true);
	private DutyCycleEncoder throughBore = new DutyCycleEncoder(2);
	private Encoder relativeEncoder = new Encoder(0, 1);
	Compressor compressor = new Compressor(21, PneumaticsModuleType.CTREPCM);
	private double mSecondStageWantedPercentOutput = 0.0;
	private boolean useMotionMagic = false;
	private double motionMagicPosition = 0.0;

	public enum SecondStageState {
		NEUTRAL, STATIONARY, TRAJECTORY, RE_ZERO, TEMP, POSITION, PERCENT_OUTPUT
	}

	public enum FirstStageState {
		LOWER, RAISED
	}

	private Arm() {
		throughBore.setDistancePerRotation(360.0 / 4.5);
		relativeEncoder.setDistancePerPulse(360.0 / (4.5 * 1024.0));
	}

	@Override
	public void update(Commands commands, RobotState state) {
		LiveGraph.add("Arm/wantedPercentOutput", commands.armWantedPercentOutput);
		compressor.enableDigital();

		useMotionMagic = false;

		state.armSecondStageAngleDegrees = ((relativeEncoder.get() * 360.0) / (2048.0 * 4.5)) + zeroPos;

		LiveGraph.add("Arm/wantedAngleDeg", commands.wantedArmPosition);

		switch (commands.wantedArmSecondStageState) {
			case TEMP:
				break;
			case NEUTRAL:
				mSecondStageWantedPercentOutput = 0.0;
				break;
			case TRAJECTORY:
				break;
			case POSITION:
				useMotionMagic = true;
				motionMagicPosition = commands.wantedArmPosition * ArmConstants.degreesToFalconTick;
//                motionMagicPosition = commands.wantedArmPosition;
				break;
			case PERCENT_OUTPUT:
				mSecondStageWantedPercentOutput = commands.armWantedPercentOutput;
				break;
			case STATIONARY:
				useMotionMagic = true;
				motionMagicPosition = secondStageMaster.getSelectedSensorPosition();
//                motionMagicPosition = state.armSecondStageAngleDegrees * ArmConstants.degreesToFalconTick;
				break;
			case RE_ZERO:
				// TODO: check if state.armSeconStageAngleDegrees or state.armAbsolutetSecondStageAngleDegrees should be used
				state.armSecondStageAbsoluteAngleDegrees = getAbsPosFromThroughBoreReading(throughBore.getAbsolutePosition());
				secondStageMaster.setSelectedSensorPosition(state.armSecondStageAngleDegrees * ArmConstants.degreesToFalconTick);
				zeroPos = state.armSecondStageAbsoluteAngleDegrees;
				relativeEncoder.reset();
				//commands.wantedArmSecondStageState = SecondStageState.STATIONARY;
				commands.wantedArmSecondStageState = SecondStageState.NEUTRAL;
				break;
		}
	}

	@Override
	public void writeHardware(RobotState state) {

		firstStageSolenoid.set(mFirstStageExtended);
		LiveGraph.add("Arm/useMotionMagic", useMotionMagic);
		LiveGraph.add("Arm/currentMotionMagic", state.armSecondStageAngleDegrees * ArmConstants.degreesToFalconTick);

		if (useMotionMagic) {
			LiveGraph.add("Arm/wantedMotionMagic", motionMagicPosition);
			double maxGravityFF = 0.05;
			// arm is offset by 90 deg
			double angle = Math.toRadians(90 + state.armSecondStageAngleDegrees);
			double gravityFF = maxGravityFF * Math.cos(angle);
			secondStageMaster.set(ControlMode.MotionMagic, motionMagicPosition, DemandType.ArbitraryFeedForward, gravityFF);
		} else {
			secondStageMaster.set(ControlMode.PercentOutput, mSecondStageWantedPercentOutput);
		}
	}

	@Override
	public void readHardware(RobotState state) {

		if (mFirstStageExtended) {
			state.armFirstStageAngleDegrees = ArmConstants.armFirstStageStowedAngle;
		} else {
			state.armFirstStageAngleDegrees = ArmConstants.armFirstStageExtendedAngle;
		}

		LiveGraph.add("Arm/relativeAngle", state.armSecondStageAngleDegrees);
		LiveGraph.add("Arm/firstStage", state.armFirstStageAngleDegrees);
		LiveGraph.add("Arm/realabs", throughBore.getAbsolutePosition());
		LiveGraph.add("Arm/Velocity", secondStageMaster.getSelectedSensorVelocity());
		LiveGraph.add("Arm/relativeEncoder", ((relativeEncoder.get() * 360.0) / (2048.0 * 4.5)) + zeroPos);
		LiveGraph.add("Arm/absoluteArm", state.armSecondStageAbsoluteAngleDegrees);
		LiveGraph.add("Arm/RAW ABSOLUTE", throughBore.getDistance());
		LiveGraph.add("arm/pos", secondStageMaster.getSelectedSensorPosition());
		LiveGraph.add("Arm/something", throughBore.get());
	}

	@Override
	public void configureHardware() {
		secondStageMaster.configFactoryDefault();
//        secondStageSlave.configFactoryDefault();
		secondStageMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
		secondStageMaster.config_kF(0, mConfig.armMasterKf, 20);
		secondStageMaster.config_kP(0, mConfig.armMasterKp, 20);
		secondStageMaster.config_kI(0, mConfig.armMasterKi, 20);
		secondStageMaster.config_kD(0, mConfig.armMasterKd, 20);
		secondStageMaster.setNeutralMode(NeutralMode.Brake);

		secondStageMaster.configMotionCruiseVelocity(mConfig.motionMagicVelocity, 20); //needs to be tuned to robot
		secondStageMaster.configMotionAcceleration(mConfig.motionMagicAcceleration, 20);
		secondStageMaster.setInverted(true);

		secondStageMaster.configSensorConversions(
				ArmConstants.armSecondStagePositionConversionRatio,
				ArmConstants.armSecondStageVelocityConversionRatio);
	}

	public void setBrakeMode(boolean brake) {
		if (brake) {
			secondStageMaster.setNeutralMode(NeutralMode.Brake);
		} else {
			secondStageMaster.setNeutralMode(NeutralMode.Coast);
		}
	}

	private double getAbsPosFromThroughBoreReading(double reading) {
		// LOOK AT REAL ABS ON LIVEGRAPH
		double absPosAtRest = 0.310;
		double absPosAtNinety = 0.18530412963260323;
		absPosAtNinety = absPosAtRest - 0.121903628;
		reading = 1 - reading;
		reading += absPosAtNinety;
		reading = reading * (360.0 / 4.5);
		return (-90.0 - reading);
	}

	public static Arm getInstance() {
		return INSTANCE;
	}
}
