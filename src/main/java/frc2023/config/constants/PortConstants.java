package frc2023.config.constants;

@SuppressWarnings ("squid:ClassVariableVisibilityCheck")
public class PortConstants {

	public static final int driverXbox = 0;
	public static final int operatorXbox = 1;

	public static final int driveGyroID = 22; //2
	/* SWERVE */
	public static final int driveFLDriveID = 1; //5
	public static final int driveFRDriveID = 7; //22
	public static final int driveBLDriveID = 3; //9
	public static final int driveBRDriveID = 5; //11

	public static final int driveFLTurnID = 0; //10
	public static final int driveFRTurnID = 6; //4
	public static final int driveBLTurnID = 2; //15
	public static final int driveBRTurnID = 4; //0
	public static final int driveFLEncoderID = 17; //21
	public static final int driveFREncoderID = 20; //1
	public static final int driveBLEncoderID = 19; //6
	public static final int driveBREncoderID = 18; //8

	/* Arm */
	public static final int armSecondStageMotorMasterID = 8;
	public static final int armSecondStageMotorSlaveID = 15;
	public static final int armFirstStageSolenoidID = 1;
	public static final int armSecondStageEncoderID = 0;

	/* Pivot */
	public static final int pivotMotorID = 11;
	public static final int pivotPotentiometerID = 1;

	/* Intake */
	public static final int intakeRollerID = 12;

	/* LIGHTING */
	public static final int lightingPwmPort = 0;
}
