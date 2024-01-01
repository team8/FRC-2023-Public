package frc2023.config.constants;

import edu.wpi.first.math.util.Units;

public class ArmConstants {

	public static final double armFirstStageStowedAngle = 50.98227905;
	public static final double armFirstStageExtendedAngle = 93.6140136;

	public static final double armBeamCenterOfMassFromPivotMetersX = Units.inchesToMeters(18.55026798);
	public static final double armBeamCenterOfMassFromPivotMetersY = Units.inchesToMeters(0.15288321);
	public static final double armBeamMassKilograms = Units.lbsToKilograms(6.6083);

	public static final double armIntakeCenterOfMassFromIntakePivotMetersX = Units.inchesToMeters(12);
	public static final double armIntakeCenterOfMassFromIntakePivotMetersY = Units.inchesToMeters(0);
	public static final double armIntakeMassKilograms = Units.lbsToKilograms(5.5);
	public static final double armDistanceToIntakePivotMeters = Units.inchesToMeters(39.33230494);
	public static final double armSecondStageGearRatio = (1.0 / 182.25);
	public static final double armSecondStagePositionConversionRatio = (1.0 / 182.25) * 360.0;
	public static final double armSecondStageVelocityConversionRatio = (1.0 / 182.25) * (1 / 60.0) * 360.0; //test
	public static final double neoStallTorque = 3.75;

	public static final double degreesToFalconTick = (1 / 360.0) * 182.25 * 2048;
}
