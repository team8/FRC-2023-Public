package frc2023.config.subsystem;

import frc2023.util.config.ConfigBase;

public class ArmConfig extends ConfigBase {

	public double armFirstStagePistonExtensionTime;

	public double armSecondStageMaxAcceleration;
	public double armSecondStageMaxVelocity;

	public double armMasterKf;
	public double armMasterKp;
	public double armMasterKi;
	public double armMasterKd;

	public double motionMagicAcceleration;
	public double motionMagicVelocity;
}
