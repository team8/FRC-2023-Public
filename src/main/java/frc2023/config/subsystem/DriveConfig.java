package frc2023.config.subsystem;

import frc2023.util.config.ConfigBase;
import frc2023.util.control.Gains;

public class DriveConfig extends ConfigBase {

	public double driveKS, driveKV, driveKA;
	public double pathVelocityMetersPerSecond, pathAccelerationMetersPerSecondSquared;
	public Gains angleGains;
	public Gains driveGains;
}
