package frc2023.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SimulatedRobot {

	public static Pose2d odometry = new Pose2d();
	public static SwerveModulePosition[] lastModuleState = new SwerveModulePosition[4];

	static {
		for (int i = 0; i < 4; i++) {
			lastModuleState[i] = new SwerveModulePosition();
		}
	}
}
