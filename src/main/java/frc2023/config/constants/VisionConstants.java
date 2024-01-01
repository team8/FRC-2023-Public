package frc2023.config.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {

	public static final String aprilTagCameraName = "OV5647";
	public static final String tapeCameraName = "tape";
	public static final String fieldName = "2023-chargedup.json";

	public static final Transform3d robotToAprilCam = new Transform3d(
			new Translation3d(7, 0.0, 32.75),
			new Rotation3d(0, Math.toRadians(55), 0));

	public static final Transform3d robotToTapeCam = new Transform3d(
			new Translation3d(0.5, 0.0, 0.5),
			new Rotation3d(0, 0, 0));

}
