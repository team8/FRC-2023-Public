package frc2023.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Flippy {

	public static double fieldLengthMeters = 16.5452703; //TODO: FIX THIS TO BE ACCURATE

	/**
	 * Flips the translation and rotation (mirrored across midline) as a reflection.
	 *
	 * @param  pose2d pose2d with rotation to flip
	 * @return        flipped translation and rotation
	 */
	public static Pose2d flipTranslation(Pose2d pose2d) {
		return new Pose2d(new Translation2d(fieldLengthMeters - pose2d.getX(), pose2d.getY()), flipRotation(pose2d.getRotation()));
	}

	/**
	 * FLips rotation vector by mirroring over midline.
	 *
	 * @param  rotation Rotation to flip
	 * @return          flipped rotation
	 */
	public static Rotation2d flipRotation(Rotation2d rotation) {
		return new Rotation2d(-rotation.getCos(), rotation.getSin());
	}
}
