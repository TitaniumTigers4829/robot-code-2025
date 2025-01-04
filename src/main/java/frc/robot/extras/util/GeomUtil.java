package frc.robot.extras.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.dyn4j.geometry.Rotation;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;

/** utils to convert between WPILIB and dyn4j geometry classes */
public class GeomUtil {
  public static Vector2 toDyn4jVector2(Translation2d wpilibTranslation2d) {
    return new Vector2(wpilibTranslation2d.getX(), wpilibTranslation2d.getY());
  }

  public static Translation2d toWpilibTranslation2d(Vector2 dyn4jVector2) {
    return new Translation2d(dyn4jVector2.x, dyn4jVector2.y);
  }

  public static Rotation toDyn4jRotation(Rotation2d wpilibRotation2d) {
    return new Rotation(wpilibRotation2d.getRadians());
  }

  public static Rotation2d toWpilibRotation2d(Rotation dyn4jRotation) {
    return new Rotation2d(dyn4jRotation.toRadians());
  }

  public static Transform toDyn4jTransform(Pose2d wpilibPose2d) {
    final Transform transform = new Transform();
    transform.setTranslation(toDyn4jVector2(wpilibPose2d.getTranslation()));
    transform.setRotation(toDyn4jRotation(wpilibPose2d.getRotation()));
    return transform;
  }

  public static Pose2d toWpilibPose2d(Transform dyn4jTransform) {
    return new Pose2d(
        toWpilibTranslation2d(dyn4jTransform.getTranslation()),
        toWpilibRotation2d(dyn4jTransform.getRotation()));
  }

  public static Vector2 toDyn4jLinearVelocity(ChassisSpeeds wpilibChassisSpeeds) {
    return new Vector2(
        wpilibChassisSpeeds.vxMetersPerSecond, wpilibChassisSpeeds.vyMetersPerSecond);
  }

  public static ChassisSpeeds toWpilibChassisSpeeds(
      Vector2 dyn4jLinearVelocity, double angularVelocityRadPerSec) {
    return new ChassisSpeeds(
        dyn4jLinearVelocity.x, dyn4jLinearVelocity.y, angularVelocityRadPerSec);
  }

  /**
   * Gets the x and y velocities of a ChassisSpeeds
   *
   * @param chassisSpeeds the ChassisSpeeds to retrieve velocities from
   * @return a Translation2d containing the velocities in the x and y direction in meters per second
   */
  public static Translation2d getChassisSpeedsTranslationalComponent(ChassisSpeeds chassisSpeeds) {
    return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
  }

  /**
   * Checks if two translations are within a certain threshold in meters
   *
   * @param t1 Translation 1 as a Translation2d
   * @param t2 Translation 2 as a Translation2d
   * @param thresholdMeters the threshold between the two translations in meters
   * @return true if the two translations are within thresholdMeters
   */
  public static boolean isTranslationWithinThreshold(
      Translation2d t1, Translation2d t2, double thresholdMeters) {
    return t1.getDistance(t2) <= thresholdMeters;
  }

  /**
   * Checks if two rotations are within a certain threshold in degrees
   *
   * @param r1 Rotations 1 as a Rotation2d
   * @param r2 Rotation 2 as a Rotation2d
   * @param thresholdMeters the threshold between the two rotations in degrees
   * @return true if the two rotations are within thresholdDegrees
   */
  public static boolean isRotationWithinThreshold(double r1, double r2, double thresholdDegrees) {
    return Math.abs(r1 - r2) <= thresholdDegrees;
  }
}
