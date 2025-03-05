package frc.robot.extras.math.mathutils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Torque;
import frc.robot.extras.math.forces.Velocity2d;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Rotation;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;

/** utils to convert between WPILIB and dyn4j geometry classes */
public class GeomUtil {

  /**
   * Converts a WPILIB Translation2d to a dyn4j Vector2
   *
   * @param wpilibTranslation2d the Translation2d to convert
   * @return the equivalent Vector2
   */
  public static Vector2 toDyn4jVector2(Translation2d wpilibTranslation2d) {
    return new Vector2(wpilibTranslation2d.getX(), wpilibTranslation2d.getY());
  }

  /**
   * Converts a Velocity2d to a dyn4j Vector2
   *
   * @param velocity2d the Velocity2d to convert
   * @return the equivalent Vector2
   */
  public static Vector2 toDyn4jVector2(Velocity2d velocity2d) {
    return new Vector2(velocity2d.getVX(), velocity2d.getVY());
  }

  /**
   * Converts a dyn4j Vector2 to a WPILIB Translation2d
   *
   * @param dyn4jVector2 the Vector2 to convert
   * @return the equivalent Translation2d
   */
  public static Translation2d toWpilibTranslation2d(Vector2 dyn4jVector2) {
    return new Translation2d(dyn4jVector2.x, dyn4jVector2.y);
  }

  /**
   * Converts a WPILIB Rotation2d to a dyn4j Rotation
   *
   * @param wpilibRotation2d the Rotation2d to convert
   * @return the equivalent Rotation
   */
  public static Rotation toDyn4jRotation(Rotation2d wpilibRotation2d) {
    return new Rotation(wpilibRotation2d.getRadians());
  }

  /**
   * Converts a dyn4j Rotation to a WPILIB Rotation2d
   *
   * @param dyn4jRotation the Rotation to convert
   * @return the equivalent Rotation2d
   */
  public static Rotation2d toWpilibRotation2d(Rotation dyn4jRotation) {
    return new Rotation2d(dyn4jRotation.toRadians());
  }

  /**
   * Converts a WPILIB Pose2d to a dyn4j Transform
   *
   * @param wpilibPose2d the Pose2d to convert
   * @return the equivalent Transform
   */
  public static Transform toDyn4jTransform(Pose2d wpilibPose2d) {
    final Transform transform = new Transform();
    transform.setTranslation(toDyn4jVector2(wpilibPose2d.getTranslation()));
    transform.setRotation(toDyn4jRotation(wpilibPose2d.getRotation()));
    return transform;
  }

  /**
   * Converts a dyn4j Transform to a WPILIB Pose2d
   *
   * @param dyn4jTransform the Transform to convert
   * @return the equivalent Pose2d
   */
  public static Pose2d toWpilibPose2d(Transform dyn4jTransform) {
    return new Pose2d(
        toWpilibTranslation2d(dyn4jTransform.getTranslation()),
        toWpilibRotation2d(dyn4jTransform.getRotation()));
  }

  /**
   * Converts a WPILIB ChassisSpeeds to a dyn4j Vector2
   *
   * @param wpilibChassisSpeeds the ChassisSpeeds to convert
   * @return the equivalent Vector2
   */
  public static Vector2 toDyn4jLinearVelocity(ChassisSpeeds wpilibChassisSpeeds) {
    return new Vector2(
        wpilibChassisSpeeds.vxMetersPerSecond, wpilibChassisSpeeds.vyMetersPerSecond);
  }

  /**
   * Converts a WPILIB ChassisSpeeds to a dyn4j angular velocity in radians per second
   *
   * @param wpilibChassisSpeeds the ChassisSpeeds to convert
   * @return the equivalent angular velocity in radians per second
   */
  public static ChassisSpeeds toWpilibChassisSpeeds(
      Vector2 dyn4jLinearVelocity, double angularVelocityRadPerSec) {
    return new ChassisSpeeds(
        dyn4jLinearVelocity.x, dyn4jLinearVelocity.y, angularVelocityRadPerSec);
  }

  public static Rectangle toDyn4jRectangle(Rectangle2d wpilibRectangle) {
    return new Rectangle(wpilibRectangle.getXWidth(), wpilibRectangle.getYWidth());
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
   * Gets the x and y velocities of a ChassisSpeeds
   *
   * @param chassisSpeeds the ChassisSpeeds to retrieve velocities from
   * @return a Velocity2d containing the velocities in the x and y direction in meters per second
   */
  public static Velocity2d getChassisSpeedsVelocityComponent(ChassisSpeeds chassisSpeeds) {
    return new Velocity2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
  }

  /**
   * Gets a dyn4j torque value as a wpilib unit
   *
   * @param torque the dyn4j torque to convert
   * @return the converted torque value as a wpilib Unit of {@link Torque}.
   */
  public static Torque toWpilibUnit(org.dyn4j.dynamics.Torque torque) {
    return NewtonMeters.of(torque.getTorque());
  }

  /**
   * Gets a dyn4j force value as a wpilib unit
   *
   * @param force the dyn4j force to convert
   * @return the converted force value as a wpilib Unit of {@link Force}.
   */
  public static Force toWpilibUnit(org.dyn4j.dynamics.Force force) {
    return Newtons.of(force.getForce().getMagnitude());
  }

  /**
   * Gets a dyn4j mass value as a wpilib unit
   *
   * @param mass the dyn4j mass to convert
   * @return the converted mass value as a pair of a wpilib Unit of {@link Mass} and a Unit of
   *     {@link MomentOfInertia}.
   */
  public static Pair<Mass, MomentOfInertia> toWpilibUnit(org.dyn4j.geometry.Mass mass) {
    return Pair.of(Kilograms.of(mass.getMass()), KilogramSquareMeters.of(mass.getInertia()));
  }

  /**
   * Checks if all translations in the input are within a certain threshold in meters.
   *
   * @param thresholdMeters the threshold between any two translations in meters
   * @param translations Varargs of Translation2d objects
   * @return true if all translations are within thresholdMeters of each other
   */
  public static boolean areTranslationsWithinThreshold(
      double thresholdMeters, Translation2d... translations) {
    int n = translations.length;

    // Compare all pairs of translations
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        if (translations[i].getDistance(translations[j]) > thresholdMeters) {
          return false; // Return false if any pair is out of threshold
        }
      }
    }
    return true; // Return true if all pairs are within threshold
  }

  /**
   * Checks if all rotations in the input are within a certain threshold in degrees.
   *
   * @param thresholdDegrees the threshold between any two rotations in degrees
   * @param rotations Varargs of rotation values (in degrees)
   * @return true if all rotations are within thresholdDegrees of each other
   */
  public static boolean areRotationsWithinThreshold(
      double thresholdDegrees, Rotation2d... rotations) {
    int n = rotations.length;

    // Compare all pairs of rotations
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        if (Math.abs(rotations[i].minus(rotations[j]).getDegrees()) > thresholdDegrees) {
          return false; // Return false if any pair is out of threshold
        }
      }
    }
    return true; // Return true if all pairs are within threshold
  }

  /**
   * Checks if all pairs of poses in the input list are within the given translation and rotation
   * thresholds.
   *
   * @param translationThresholdMeters the maximum allowed distance between any two translations in
   *     meters
   * @param rotationThresholdDegrees the maximum allowed difference between any two rotations in
   *     degrees
   * @param poses a list of Pose2d objects
   * @return true if all pairs of poses are within the given thresholds, false otherwise
   */
  public static boolean arePosesWithinThreshold(
      double translationThresholdMeters, double rotationThresholdDegrees, Pose2d... poses) {
    int n = poses.length;

    // Compare all pairs of poses
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        // Check translation threshold
        if (poses[i].getTranslation().getDistance(poses[j].getTranslation())
            > translationThresholdMeters) {
          return false; // Return false if any pair of translations exceeds the threshold
        }

        // Check rotation threshold
        if (Math.abs(poses[i].getRotation().minus(poses[j].getRotation()).getDegrees())
            > rotationThresholdDegrees) {
          return false; // Return false if any pair of rotations exceeds the threshold
        }
      }
    }
    return true; // Return true if all pairs are within the thresholds
  }
}
