package frc.robot.extras.math.mathutils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Torque;
import frc.robot.extras.math.forces.Velocity2d;
import org.ode4j.math.DVector3;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternion;
// import org.ode4j.math.; 

/** 
 * utils to convert between WPILIB geometry classes and ODE4j math types 
 * (embedded in the XY plane: Z=0, axis up = (0,0,1)). 
 */
public class GeomUtil {

  /** ------------------- Translation & Velocity ------------------- */

  public static DVector3 toOdeVector(Translation2d t) {
    return new DVector3(t.getX(), t.getY(), 0.0);
  }

  public static DVector3 toOdeVector(Velocity2d v) {
    return new DVector3(v.getVX(), v.getVY(), 0.0);
  }

  public static Translation2d toWpilibTranslation(DVector3 v) {
    return new Translation2d(v.get0(), v.get1());
  }

  /** ------------------------ Rotation ------------------------ */

  /**
   * Creates an ODE4j quaternion representing a rotation about +Z by the given angle.
   */
  public static DQuaternion toOdeQuaternion(Rotation2d r) {
    final double half = r.getRadians() * 0.5;
    // axis = (0,0,1)
    return new DQuaternion(Math.cos(half), 0, 0, Math.sin(half));
  }

  /** Extracts the heading (around Z) from an ODE quaternion. */
  public static Rotation2d toWpilibRotation(DQuaternion q) {
    // θ = 2·atan2(z, w)
    double angle = 2.0 * Math.atan2(q.getZ(), q.getW());
    return new Rotation2d(angle);
  }

  /** ------------------------ Pose / Transform ------------------------ */

   /**
   * Packs a Pose2d into a pair of (translation, rotation).
   */
  public static Pair<DVector3, DQuaternion> toOdePose(Pose2d pose) {
    DVector3 pos = toOdeVector(pose.getTranslation());
    DQuaternion rot = toOdeQuaternion(pose.getRotation());
    return Pair.of(pos, rot);
  }

  /**
   * Unpacks an ODE4j (translation, rotation) back into a Pose2d.
   */
  public static Pose2d toWpilibPose(DVector3 pos, DQuaternion rot) {
    return new Pose2d(
      toWpilibTranslation(pos),
      toWpilibRotation(rot)
    );
  }


  /** ---------------------- ChassisSpeeds ---------------------- */

  public static DVector3 toOdeLinearVel(ChassisSpeeds speeds) {
    return new DVector3(
      speeds.vxMetersPerSecond,
      speeds.vyMetersPerSecond,
      0.0
    );
  }

  public static ChassisSpeeds toWpilibChassisSpeeds(DVector3 linVel, double angVelRadPerSec) {
    return new ChassisSpeeds(
      linVel.get0(),
      linVel.get1(),
      angVelRadPerSec
    );
  }

  /** -------------------- Units: Force, Torque, Mass -------------------- */

  public static Force toWpilibUnit(org.ode4j.math.DVector3 forceVec) {
    // magnitude in N
    return Newtons.of(forceVec.length());
  }

  public static Torque toWpilibUnit(org.ode4j.math.DVector3 torqueVec, boolean unused) {
    // treat as pure torque around Z
    return NewtonMeters.of(torqueVec.length()); 
  }

  public static Pair<Mass, MomentOfInertia> toWpilibUnit(double mass, double inertiaAboutZ) {
    return Pair.of(Kilograms.of(mass), KilogramSquareMeters.of(inertiaAboutZ));
  }

  /** ------------------- Threshold Checks ------------------- */

  public static boolean areTranslationsWithinThreshold(
      double threshold, Translation2d... ts) {
    for (int i = 0; i < ts.length; i++) {
      for (int j = i+1; j < ts.length; j++) {
        if (ts[i].getDistance(ts[j]) > threshold) {
          return false;
        }
      }
    }
    return true;
  }

  public static boolean areRotationsWithinThreshold(
      double thresholdDeg, Rotation2d... rs) {
    for (int i = 0; i < rs.length; i++) {
      for (int j = i+1; j < rs.length; j++) {
        if (Math.abs(rs[i].minus(rs[j]).getDegrees()) > thresholdDeg) {
          return false;
        }
      }
    }
    return true;
  }

  public static boolean arePosesWithinThreshold(
      double transThresh, double rotThreshDeg, Pose2d... poses) {
    for (int i = 0; i < poses.length; i++) {
      for (int j = i+1; j < poses.length; j++) {
        if (poses[i].getTranslation().getDistance(poses[j].getTranslation()) > transThresh
         || Math.abs(poses[i].getRotation().minus(poses[j].getRotation()).getDegrees()) > rotThreshDeg) {
          return false;
        }
      }
    }
    return true;
  }
}
