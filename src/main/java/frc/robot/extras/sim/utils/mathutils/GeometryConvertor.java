package frc.robot.extras.sim.utils.mathutils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Torque;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Newtons;

import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Rotation;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;
import sham.utils.geometry.Velocity2d;

/** utils to convert between WPILIB and dyn4j geometry classes */
public class GeometryConvertor {
    public static Vector2 toDyn4jVector2(Translation2d wpilibTranslation2d) {
        return new Vector2(wpilibTranslation2d.getX(), wpilibTranslation2d.getY());
    }

    public static Vector2 toDyn4jVector2(Velocity2d wpilibTranslation2d) {
        return new Vector2(wpilibTranslation2d.getVX(), wpilibTranslation2d.getVY());
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

    public static Rectangle toDyn4jRectangle(Rectangle2d wpilibRectangle2d) {
        Pose2d rectCenter = wpilibRectangle2d.getCenter();
        Rectangle shape = new Rectangle(wpilibRectangle2d.getXWidth(), wpilibRectangle2d.getYWidth());
        shape.translate(toDyn4jVector2(rectCenter.getTranslation()));
        shape.rotate(toDyn4jRotation(rectCenter.getRotation()));
        return shape;
    }

    public static Transform2d toWpilibTransform2d(Transform dyn4jTransform) {
        return new Transform2d(
            toWpilibTranslation2d(dyn4jTransform.getTranslation()),
            toWpilibRotation2d(dyn4jTransform.getRotation()));
    }

    public static Pose2d toWpilibPose2d(Transform dyn4jTransform) {
        return new Pose2d(
            toWpilibTranslation2d(dyn4jTransform.getTranslation()),
            toWpilibRotation2d(dyn4jTransform.getRotation()));
    }

    public static Vector2 toDyn4jLinearVelocity(ChassisSpeeds wpilibChassisSpeeds) {
        return new Vector2(wpilibChassisSpeeds.vxMetersPerSecond, wpilibChassisSpeeds.vyMetersPerSecond);
    }

    public static ChassisSpeeds toWpilibChassisSpeeds(Vector2 dyn4jLinearVelocity, double angularVelocityRadPerSec) {
        return new ChassisSpeeds(dyn4jLinearVelocity.x, dyn4jLinearVelocity.y, angularVelocityRadPerSec);
    }

    public static Velocity2d getChassisSpeedsTranslationalComponent(ChassisSpeeds chassisSpeeds) {
        return new Velocity2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    public static Torque toWpilibUnit(org.dyn4j.dynamics.Torque torque) {
        return NewtonMeters.of(torque.getTorque());
    }

    public static Force toWpilibUnit(org.dyn4j.dynamics.Force force) {
        return Newtons.of(force.getForce().getMagnitude());
    }

    public static Pair<Mass, MomentOfInertia> toWpilibUnit(org.dyn4j.geometry.Mass mass) {
        return Pair.of(Kilograms.of(mass.getMass()), KilogramSquareMeters.of(mass.getInertia()));
    }
}
