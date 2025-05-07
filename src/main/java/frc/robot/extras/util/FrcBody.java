package frc.robot.extras.util;

import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.extras.math.forces.Velocity2d;
import frc.robot.extras.math.mathutils.GeomUtil;
import org.ode4j.ode.internal.DxBody;

/** A Dyn4j Body with additional methods for serialization and conversion to WPILib types. */
public class FrcBody extends DxBody {

  /**
   * A snapshot of the state of a {@link FrcBody}.
   *
   * <p>Used for serialization and deserialization.
   *
   * <p>Contains all the information needed to recreate a {@link FrcBody} at a later time.
   *
   * <p>Contains the following fields:
   *
   * <ul>
   *   <li>{@link Pose2d} pose - the pose of the body. This is the position and rotation of the
   *       body.
   *   <li>{@link Mass} mass - the mass of the body. in kilograms.
   *   <li>{@link MomentOfInertia} momentOfInertia - the moment of inertia of the body. Which is the
   *       resistance of the body to rotation.
   *   <li>{@link Velocity2d} velocity - the linear velocity of the body. This is the rate at which
   *       the body is moving.
   *   <li>{@link AngularVelocity} angularVelocity - the angular velocity of the body. This is the
   *       rate at which the body is rotating.
   *   <li>double linearDamping - the linear damping of the body. This is the rate at which the body
   *       loses linear velocity.
   *   <li>double angularDamping - the angular damping of the body. This is the rate at which the
   *       body loses angular velocity.
   *   <li>double gravityScale - the gravity scale of the body. This is the factor by which gravity
   *       affects the body.
   *   <li>boolean isBullet - whether the body is a bullet. A bullet body is a body that is not
   *       affected by other bodies during a simulation step.
   *   <li>double atRestTime - the time the body has been at rest. This is the time the body has not
   *       been moving.
   *   <li>{@link Translation2d} forces - the forces acting on the body. This is the force that
   *       causes the body to move.
   *   <li>{@link Torque} torque - the torque acting on the body. This is the force that causes the
   *       body to rotate.
   *   <li>{@link Translation2d} accumulatedForce - the accumulated forces acting on the body. This
   *       is the sum of all the forces acting on the body.
   *   <li>{@link Torque} accumulatedTorque - the accumulated torque acting on the body. This is the
   *       sum of all the torques acting on the body.
   * </ul>
   *
   * <p>Implements {@link StructSerializable} for serialization and deserialization.
   */
  public record FrcBodySnapshot(
      Pose2d pose,
      Mass mass,
      MomentOfInertia momentOfInertia,
      Velocity2d velocity,
      AngularVelocity angularVelocity,
      double linearDamping,
      double angularDamping,
      double gravityScale,
      boolean isBullet,
      double atRestTime,
      Translation2d forces,
      Torque torque,
      Translation2d accumulatedForce,
      Torque accumulatedTorque)
      implements StructSerializable {
    public static final Struct<FrcBodySnapshot> struct =
        StructGenerator.genRecord(FrcBodySnapshot.class);
  }

  public FrcBodySnapshot snapshot() {
    return new FrcBodySnapshot(
        GeomUtil.toWpilibPose2d(get()),
        GeomUtil.toWpilibUnit(getMass()).getFirst(),
        GeomUtil.toWpilibUnit(getMass()).getSecond(),
        new Velocity2d(getLinearVel()., getLinearVel().y),
        RadiansPerSecond.of(-getAngularVel()),
        getLinearDamping(),
        getAngularDamping(),
        getGravityMode(),
        isKinematic(),
        this.world,
        new Translation2d(getForce().x, getForce().y),
        NewtonMeters.of(getTorque()),
        new Translation2d(getAccumulatedForce().x, getAccumulatedForce().y),
        NewtonMeters.of(getAccumulatedTorque()));
  }
}
