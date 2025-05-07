package frc.robot.extras.util;

import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.extras.math.forces.Velocity2d;
import frc.robot.extras.math.forces.Velocity3d;
import frc.robot.extras.math.mathutils.GeomUtil;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxWorld;

/** A Dyn4j Body with additional methods for serialization and conversion to WPILib types. */
public class FrcBody extends DxBody {

  public FrcBody(DxWorld world) {
    super(world);
  }
  public record FrcBodySnapshot(
      Pose3d pose,
      Mass mass,
      MomentOfInertia momentOfInertia,
      Velocity3d velocity,
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
        GeomUtil.toWpilibPose(getPosition(), getQuaternion()),
        GeomUtil.toWpilibUnit(getMass().getMass(), getMass().),
        GeomUtil.toWpilibUnitTorque(getTorque()),
        new Velocity3d(getLinearVel().get0(), getLinearVel().get1(), getLinearVel().get2()),
        RadiansPerSecond.of(-getAngularVel().get2()), // TODO: check this one
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
