package frc.robot.extras.util;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.extras.math.forces.Velocity3d;
import frc.robot.extras.math.mathutils.GeomUtil;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;

/** A wrapper around ODE4j’s DBody to a WPILib‑friendly snapshot API. */
// TODO: see if this works
public class FrcBody {
  private final DBody body;
  private final DGeom geom;

  public FrcBody(DWorld world, DGeom geom) {
    this.body = OdeHelper.createBody(world);
    this.geom = geom;
    geom.setBody(body);
  }

  public DBody getBody() {
    return body;
  }

  /** The DGeom attached to this body. */
  public DGeom getFirstGeom() {
    return geom;
  }

  public record FrcBodySnapshot(
      Pose3d pose,
      Mass mass,
      MomentOfInertia momentOfInertia,
      Velocity3d velocity,
      AngularVelocity angularVelocity,
      double linearDamping,
      double angularDamping,
      boolean isKinematic,
      Translation2d forces,
      Torque torque)
      implements StructSerializable {
    public static final Struct<FrcBodySnapshot> struct =
        StructGenerator.genRecord(FrcBodySnapshot.class);
  }

  public FrcBodySnapshot snapshot() {
    // --- Pose ---
    DVector3C posArr = body.getPosition(); // [x,y,z]
    DQuaternionC quatArr = body.getQuaternion(); // [w,x,y,z]
    Pose3d pose3d = GeomUtil.toWpilibPose(posArr, quatArr);

    // --- Mass & Inertia ---
    DMass m = OdeHelper.createMass();
    // TODO: figure out what to do with mass. Move to another method probs
    body.setMass(m); // fills m with current mass/inertia
    Mass wpim = Mass.ofBaseUnits(m.getMass(), Kilograms);
    MomentOfInertia wpiI = MomentOfInertia.ofBaseUnits(m.getI().get22(), KilogramSquareMeters);

    // --- Velocities ---
    double[] linV = body.getLinearVel().toDoubleArray(); // [vx,vy,vz]
    double[] angV = body.getAngularVel().toDoubleArray(); // [wx,wy,wz]
    Velocity3d vel = new Velocity3d(linV[0], linV[1], linV[2]);
    AngularVelocity angVel = RadiansPerSecond.of(angV[2]);

    // --- Forces & Torques ---
    double[] f = body.getForce().toDoubleArray(); // [fx,fy,fz]
    double[] tqArr = body.getTorque().toDoubleArray(); // [tx,ty,tz]
    Translation2d force2d = new Translation2d(f[0], f[1]);
    Torque torque2d = NewtonMeters.of(tqArr[2]);

    return new FrcBodySnapshot(
        pose3d,
        wpim,
        wpiI,
        vel,
        angVel,
        body.getLinearDamping(),
        body.getAngularDamping(),
        body.isKinematic(),
        force2d,
        torque2d);
  }
}
