package frc.robot.sim.simMechanism;

import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;

/**
 * Simple wrapper around an ODE4j world and bodies to replace Dyn4j's world/body, now fully in 3D
 * (no 2D embeddings).
 */
public class OdeWorld {
  private final DWorld world;
  private final DSpace space;

  public OdeWorld() {
    world = OdeHelper.createWorld();
    // standard gravity along -Z axis
    world.setGravity(0, 0, -9.81);
    space = OdeHelper.createSimpleSpace(null);
  }

  public DWorld getWorld() {
    return world;
  }

  public DSpace getSpace() {
    return space;
  }

  /**
   * Creates a body with a box fixture in the world.
   *
   * @param length X dimension (m)
   * @param width Y dimension (m)
   * @param height Z dimension (m)
   * @param massValue total mass (kg)
   * @return the created DBody
   */
  public DBody createBoxBody(double length, double width, double height, double massValue) {
    // 1) Create the rigid body
    DBody body = OdeHelper.createBody(world);

    // 2) Configure and assign mass/inertia
    DMass m = OdeHelper.createMass();
    // setBox expects density and half extents
    double hx = length / 2.0;
    double hy = width / 2.0;
    double hz = height / 2.0;
    m.setBox(1.0, hx, hy, hz);
    // adjust total mass
    m.adjust(massValue);
    body.setMass(m);

    // 3) Attach collision geometry
    DBox box = OdeHelper.createBox(space, length, width, height);
    box.setBody(body);

    return body;
  }

  /**
   * Creates a body with a sphere fixture in the world.
   *
   * @param radius sphere radius (m)
   * @param massValue total mass (kg)
   * @return the created DBody
   */
  public DBody createSphereBody(double radius, double massValue) {
    DBody body = OdeHelper.createBody(world);
    DMass m = OdeHelper.createMass();
    m.setSphere(1.0, radius);
    m.adjust(massValue);
    body.setMass(m);

    org.ode4j.ode.DSphere sphere = OdeHelper.createSphere(space, radius);
    sphere.setBody(body);
    return body;
  }

  /**
   * Creates a body with a capsule fixture in the world.
   *
   * @param radius capsule radius (m)
   * @param length capsule length (m)
   * @param massValue total mass (kg)
   * @return the created DBody
   */
  public DBody createCapsuleBody(double radius, double length, double massValue) {
    DBody body = OdeHelper.createBody(world);
    DMass m = OdeHelper.createMass();
    m.setCapsule(1.0, 3, radius, length); // 3 = capsule along Z axis
    m.adjust(massValue);
    body.setMass(m);

    org.ode4j.ode.DCapsule capsule = OdeHelper.createCapsule(space, radius, length);
    capsule.setBody(body);
    return body;
  }

  /**
   * Step the world simulation forward by dt seconds.
   *
   * @param dt time step (s)
   */
  public void step(double dt) {
    // collision detection would go here
    world.step(dt);
  }
}
