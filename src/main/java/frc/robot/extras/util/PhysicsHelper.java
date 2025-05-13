package frc.robot.extras.util;

import org.ode4j.math.*;
import org.ode4j.ode.*;

public final class PhysicsHelper {

  private PhysicsHelper() {
    // Prevent instantiation
  }

  /**
   * Applies a force to a body at its center of mass.
   *
   * @param body The body to apply the force to.
   * @param force The force vector.
   */
  public static void applyForce(DBody body, DVector3 force) {
    body.addForce(force);
  }

  /**
   * Applies a torque to a body.
   *
   * @param body The body to apply the torque to.
   * @param torque The torque vector.
   */
  public static void applyTorque(DBody body, DVector3 torque) {
    body.addTorque(torque);
  }

  /**
   * Applies a force at a specific point on the body.
   *
   * @param body The body to apply the force to.
   * @param force The force vector.
   * @param point The point (in world coordinates) where the force is applied.
   */
  public static void applyForceAtPoint(DBody body, DVector3 force, DVector3 point) {
    body.addForceAtPos(force, point);
  }

  /**
   * Applies a frictional torque to simulate angular damping.
   *
   * @param body The body to apply the friction to.
   * @param frictionCoefficient The coefficient of angular friction.
   */
  public static void applyAngularFriction(DBody body, double frictionCoefficient) {
    DVector3C angularVel = body.getAngularVel();
    DVector3 frictionTorque =
        new DVector3(
            -frictionCoefficient * angularVel.get0(),
            -frictionCoefficient * angularVel.get1(),
            -frictionCoefficient * angularVel.get2());
    body.addTorque(frictionTorque);
  }

  /**
   * Applies linear damping to simulate air resistance or surface friction.
   *
   * @param body The body to apply the damping to.
   * @param dampingCoefficient The coefficient of linear damping.
   */
  public static void applyLinearDamping(DBody body, double dampingCoefficient) {
    DVector3C linearVel = body.getLinearVel();
    DVector3 dampingForce =
        new DVector3(
            -dampingCoefficient * linearVel.get0(),
            -dampingCoefficient * linearVel.get1(),
            -dampingCoefficient * linearVel.get2());
    body.addForce(dampingForce);
  }

  /**
   * Creates a contact joint with specified friction parameters.
   *
   * @param world The dynamics world.
   * @param contactGroup The joint group to which the contact joint will be added.
   * @param contact The contact information, including position and normal.
   * @param mu The coefficient of friction.
   */
  public static void createContactJoint(
      DWorld world, DJointGroup contactGroup, DContactGeom contact, double mu) {
    DContact dContact = new DContact();
    dContact.geom.g1 = contact.g1;
    dContact.geom.g2 = contact.g2;
    dContact.surface.mode = dContact.surface.mode;
    // | dContact.surface.;
    dContact.surface.mu = mu;

    DJoint contactJoint = OdeHelper.createContactJoint(world, contactGroup, dContact);
    contactJoint.attach(contact.g1.getBody(), contact.g2.getBody());
  }
}
