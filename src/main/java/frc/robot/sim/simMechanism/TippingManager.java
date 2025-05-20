package frc.robot.sim.simMechanism;

import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DHingeJoint;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJoint.PARAM_N;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxWorld;
import org.ode4j.ode.internal.joints.DxJointGroup;

/**
 * Manages tipping hinge and per-wheel contacts when the chassis tips. Uses ODE4j to create/destroy
 * hinge joints and disable/enable wheel contacts.
 */
public class TippingManager {
  private final DxWorld world;
  private final DxBody chassisBody;
  private final List<DVector3> supportPolygon;
  private final DxJointGroup contactGroup;
  private DHingeJoint tippingHinge;

  // Tipping edge information for contact filtering
  private DVector3 tippingEdgeStart;
  private DVector3 tippingEdgeNormal;
  private int activeTippingEdge = -1;

  /**
   * @param world the ODE physics world
   * @param chassisBody the chassis rigid body
   * @param supportPolygon list of chassis-local wheel contact points (convex hull)
   * @param contactGroup joint group for wheel contacts
   */
  public TippingManager(
      DxWorld world, DxBody chassisBody, List<DVector3> supportPolygon, DxJointGroup contactGroup) {
    this.world = world;
    this.chassisBody = chassisBody;
    this.supportPolygon = supportPolygon;
    this.contactGroup = contactGroup;
    this.tippingEdgeStart = new DVector3();
    this.tippingEdgeNormal = new DVector3();
  }

  /**
   * Activates a hinge joint along the edge indicated by edgeIdx and disables contacts of lifted
   * wheels.
   *
   * @param edgeIdx index in supportPolygon of the first vertex of the tipping edge
   */
  public void activateHinge(int edgeIdx) {
    if (tippingHinge != null) {
      return; // already active
    }
    // Get the two vertices in world coordinates
    DVector3 a = supportPolygon.get(edgeIdx);
    DVector3 b = supportPolygon.get((edgeIdx + 1) % supportPolygon.size());

    // Create hinge joint between chassis and static ground
    tippingHinge = OdeHelper.createHingeJoint(world);
    tippingHinge.attach(chassisBody, null); // null body = static environment

    // Anchor at midpoint of edge, axis vertical
    double anchorX = (a.get0() + b.get0()) * 0.5;
    double anchorY = (a.get1() + b.get1()) * 0.5;
    tippingHinge.setAnchor(anchorX, anchorY, 0);
    tippingHinge.setAxis(0, 0, 1);

    // Configure joint parameters (stiffness, damping)
    tippingHinge.setParam(PARAM_N.dParamStopERP1, 0.8);
    tippingHinge.setParam(PARAM_N.dParamStopCFM2, 1e-5);

    // Disable contacts on lifted side
    disableLiftSideContacts(edgeIdx);
  }

  /** Deactivates the tipping hinge and re-enables all wheel contacts. */
  public void deactivateHinge() {
    if (tippingHinge != null) {
      tippingHinge.destroy();
      tippingHinge = null;
    }
    activeTippingEdge = -1; // No active tipping edge
    enableAllWheelContacts();
  }

  /**
   * Disables contact joints for wheels on the side being tipped up. This method prepares the
   * tipping edge data for the contact callbacks to use.
   *
   * @param edgeIdx index of the tipping edge
   */
  private void disableLiftSideContacts(int edgeIdx) {
    // Store the edge index
    activeTippingEdge = edgeIdx;

    // Get the edge vertices
    DVector3 a = supportPolygon.get(edgeIdx);
    DVector3 b = supportPolygon.get((edgeIdx + 1) % supportPolygon.size());

    // Calculate edge vector
    DVector3 edge = new DVector3();
    edge.eqDiff(b, a); // edge = b - a

    // Calculate outward normal (perpendicular to edge in XY plane)
    tippingEdgeNormal.set(-edge.get1(), edge.get0(), 0.0);
    tippingEdgeNormal.normalize(); // Ensure it's a unit vector

    // Store edge start for reference
    tippingEdgeStart.set(a);

    // Empty contact group to force recreation of contacts with new filtering
    enableAllWheelContacts();
  }

  /** Re-enables all wheel contact joints by clearing contact group so they'll be recreated. */
  private void enableAllWheelContacts() {
    contactGroup.empty();
  }

  /**
   * Filter method to be called during collision detection. This should be integrated with your
   * collision system's near callback.
   *
   * @param contact The contact to be potentially created
   * @param pos The position of the contact point in world coordinates
   * @return true if the contact should be allowed, false if it should be filtered out
   */
  public boolean filterContact(DContact contact, DVector3 pos) {
    // If no tipping edge is active, allow all contacts
    if (activeTippingEdge == -1) {
      return true;
    }

    // Calculate vector from edge start to contact point
    DVector3 relVector = new DVector3();
    relVector.eqDiff(pos, tippingEdgeStart);

    // Determine which side of the edge the contact falls on
    double dot = relVector.dot(tippingEdgeNormal);

    // If dot product is positive, the point is on the "outside" (lifting) side
    // and should be filtered out
    return dot <= 0;
  }

  /**
   * Process collision between two geometries. This method should be called from the collision
   * system's near callback.
   *
   * @param o1 First collision geometry
   * @param o2 Second collision geometry
   * @return Number of contacts created
   */
  public int processCollision(DGeom o1, DGeom o2) {
    // Get collider bodies (if any)
    DxBody b1 = (DxBody) o1.getBody();
    DxBody b2 = (DxBody) o2.getBody();

    // Skip if neither geometry belongs to the chassis we're managing
    if (b1 != chassisBody && b2 != chassisBody) {
      return 0;
    }

    // Maximum number of contact points to generate
    final int N = 4;
    DContactBuffer contacts = new DContactBuffer(N);

    // Get potential contacts between geometries
    int numContacts = OdeHelper.collide(o1, o2, N, contacts.getGeomBuffer());

    // No contacts found
    if (numContacts == 0) {
      return 0;
    }

    // Process each potential contact
    int createdContacts = 0;
    for (int i = 0; i < numContacts; i++) {
      DContact contact = contacts.get(i);

      // Extract contact position
      DVector3 pos = contact.geom.pos;

      // Apply filtering based on tipping edge
      if (filterContact(contact, pos)) {
        // Configure contact properties
        contact.surface.mode = OdeConstants.dContactBounce | OdeConstants.dContactSoftERP;
        contact.surface.mu = 0.5; // Friction coefficient
        contact.surface.bounce = 0.1;
        contact.surface.bounce_vel = 0.1;
        contact.surface.soft_erp = 0.5;

        // Create contact joint
        DJoint c = OdeHelper.createContactJoint(world, contactGroup, contact);
        c.attach(b1, b2);
        createdContacts++;
      }
    }

    return createdContacts;
  }

  /**
   * Get the current rotor inertia if needed by modules. Placeholder: can delegate to chassis
   * mass/inertia logic.
   */
  public double getRotorInertia() {
    // Adapt this to return real rotor inertia calculation
    return 0.0;
  }

  /**
   * Check if tipping is currently active.
   *
   * @return true if the robot is currently tipping on an edge
   */
  @AutoLogOutput(key = "TippingManager/IsTipping")
  public boolean isTipping() {
    return tippingHinge != null;
  }

  /**
   * Get the index of the active tipping edge.
   *
   * @return Edge index or -1 if not tipping
   */
  public int getActiveTippingEdge() {
    return activeTippingEdge;
  }
}
