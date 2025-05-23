package frc.robot.sim.simMechanism.simSwerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Torque;
import frc.robot.extras.math.mathutils.MassMath.PhysicsMass;
import frc.robot.extras.math.mathutils.MeasureMath;
import frc.robot.extras.math.mathutils.MeasureMath.XY;
import frc.robot.sim.SimRobot;
import frc.robot.sim.configs.SimSwerveConfig;
import frc.robot.sim.simController.SimMotorController;
import frc.robot.sim.simField.SimArena.SimEnvTiming;
import frc.robot.sim.simMechanism.SimDriveTrain;
import frc.robot.sim.simMechanism.TippingManager;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.internal.DxBody;
import org.ode4j.ode.internal.DxWorld;
import org.ode4j.ode.internal.joints.DxJointGroup;

/** The class which simulates the swerve drive using ODE4j. */
public class SimSwerve extends SimDriveTrain {
  private final SimEnvTiming timing;
  private final SimSwerveModule[] moduleSimulations;
  private final SimGyro gyroSimulation;
  private final SimSwerveConfig config;
  private final SimRobot<SimSwerve> robot;
  private final PhysicsMass chassisMass;
  private final SwerveDriveKinematics kinematics;

  private MomentOfInertia rotorInertia;
  private final MomentOfInertia rotorInertiaWhenTranslating;
  private final MomentOfInertia rotorInertiaWhenRotating;

  private final TippingManager tippingManager;
  private final List<DVector3> supportPolygon;
  private final DxJointGroup contactGroup;

  private Pose2d lastPose;

  // Tipping detection parameters
  private static final double TIPPING_THRESHOLD =
      0.95; // Dot product threshold for tipping detection
  private static final double STABLE_THRESHOLD =
      0.98; // Dot product threshold to consider robot stable

  public SimSwerve(SimRobot<SimSwerve> robot, SimSwerveConfig config) {
    super(config, robot.arena());
    this.robot = robot;
    this.timing = robot.timing();
    this.config = config;
    this.lastPose = new Pose2d();

    // Create contact joint group for wheel contacts
    this.contactGroup = (DxJointGroup) OdeHelper.createJointGroup();

    // initialize modules & gyro
    this.moduleSimulations = new SimSwerveModule[config.moduleTranslations.length];
    Force gravityPerModule = Newtons.of(config.robotMassKg * 9.81).div(moduleSimulations.length);
    for (int i = 0; i < moduleSimulations.length; i++) {
      final int moduleIndex = i; // jank ass solution
      moduleSimulations[i] =
          new SimSwerveModule(
              robot,
              config,
              i,
              gravityPerModule,
              () -> getAdjustedRotorInertia(moduleIndex),
              SimMotorController.none(),
              SimMotorController.none());
    }
    this.gyroSimulation = new SimGyro(timing, config.gyroConfig);

    // Compute support polygon from module translations
    // Convert translations to DVector3 for TippingManager
    supportPolygon =
        Arrays.stream(config.moduleTranslations)
            .map(t -> new DVector3(t.getX(), t.getY(), 0.0))
            .collect(Collectors.toList());

    // Create tipping manager
    tippingManager =
        new TippingManager(
            (DxWorld) robot.arena().getWorld(),
            (DxBody) getChassisBody(),
            supportPolygon,
            contactGroup);

    // chassis mass/inertia
    this.chassisMass =
        new PhysicsMass(Kilograms.of(config.robotMassKg), KilogramSquareMeters.of(config.robotMoI));
    this.kinematics = new SwerveDriveKinematics(config.moduleTranslations);

    // precompute rotor inertias
    Distance wheelRadius = Meters.of(config.swerveModuleConfig.wheelsRadiusMeters);
    Distance wheelBase = XY.of(config.moduleTranslations[0]).magnitude();
    var rotationalMass =
        MeasureMath.div(chassisMass.moi(), wheelBase.times(wheelBase))
            .div(moduleSimulations.length);
    this.rotorInertiaWhenTranslating =
        MeasureMath.times(
            chassisMass.mass().div(moduleSimulations.length), wheelRadius.times(wheelRadius));
    this.rotorInertiaWhenRotating =
        MeasureMath.times(rotationalMass, wheelRadius.times(wheelRadius));
    this.rotorInertia = rotorInertiaWhenTranslating;

    // Set up collision callbacks
    // setupCollisionCallbacks();
  }

  // /** Sets up collision callbacks to integrate TippingManager's contact filtering. */
  // private void setupCollisionCallbacks(Object date) {
  //   // Register a near callback with the space
  //   OdeHelper.spaceCollide(getSpace(), date, this::nearCallback);
  // }

  @Override
  public void simTick() {
    // Update tipping manager first

    // Check for tipping conditions
    checkTipping();

    // Continue with normal sim tick
    simulateModulePropulsion();
    simulateModuleFriction();

    // update gyro
    gyroSimulation.updateSimulationSubTick(
        getChassisWorldPose2d().getRotation().getMeasure(), getTickTwist());
    Logger.recordOutput("Odometry/ChassisPose", getChassisWorldPose2d());
    super.simTick();
  }

  /** Checks if the robot should be tipping and activates/deactivates tipping as needed. */
  private void checkTipping() {
    // Get chassis orientation (Z-up coordinate system)
    DVector3 worldUp = new DVector3(0, 0, 1);
    DVector3 chassisUp = new DVector3();
    getChassisBody().getRelPointPos(0, 0, 1, chassisUp);
    chassisUp.normalize();

    // Dot product with world up vector (1 = aligned, 0 = perpendicular)
    double upDot = chassisUp.dot(worldUp);
    Logger.recordOutput("TippingManager/ChassisUpDot", upDot);

    // If we're already tipping and robot has stabilized, deactivate tipping
    if (tippingManager.isTipping() && upDot > STABLE_THRESHOLD) {
      tippingManager.deactivateHinge();
      Logger.recordOutput("TippingManager/Status", "Stabilized");
    }
    // If we're not tipping but should be, find the closest edge and activate tipping
    else if (!tippingManager.isTipping() && upDot < TIPPING_THRESHOLD) {
      // Find which edge we're tipping over
      int edgeIdx = findTippingEdge();
      if (edgeIdx >= 0) {

        tippingManager.activateHinge(edgeIdx);
        Logger.recordOutput("TippingManager/Status", "Tipping on edge " + edgeIdx);
      }
    }
  }

  /**
   * Finds the edge that the robot is tipping over based on chassis orientation.
   *
   * @return Edge index or -1 if no edge found
   */
  private int findTippingEdge() {
    // Get chassis orientation and velocity
    DVector3 gravity = new DVector3(0, 0, -9.81);

    // Project gravity onto XY plane in chassis frame
    DVector3 chassisGravity = new DVector3();
    getChassisBody().vectorFromWorld(gravity, chassisGravity);
    chassisGravity.set2(0); // Zero out Z component to get XY projection

    if (chassisGravity.length() < 0.1) {
      return -1; // Not enough horizontal gravity component to determine tipping
    }

    chassisGravity.normalize();

    // Find edge most aligned with the gravity direction
    int bestEdge = -1;
    double bestAlignment = -Double.MAX_VALUE;

    for (int i = 0; i < supportPolygon.size(); i++) {
      DVector3 a = supportPolygon.get(i);
      DVector3 b = supportPolygon.get((i + 1) % supportPolygon.size());

      // Calculate edge normal (perpendicular to edge in XY plane)
      DVector3 edge = new DVector3();
      edge.eqDiff(b, a);
      DVector3 normal = new DVector3(-edge.get1(), edge.get0(), 0);
      normal.normalize();

      // Check alignment with gravity projection
      double alignment = normal.dot(chassisGravity);

      // If this edge has better alignment with gravity direction
      if (alignment > bestAlignment) {
        bestAlignment = alignment;
        bestEdge = i;
      }
    }

    // Only return an edge if alignment is significant
    return (bestAlignment > 0.7) ? bestEdge : -1;
  }

  /**
   * Gets the adjusted rotor inertia for a specific module based on tipping state.
   *
   * @param moduleIdx The module index
   * @return The adjusted rotor inertia for the module
   */
  private MomentOfInertia getAdjustedRotorInertia(int moduleIdx) {
    // If tipping is active, use TippingManager's rotor inertia calculation
    if (tippingManager.isTipping()) {
      // Check if this module is on the lifted side
      int tippingEdge = tippingManager.getActiveTippingEdge();
      if (tippingEdge >= 0) {
        // Get edge vertices
        DVector3 a = supportPolygon.get(tippingEdge);
        DVector3 b = supportPolygon.get((tippingEdge + 1) % supportPolygon.size());

        // Calculate edge normal (pointing outward)
        DVector3 edge = new DVector3();
        edge.eqDiff(b, a);
        DVector3 normal = new DVector3(-edge.get1(), edge.get0(), 0);
        normal.normalize();

        // Calculate vector from edge start to module position
        DVector3 modulePos =
            new DVector3(
                config.moduleTranslations[moduleIdx].getX(),
                config.moduleTranslations[moduleIdx].getY(),
                0);
        DVector3 relVector = new DVector3();
        relVector.eqDiff(modulePos, a);

        // Dot product determines which side of edge the module is on
        double dot = relVector.dot(normal);

        // If module is on lifting side, reduce inertia
        if (dot > 0) {
          // Scale base rotor inertia by some factor for lifted wheels
          double liftFactor = 0.5; // Reduced effective inertia for lifted wheels
          return rotorInertia.times(liftFactor);
        }
      }
    }

    // Default to normal rotor inertia calculation
    return rotorInertia;
  }

  /** Simulates the module propulsion. */
  private void simulateModulePropulsion() {
    Rotation2d rot = getChassisWorldPose2d().getRotation();
    Force totalForceX = Newtons.zero();
    Force totalForceY = Newtons.zero();
    Torque totalTorque = NewtonMeters.zero();
    Force propulsionTotalMag = Newtons.zero();

    // accumulate module forces
    for (SimSwerveModule mod : moduleSimulations) {
      XY<Distance> pos = XY.of(mod.translation().rotateBy(rot));
      XY<Force> propulsion = mod.force(rot);
      propulsionTotalMag = propulsionTotalMag.plus(propulsion.magnitude());
      var pack = chassisMass.forcesDueToOffsetForces(propulsion, pos);
      totalForceX = totalForceX.plus(pack.getFirst().x());
      totalForceY = totalForceY.plus(pack.getFirst().y());
      totalTorque = totalTorque.plus(pack.getSecond());
      Logger.recordOutput("Propulsion/Module" + mod.id() + "/force", pack.getFirst());
      Logger.recordOutput("Propulsion/Module" + mod.id() + "/torque", pack.getSecond());
    }

    // dynamic rotor inertia interpolation
    double fx = totalForceX.in(Newtons);
    double fy = totalForceY.in(Newtons);
    double ft = propulsionTotalMag.in(Newtons);
    double txRatio = ft > 1e-3 ? fx / ft : 0.0;
    double tyRatio = ft > 1e-3 ? fy / ft : 0.0;
    double transRatio = Math.hypot(txRatio, tyRatio);
    this.rotorInertia =
        rotorInertiaWhenTranslating
            .times(transRatio)
            .plus(rotorInertiaWhenRotating.times(1 - transRatio));
    Logger.recordOutput("RotorInertia/translationRatio", transRatio);

    // apply to body
    DVector3 f = new DVector3(totalForceX.in(Newtons), totalForceY.in(Newtons), 0.0);
    DVector3 tau = new DVector3(0.0, 0.0, totalTorque.in(NewtonMeters));
    chassis.addForce(f);
    chassis.addTorque(tau);

    Logger.recordOutput("Propulsion/TotalForce", new XY<>(totalForceX, totalForceY));
    Logger.recordOutput("Propulsion/TotalTorque", totalTorque);
  }

  /** Simulates the module friction. */
  private void simulateModuleFriction() {
    Rotation2d rot = getChassisWorldPose2d().getRotation();
    ChassisSpeeds speeds = getChassisWorldSpeeds();

    LinearAcceleration ax = MetersPerSecondPerSecond.zero();
    LinearAcceleration ay = MetersPerSecondPerSecond.zero();
    AngularAcceleration alpha = RadiansPerSecondPerSecond.zero();

    // accumulate friction
    for (int i = 0; i < moduleSimulations.length; i++) {
      XY<Force> friction = moduleSimulations[i].friction(speeds, rot);
      Pair<XY<LinearAcceleration>, AngularAcceleration> pack =
          chassisMass.accelerationsDueToForce(
              friction, XY.of(moduleSimulations[i].translation().rotateBy(rot)));
      ax = ax.plus(pack.getFirst().x());
      ay = ay.plus(pack.getFirst().y());
      alpha = alpha.plus(pack.getSecond());
      Logger.recordOutput("Friction/Module" + i + "/force", friction);
    }

    // clamp to no reverse
    ChassisSpeeds wheelSpeeds =
        kinematics.toChassisSpeeds(
            Arrays.stream(moduleSimulations)
                .map(SimSwerveModule::state)
                .toArray(SwerveModuleState[]::new));
    ChassisSpeeds unwanted = wheelSpeeds.minus(speeds);
    LinearAcceleration axStop =
        MeasureMath.negate(MetersPerSecond.of(unwanted.vxMetersPerSecond)).div(timing.dt());
    LinearAcceleration ayStop =
        MeasureMath.negate(MetersPerSecond.of(unwanted.vyMetersPerSecond)).div(timing.dt());
    AngularAcceleration alphaStop =
        MeasureMath.negate(RadiansPerSecond.of(unwanted.omegaRadiansPerSecond)).div(timing.dt());
    ax = MeasureMath.clamp(ax, axStop);
    ay = MeasureMath.clamp(ay, ayStop);
    alpha = MeasureMath.clamp(alpha, alphaStop);

    // apply friction
    Force fx = chassisMass.forceDueToAcceleration(ax);
    Force fy = chassisMass.forceDueToAcceleration(ay);
    Torque tau = chassisMass.torqueDueToAcceleration(alpha);
    DVector3 f = new DVector3(fx.in(Newtons), fy.in(Newtons), 0.0);
    DVector3 t = new DVector3(0, 0, tau.in(NewtonMeters));
    chassis.addForce(f);
    chassis.addTorque(t);

    Logger.recordOutput("Friction/TotalForce", new XY<>(fx, fy));
    Logger.recordOutput("Friction/TotalTorque", tau);
  }

  /**
   * Computes the small‐time‐step twist (dx, dy, dθ) from the change in chassis position and heading
   * since the last tick.
   */
  public Twist2d getTickTwist() {
    Pose2d curr = getChassisWorldPose2d();

    double dx = curr.getX() - lastPose.getX();
    double dy = curr.getY() - lastPose.getY();
    double dtheta = curr.getRotation().minus(lastPose.getRotation()).getRadians();
    double sin = Math.sin(-lastPose.getRotation().getRadians());
    double cos = Math.cos(-lastPose.getRotation().getRadians());

    double localDx = dx * cos - dy * sin;
    double localDy = dx * sin + dy * cos;
    lastPose = curr;
    return new Twist2d(localDx, localDy, dtheta);
  }

  /**
   * Gets the swerve modules.
   *
   * @return the current moduleSimulations.
   */
  public SimSwerveModule[] getModules() {
    return moduleSimulations;
  }

  /**
   * Gets the gyro.
   *
   * @return the current gyroSimulation.
   */
  public SimGyro getGyro() {
    return this.gyroSimulation;
  }

  /**
   * Gets the tipping manager.
   *
   * @return the current tippingManager.
   */
  public TippingManager getTippingManager() {
    return this.tippingManager;
  }

  /**
   * Uses {@link SimMotorController}'s to control the moduleSimulations.
   *
   * @param moduleId the id of the module.
   * @param driveController the drive controller.
   * @param steerController the steer controller.
   * @return this SimSwerve instance.
   */
  public SimSwerve withSetModuleControllers(
      int moduleId, SimMotorController driveController, SimMotorController steerController) {
    moduleSimulations[moduleId].teardown();
    moduleSimulations[moduleId] =
        new SimSwerveModule(
            robot,
            config,
            moduleId,
            Newtons.of(config.robotMassKg * 9.8).div(moduleSimulations.length),
            () -> getAdjustedRotorInertia(moduleId),
            driveController,
            steerController);
    return this;
  }

  /**
   * Gets the current sim timing.
   *
   * @return the current {@link SimEnvTiming}.
   */
  public SimEnvTiming timing() {
    return timing;
  }

  /** ODE4j near‑callback signature */
  private void nearCallback(DGeom geom1, DGeom geom2) {
    // First, let your tipping logic decide whether to filter/handle the contact
    if (tippingManager != null && tippingManager.isTipping()) {
      // any contacts it wants to generate for the chassis get created here
      int created = tippingManager.processCollision(geom1, geom2);
      if (created > 0) {
        // if you need to record or log anything, do it here
      }
      // return early if you want to prevent any default contact creation
      return;
    }
    // Otherwise, fall back to your normal wheel‑ground contacts
    defaultCollision(geom1, geom2);
  }

  /** Your existing contact generation routine for non‑tipping cases */
  private void defaultCollision(DGeom o1, DGeom o2) {
    DContactBuffer contacts = new DContactBuffer(4);
    int n = OdeHelper.collide(o1, o2, 4, contacts.getGeomBuffer());
    for (int i = 0; i < n; i++) {
      DContact contact = contacts.get(i);
      // configure contact.surface… etc
      DJoint j = OdeHelper.createContactJoint(getWorld(), contactGroup, contact);
      j.attach(o1.getBody(), o2.getBody());
    }
  }

  /** Clean up resources when simulation is done. */
  @Override
  public void teardown() {
    // Clean up contact joint group
    if (contactGroup != null) {
      contactGroup.empty();
    }
    for (int i = 1; i < moduleSimulations.length; i++) {
      moduleSimulations[i].teardown();
    }
    super.teardown();
  }
}
