package frc.robot.sim.simMechanism;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.extras.math.mathutils.GeomUtil;
import frc.robot.sim.SimRobot;
import frc.robot.sim.configs.SimDriveTrainConfig;
import frc.robot.sim.configs.SimSwerveConfig;
import frc.robot.sim.simField.SimArena;
import frc.robot.sim.simField.SimArena.SimEnvTiming;
import frc.robot.sim.simMechanism.simSwerve.SimSwerve;
import org.littletonrobotics.junction.Logger;
import org.ode4j.math.DQuaternionC;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;

/** Abstract drivetrain simulation using ODE4j in full 3D, but exposes a 2D interface. */
public class SimDriveTrain {
  public static final double kBumperCoF = 0.65;
  public static final double kBumperCoR = 0.005;

  private final DSpace space;
  private final DWorld world;
  protected final DBody chassis;
  private final SimEnvTiming timing;

  private boolean chassisAdded = false;

  @SuppressWarnings("unchecked")
  protected SimDriveTrain(SimDriveTrainConfig<?, ?> config, SimArena arena) {
    this.timing = arena.timing;
    this.world = arena.getWorld();
    this.space = arena.getSpace();

    // Create a 3D box body (chassis)
    // double bumperHeight = 0.1; // thin Z dimension
    this.chassis = createBoxBody(world, config, timing);

    // Teleport to origin (Pose2d)
    setChassisWorldPose(new Pose2d(), true);
  }

  /**
   * Ensure the chassis geom is added exactly once (call from SimArena instead of manual
   * odeSpace.add).
   */
  public void addToSpace() {
    if (!chassisAdded) {
      // Grab the geom you created in createBoxBody()
      DBox boxGeom = (DBox) chassis.getFirstGeom();
      space.add(boxGeom);
      chassisAdded = true;
    }
  }

  /** Teleport chassis to given 2D pose (Z=0). */
  // TODO: Use Pose3d!
  public void setChassisWorldPose(Pose2d pose, boolean resetVel) {
    chassis.setPosition(pose.getX(), pose.getY(), 0.0);
    DQuaternionC q = GeomUtil.toOdeQuaternion(new Rotation3d(pose.getRotation()));
    chassis.setQuaternion(q);
    if (resetVel) {
      chassis.setLinearVel(0, 0, 0);
      chassis.setAngularVel(0, 0, 0);
    }
    Logger.recordOutput("Odometry/ChassisPose3d", getChassisWorldPose3d());
  }

  /** Instantly set linear (XY) and angular (Z) speeds. */
  public void setChassisWorldSpeeds(ChassisSpeeds speeds) {
    chassis.setLinearVel(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0.0);
    chassis.setAngularVel(0, 0, speeds.omegaRadiansPerSecond);
  }

  /** Read back current pose from the 3D body. */
  public Pose3d getChassisWorldPose3d() {
    return GeomUtil.toWpilibPose(chassis.getPosition(), chassis.getQuaternion());
  }

  // private static DBody createBoxBody(
  //     DWorld world, DSpace space, SimDriveTrainConfig<?, ?> config, SimEnvTiming timing) {
  //   DBody body = OdeHelper.createBody(world);
  //   // set mass and attach geom created in the *same* space
  //   DMass m = OdeHelper.createMass();
  //   m.setBox(1.0, config.bumperLengthXMeters / 2.0, config.bumperWidthYMeters / 2.0, 0.1 / 2.0);
  //   m.adjust(config.robotMassKg);
  //   body.setMass(m);
  //   DBox geom =
  //       OdeHelper.createBox(space, config.bumperLengthXMeters, config.bumperWidthYMeters, 0.1);
  //   geom.setBody(body);
  //   return body;
  // }

  private static DBody createBoxBody(
      DWorld world, SimDriveTrainConfig<?, ?> config, SimEnvTiming timing) {
    DBody body = OdeHelper.createBody(world);
    DMass m = OdeHelper.createMass();
    m.setBox(1.0, config.bumperLengthXMeters / 2.0, config.bumperWidthYMeters / 2.0, 0.1 / 2.0);
    m.adjust(config.robotMassKg);
    body.setMass(m);

    // Create the geom *un‑attached*
    DBox geom =
        OdeHelper.createBox(null, config.bumperLengthXMeters, config.bumperWidthYMeters, 0.1);
    geom.setBody(body);

    return body;
  }

  /** Re ad back current 2D pose from the 3D body. */
  public Pose2d getChassisWorldPose2d() {
    return getChassisWorldPose3d().toPose2d();
  }

  /** Read back current chassis speeds (vx, vy, ωz). */
  public ChassisSpeeds getChassisWorldSpeeds() {
    return GeomUtil.toWpilibChassisSpeeds(chassis.getLinearVel(), chassis.getAngularVel());
  }

  /** Step the physics world and log. */
  public void simTick() {
    Logger.recordOutput("Forces/DriveTrainPose2d", getChassisWorldPose2d());
    Logger.recordOutput("Forces/DriveTrainSpeeds", getChassisWorldSpeeds());
    // world.step(timing.dt.in(Seconds));
  }

  /** Factory for different drivetrain types. */
  @SuppressWarnings("unchecked")
  public static <T extends SimDriveTrain, C extends SimDriveTrainConfig<T, C>> T createDriveTrain(
      SimRobot<T> robot, C config) {
    if (config instanceof SimSwerveConfig) {
      return (T) new SimSwerve((SimRobot<SimSwerve>) robot, (SimSwerveConfig) config);
    }
    throw new IllegalArgumentException("Unknown drivetrain configuration");
  }

  public DSpace getSpace() {
    return space;
  }

  public DWorld getWorld() {
    return world;
  }

  public DBody getChassisBody() {
    return chassis;
  }
}
