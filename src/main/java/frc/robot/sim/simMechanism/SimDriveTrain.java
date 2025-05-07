package frc.robot.sim.simMechanism;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.extras.math.mathutils.GeomUtil;
import frc.robot.extras.util.FrcBody;
import frc.robot.sim.SimRobot;
import frc.robot.sim.configs.SimDriveTrainConfig;
import frc.robot.sim.configs.SimSwerveConfig;
import frc.robot.sim.simField.SimArena;
import frc.robot.sim.simField.SimArena.SimEnvTiming;
import frc.robot.sim.simMechanism.simSwerve.SimSwerve;
import org.littletonrobotics.junction.Logger;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;

/**
 * Abstract drivetrain simulation using ODE4j in full 3D.
 */
public class SimDriveTrain {
  public static final double kBumperCoF = 0.65;
  public static final double kBumperCoR = 0.005;

  private final OdeWorld ode;
  protected final org.ode4j.ode.DBody chassis;
  private final SimEnvTiming timing;

  @SuppressWarnings("unchecked")
  protected SimDriveTrain(SimDriveTrainConfig<?, ?> config, SimEnvTiming timing) {
    this.timing = timing;

    // 1) Create a 3D world and box‐shaped chassis body
    this.ode = new OdeWorld();
    double bumperHeight = 0.1; // a thin chassis in Z
    this.chassis = ode.createBoxBody(
      config.bumperLengthXMeters,
      config.bumperWidthYMeters,
      bumperHeight,
      config.robotMassKg
    );

    // 2) (Optional) set restitution & friction on the geom 
    //    (not shown: you could cast to DGeom and call setSurfaceFriction, setBounce, etc.)

    // 3) Teleport to origin initially
    setChassisWorldPose(new Pose3d(), true);
  }

  public void setChassisWorldPose(Pose3d robotPose, boolean resetVelocity) {
    // Position in 3D (Z=0)
    DVector3 pos = new DVector3(robotPose.getX(), robotPose.getY(), 0.0);
    chassis.setPosition(pos.get0(), pos.get1(), pos.get2());

    // Orientation about Z axis
    DQuaternionC q = GeomUtil.toOdeQuaternion(robotPose.getRotation());
    chassis.setQuaternion(q);

    if (resetVelocity) {
      chassis.setLinearVel(0, 0, 0);
      // zero out angular velocity vector
      chassis.setAngularVel(0, 0, 0);
    }

    Logger.recordOutput("Odometry/ChassisPose", getChassisWorldPose());
  }

  public void setChassisWorldSpeeds(ChassisSpeeds givenSpeeds) {
    // Linear XY, zero Z
    chassis.setLinearVel(
      givenSpeeds.vxMetersPerSecond,
      givenSpeeds.vyMetersPerSecond,
      0.0 // unfortunately, the chassis does not fly :(
    );
    // Angular velocity only about Z axis
    chassis.setAngularVel(0, 0, givenSpeeds.omegaRadiansPerSecond);
  }

  public Pose3d getChassisWorldPose() {
    // read 3D pos + quaternion, then extract Pose2d
    
    DQuaternionC quat = chassis.getQuaternion(); // {w, x, y, z}
    Rotation3d rot = GeomUtil.toWpilibRotation(quat);
    return new Pose3d(GeomUtil.toWpilibTranslation(chassis.getPosition()), rot);
  }

  public ChassisSpeeds getChassisWorldSpeeds() {
    double[] lin = chassis.getLinearVel().toDoubleArray();   // {vx, vy, vz}
    double[] ang = chassis.getAngularVel().toDoubleArray();  // {wx, wy, wz}
    // we only care about Z‐rotation rate
    return new ChassisSpeeds(lin[0], lin[1], ang[2]);
  }

  public void simTick() {
    Logger.recordOutput("Forces/DriveTrainForces/pose", getChassisWorldPose());
    Logger.recordOutput("Forces/DriveTrainForces/velocity", getChassisWorldSpeeds());
    // then step world
    ode.step(timing.dt().in(Seconds));
  }

  @SuppressWarnings("unchecked")
  public static <T extends SimDriveTrain, C extends SimDriveTrainConfig<T, C>> T createDriveTrain(
      SimRobot<T> robot, C config) {
    if (config instanceof SimSwerveConfig) {
      return (T) new SimSwerve((SimRobot<SimSwerve>) robot, (SimSwerveConfig) config);
    }
    throw new IllegalArgumentException("Unknown drivetrain configuration");
  }
}
