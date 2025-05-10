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
import frc.robot.extras.math.mathutils.MeasureMath;
import frc.robot.extras.math.mathutils.MassMath.PhysicsMass;
import frc.robot.extras.math.mathutils.MeasureMath.XY;
import frc.robot.sim.SimRobot;
import frc.robot.sim.configs.SimSwerveConfig;
import frc.robot.sim.simController.SimMotorController;
import frc.robot.sim.simField.SimArena.SimEnvTiming;
import frc.robot.sim.simMechanism.SimDriveTrain;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;
import org.ode4j.math.DVector3;

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

  public SimSwerve(SimRobot<SimSwerve> robot, SimSwerveConfig config) {
    super(config, robot.timing());
    this.robot = robot;
    this.timing = robot.timing();
    this.config = config;
    // initialize modules & gyro
    this.moduleSimulations = new SimSwerveModule[config.moduleTranslations.length];
    Force gravityPerModule = Newtons.of(config.robotMassKg * 9.81)
                                    .div(moduleSimulations.length);
    for (int i = 0; i < moduleSimulations.length; i++) {
      moduleSimulations[i] = new SimSwerveModule(
        robot, config, i, gravityPerModule,
        () -> rotorInertia,
        SimMotorController.none(), SimMotorController.none()
      );
    }
    this.gyroSimulation = new SimGyro(timing, config.gyroConfig);

    // chassis mass/inertia
    this.chassisMass = new PhysicsMass(
      Kilograms.of(config.robotMassKg),
      KilogramSquareMeters.of(config.robotMoI)
    );
    this.kinematics = new SwerveDriveKinematics(config.moduleTranslations);

    // precompute rotor inertias
    Distance wheelRadius = Meters.of(config.swerveModuleConfig.wheelsRadiusMeters);
    Distance wheelBase   = XY.of(config.moduleTranslations[0]).magnitude();
    var rotationalMass    = MeasureMath.div(chassisMass.moi(), wheelBase.times(wheelBase)).div(moduleSimulations.length);
    this.rotorInertiaWhenTranslating = MeasureMath.times(chassisMass.mass().div(moduleSimulations.length), wheelRadius.times(wheelRadius));
    this.rotorInertiaWhenRotating = MeasureMath.times(rotationalMass, wheelRadius.times(wheelRadius));
    this.rotorInertia = rotorInertiaWhenTranslating;
  }

  @Override
  public void simTick() {
    simulateModulePropulsion();
    simulateModuleFriction();

    // update gyro
    gyroSimulation.updateSimulationSubTick(
      getChassisWorldPose2d().getRotation().getMeasure(),
      getTickTwist()
    );

    Logger.recordOutput("Odometry/ChassisPose", getChassisWorldPose2d());
    super.simTick();
  }

  private void simulateModulePropulsion() {
    Rotation2d rot = getChassisWorldPose2d().getRotation();
    Force totalForceX = Newtons.zero();
    Force totalForceY = Newtons.zero();
    Torque totalTorque = NewtonMeters.zero();

    for (SimSwerveModule mod : moduleSimulations) {
      XY<Distance> pos = XY.of(mod.translation().rotateBy(rot));
      XY<Force> propulsion = mod.force(rot);
      var pack = chassisMass.forcesDueToOffsetForces(propulsion, pos);

      totalForceX  = totalForceX.plus(pack.getFirst().x());
      totalForceY  = totalForceY.plus(pack.getFirst().y());
      totalTorque  = totalTorque.plus(pack.getSecond());

      Logger.recordOutput("Propulsion/Module" + mod.id() + "/force", pack.getFirst());
      Logger.recordOutput("Propulsion/Module" + mod.id() + "/torque", pack.getSecond());
    }

    // apply to body in 3D
    DVector3 f = new DVector3(
      totalForceX.in(Newtons),
      totalForceY.in(Newtons),
      0.0
    );
    DVector3 τ = new DVector3(0.0, 0.0, totalTorque.in(NewtonMeters));
    chassis.addForce(f);
    chassis.addTorque(τ);

    Logger.recordOutput("Propulsion/TotalForce", new XY<>(totalForceX, totalForceY));
    Logger.recordOutput("Propulsion/TotalTorque", totalTorque);
  }

  private void simulateModuleFriction() {
    Rotation2d rot = getChassisWorldPose2d().getRotation();
    ChassisSpeeds speeds = getChassisWorldSpeeds();

    LinearAcceleration ax = MetersPerSecondPerSecond.zero();
    LinearAcceleration ay = MetersPerSecondPerSecond.zero();
    AngularAcceleration α = RadiansPerSecondPerSecond.zero();

    for (int i = 0; i < moduleSimulations.length; i++) {
      XY<Force> friction = moduleSimulations[i].friction(speeds, rot);
      Pair<XY<LinearAcceleration>, AngularAcceleration> pack = chassisMass.accelerationsDueToForce(
        friction, XY.of(moduleSimulations[i].translation().rotateBy(rot))
      );
      ax = ax.plus(pack.getFirst().x());
      ay = ay.plus(pack.getFirst().y());
      α  = α.plus(pack.getSecond());

      Logger.recordOutput("Friction/Module" + i + "/force", friction);
      Logger.recordOutput("Friction/Module" + i + "/linearAccel", pack.getFirst());
      Logger.recordOutput("Friction/Module" + i + "/angularAccel", pack.getSecond());
    }

    // clamp to no-reverse
    ChassisSpeeds currentWs = kinematics.toChassisSpeeds(
      Arrays.stream(moduleSimulations)
            .map(SimSwerveModule::state)
            .toArray(SwerveModuleState[]::new)
    );
    ChassisSpeeds unwanted = currentWs.minus(speeds);

    var axStop = MetersPerSecond.of(-unwanted.vxMetersPerSecond)
                   .div(timing.dt());
    var ayStop = MetersPerSecond.of(-unwanted.vyMetersPerSecond)
                   .div(timing.dt());
    var αStop  = RadiansPerSecond.of(-unwanted.omegaRadiansPerSecond)
                   .div(timing.dt());

    ax = MeasureMath.clamp(ax, axStop);
    ay = MeasureMath.clamp(ay, ayStop);
    α  = MeasureMath.clamp(α, αStop);

    // convert to forces & torque, apply
    Force fx = chassisMass.forceDueToAcceleration(ax);
    Force fy = chassisMass.forceDueToAcceleration(ay);
    Torque τ  = chassisMass.torqueDueToAcceleration(α);

    DVector3 f = new DVector3(fx.in(Newtons), fy.in(Newtons), 0.0);
    DVector3 t = new DVector3(0, 0, τ.in(NewtonMeters));
    chassis.addForce(f);
    chassis.addTorque(t);

    Logger.recordOutput("Friction/TotalForce", new XY<>(fx, fy));
    Logger.recordOutput("Friction/TotalTorque", τ);
  }
  
// **
//  * Computes the small‐time‐step twist (dx, dy, dθ) from the change in chassis
//  * position and heading since the last tick.
//  */
public Twist2d getTickTwist() {
  // Get the chassis’s change in position in robot‐local coordinates:
  // You’ll need to track the previous pose each tick.
  Pose2d prev = new Pose2d();   // TODO: make lastPose variable   // store this at end of simTick()
  Pose2d curr = getChassisWorldPose2d();

  // Compute the delta in world frame:
  double dx = curr.getX() - prev.getX();
  double dy = curr.getY() - prev.getY();
  double dtheta = curr.getRotation().minus(prev.getRotation()).getRadians();

  // Rotate that world‐delta back into robot’s local frame at prev heading:
  double sin = Math.sin(-prev.getRotation().getRadians());
  double cos = Math.cos(-prev.getRotation().getRadians());
  double localDx =  dx * cos - dy * sin;
  double localDy =  dx * sin + dy * cos;

  // Update lastPose for next tick
  prev = curr;

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
            () -> rotorInertia,
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
}
