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

  private Pose2d lastPose;

  public SimSwerve(SimRobot<SimSwerve> robot, SimSwerveConfig config) {
    super(config, robot.arena());
    this.robot = robot;
    this.timing = robot.timing();
    this.config = config;
    // initialize modules & gyro
    this.moduleSimulations = new SimSwerveModule[config.moduleTranslations.length];
    Force gravityPerModule = Newtons.of(config.robotMassKg * 9.81).div(moduleSimulations.length);
    for (int i = 0; i < moduleSimulations.length; i++) {
      moduleSimulations[i] =
          new SimSwerveModule(
              robot,
              config,
              i,
              gravityPerModule,
              () -> rotorInertia,
              SimMotorController.none(),
              SimMotorController.none());
    }
    this.gyroSimulation = new SimGyro(timing, config.gyroConfig);

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
  }

  @Override
  public void simTick() {
    simulateModulePropulsion();
    simulateModuleFriction();

    // update gyro
    gyroSimulation.updateSimulationSubTick(
        getChassisWorldPose2d().getRotation().getMeasure(), getTickTwist());

    Logger.recordOutput("Odometry/ChassisPose", getChassisWorldPose2d());
    super.simTick();
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
    var axStop =
        MeasureMath.negate(MetersPerSecond.of(unwanted.vxMetersPerSecond)).div(timing.dt());
    var ayStop =
        MeasureMath.negate(MetersPerSecond.of(unwanted.vyMetersPerSecond)).div(timing.dt());
    var alphaStop =
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

  /*
   **
   * Computes the smaFll‐time‐step twist (dx, dy, dθ) from the change in chassis
   * position and heading since the last tick.
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
