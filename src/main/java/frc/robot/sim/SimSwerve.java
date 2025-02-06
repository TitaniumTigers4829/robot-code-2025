package frc.robot.sim;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.extras.util.mathutils.MeasureMath.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Torque;
import frc.robot.extras.util.RuntimeLog;
import frc.robot.extras.util.mathutils.MassMath.PhysicsMass;
import frc.robot.extras.util.mathutils.MeasureMath;
import frc.robot.extras.util.mathutils.MeasureMath.XY;
import frc.robot.sim.SimArena.SimEnvTiming;
import frc.robot.sim.configs.SimSwerveConfig;
import java.util.Arrays;
import org.dyn4j.geometry.Vector2;
import org.littletonrobotics.junction.Logger;

public class SimSwerve extends SimDriveTrain {
  protected final SimEnvTiming timing;
  protected final SimSwerveModule[] moduleSimulations;
  protected final SimGyro gyroSimulation;
  private final SimSwerveConfig config;
  private final SimRobot<SimSwerve> robot;
  private final PhysicsMass chassisMass;
  private final SwerveDriveKinematics kinematics;

  private MomentOfInertia rotorInertia;

  private final MomentOfInertia rotorInertiaWhenTranslating;
  private final MomentOfInertia rotorInertiaWhenRotating;

  /**
   * Creates a Swerve Drive Simulation.
   *
   * <p>This constructor initializes a swerve drive simulation with the given robot mass, bumper
   * dimensions, module simulations, module translations, gyro simulation, and initial pose on the
   * field.
   *
   * @param config a {@link SimSwerveConfig} instance containing the configurations of * this
   *     drivetrain
   */
  public SimSwerve(SimRobot<SimSwerve> robot, SimSwerveConfig config) {
    super(config, robot.timing());
    RuntimeLog.debug("initializing sim swerve");
    this.robot = robot;
    this.timing = robot.timing();
    this.config = config;
    this.moduleSimulations = new SimSwerveModule[config.moduleTranslations.length];
    final Force gravityForceOnEachModule =
        Newtons.of(config.robotMassKg * 9.8).div(moduleSimulations.length);
    for (int i = 0; i < moduleSimulations.length; i++) {
      RuntimeLog.debug("initializing module simulation " + i);
      moduleSimulations[i] =
          new SimSwerveModule(
              robot,
              config,
              i,
              gravityForceOnEachModule,
              () -> rotorInertia,
              SimMotorController.none(),
              SimMotorController.none());
    }

    RuntimeLog.debug("initializing gyro sim");
    this.gyroSimulation = new SimGyro(timing, config.gyroConfig);

    this.chassisMass =
        new PhysicsMass(Kilograms.of(config.robotMassKg), KilogramSquareMeters.of(config.robotMoI));
    this.kinematics = new SwerveDriveKinematics(config.moduleTranslations);

    // pre-calculate the rotor inertia for the chassis when translating and rotating
    // to interpolate between them based on the propulsion forces
    RuntimeLog.debug("calculating robot values");
    final Distance wheelRadius = Meters.of(config.swerveModuleConfig.wheelsRadiusMeters);
    final Distance wheelBase = XY.of(config.moduleTranslations[0]).magnitude();
    final Mass rotationalMass =
        div(chassisMass.moi(), wheelBase.times(wheelBase)).div(moduleSimulations.length);
    this.rotorInertiaWhenTranslating =
        times(chassisMass.mass().div(moduleSimulations.length), wheelRadius.times(wheelRadius));
    this.rotorInertiaWhenRotating = times(rotationalMass, wheelRadius.times(wheelRadius));
    this.rotorInertia = rotorInertiaWhenTranslating;

    RuntimeLog.debug("created swerve drive simulation");
  }

  @Override
  protected void simTick() {
    simulateModulePropulsion();
    simulateModuleFriction();
    gyroSimulation.updateSimulationSubTick(
        this.getChassisWorldPose().getRotation().getMeasure(), this.getTickTwist());
    Logger.recordOutput("Odometry/ChassisPose", getChassisWorldPose());
    super.simTick();
  }

  private void simulateModuleFriction() {
    final Rotation2d chassisRotation = getChassisWorldPose().getRotation();
    final ChassisSpeeds chassisSpeeds = this.getChassisWorldSpeeds();

    // sum up all the acceleration due to friction from each module
    LinearAcceleration xFrictionAccel = MetersPerSecondPerSecond.zero();
    LinearAcceleration yFrictionAccel = MetersPerSecondPerSecond.zero();
    AngularAcceleration angularFrictionAccel = RadiansPerSecondPerSecond.zero();
    for (int i = 0; i < moduleSimulations.length; i++) {
      final XY<Force> frictionForce = moduleSimulations[i].friction(chassisSpeeds, chassisRotation);
      final var pack =
          chassisMass.accelerationsDueToForce(
              frictionForce, XY.of(moduleSimulations[i].translation().rotateBy(chassisRotation)));
      xFrictionAccel = xFrictionAccel.plus(pack.getFirst().x());
      yFrictionAccel = yFrictionAccel.plus(pack.getFirst().y());
      angularFrictionAccel = angularFrictionAccel.plus(pack.getSecond());

      Logger.recordOutput("Forces/Friction/module" + i + "/frictionForce", frictionForce);
      Logger.recordOutput("Forces/Friction/module" + i + "/xyFrictionAccel", pack.getFirst());
      Logger.recordOutput("Forces/Friction/module" + i + "/angularFrictionAccel", pack.getSecond());
    }

    // clamp the friction acceleration to prevent the robot from accelerating in the opposite
    // direction
    final ChassisSpeeds wheelSpeeds =
        kinematics.toChassisSpeeds(
            Arrays.stream(moduleSimulations)
                .map(SimSwerveModule::state)
                .toArray(SwerveModuleState[]::new));
    ChassisSpeeds newWheelSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(wheelSpeeds, chassisRotation);
    final ChassisSpeeds unwantedSpeeds = newWheelSpeeds.minus(chassisSpeeds);

    Logger.recordOutput("Friction/wheelSpeeds", wheelSpeeds);
    Logger.recordOutput("Friction/unwantedSpeeds", unwantedSpeeds);

    final LinearAcceleration xAccelNeededToStop =
        MeasureMath.negate(MetersPerSecond.of(unwantedSpeeds.vxMetersPerSecond)).div(timing.dt());
    final LinearAcceleration yAccelNeededToStop =
        MeasureMath.negate(MetersPerSecond.of(unwantedSpeeds.vyMetersPerSecond)).div(timing.dt());
    final AngularAcceleration angularAccelNeededToStop =
        MeasureMath.negate(RadiansPerSecond.of(unwantedSpeeds.omegaRadiansPerSecond))
            .div(timing.dt());
    Logger.recordOutput("Forces/Friction/xFrictionAccelPreClamp", xFrictionAccel);
    Logger.recordOutput("Forces/Friction/yFrictionAccelPreClamp", yFrictionAccel);
    xFrictionAccel = MeasureMath.clamp(xFrictionAccel, xAccelNeededToStop);
    yFrictionAccel = MeasureMath.clamp(yFrictionAccel, yAccelNeededToStop);
    angularFrictionAccel = MeasureMath.clamp(angularFrictionAccel, angularAccelNeededToStop);

    Logger.recordOutput("Forces/Friction/xAccelNeededToStop", xAccelNeededToStop);
    Logger.recordOutput("Forces/Friction/yAccelNeededToStop", yAccelNeededToStop);
    Logger.recordOutput("Forces/Friction/angularAccelNeededToStop", angularAccelNeededToStop);
    Logger.recordOutput("Forces/Friction/xFrictionAccel", xFrictionAccel);
    Logger.recordOutput("Forces/Friction/yFrictionAccel", yFrictionAccel);
    Logger.recordOutput("Forces/Friction/angularFrictionAccel", angularFrictionAccel);

    // convert the friction acceleration to forces and torques and apply them to the chassis
    final Force xFrictionForce = chassisMass.forceDueToAcceleration(xFrictionAccel);
    final Force yFrictionForce = chassisMass.forceDueToAcceleration(yFrictionAccel);
    final Torque angularFrictionTorque = chassisMass.torqueDueToAcceleration(angularFrictionAccel);
    chassis.applyForce(new Vector2(xFrictionForce.in(Newtons), yFrictionForce.in(Newtons)));
    chassis.applyTorque(angularFrictionTorque.in(NewtonMeters));
  }

  private void simulateModulePropulsion() {
    final Rotation2d chassisRotation = getChassisWorldPose().getRotation();

    // apply propulsion forces to chassis
    Force propulsionForceTotal = Newtons.zero();
    Force propulsionForceX = Newtons.zero();
    Force propulsionForceY = Newtons.zero();
    Torque propulsionTorque = NewtonMeters.zero();
    for (final SimSwerveModule module : moduleSimulations) {
      final XY<Distance> forcePosition = XY.of(module.translation().rotateBy(chassisRotation));
      final XY<Force> propulsion = module.force(chassisRotation);

      propulsionForceTotal = propulsionForceTotal.plus(propulsion.magnitude());

      final var pack = chassisMass.forcesDueToOffsetForces(propulsion, forcePosition);
      propulsionForceX = propulsionForceX.plus(pack.getFirst().x());
      propulsionForceY = propulsionForceY.plus(pack.getFirst().y());
      propulsionTorque = propulsionTorque.plus(pack.getSecond());

      Logger.recordOutput("Forces/Propulsion/module" + module.id() + "/forces", pack.getFirst());
      Logger.recordOutput("Forces/Propulsion/module" + module.id() + "/torque", pack.getSecond());
    }

    // The rotor inertia can very depending on how the modules are driving the chassis.
    // How the propulsion impacts the chassis determines how the rotor inertia is calculated.
    // Figure out what proportion of the propulsion force is in the x and y axis versus the yaw
    // axis.
    // This will determine how the rotor inertia is calculated.
    final double propulsionForceXMag = propulsionForceX.in(Newtons);
    final double propulsionForceYMag = propulsionForceY.in(Newtons);
    final double propulsionForceTotalMag = propulsionForceTotal.in(Newtons);
    if (Math.abs(propulsionForceTotalMag) > 1e-3) {
      final double xPropulsionRatio = propulsionForceXMag / propulsionForceTotalMag;
      final double yPropulsionRatio = propulsionForceYMag / propulsionForceTotalMag;
      final double translationRatio = Math.hypot(xPropulsionRatio, yPropulsionRatio);

      Logger.recordOutput("Forces/RotorInertia/xPropulsionRatio", xPropulsionRatio);
      Logger.recordOutput("Forces/RotorInertia/yPropulsionRatio", yPropulsionRatio);
      Logger.recordOutput("Forces/RotorInertia/translationRatio", translationRatio);

      // Calculate the rotor inertia based on the propulsion ratios
      this.rotorInertia =
          rotorInertiaWhenTranslating
              .times(translationRatio)
              .plus(rotorInertiaWhenRotating.times(1 - translationRatio));
    } else {
      this.rotorInertia = rotorInertiaWhenTranslating;

      Logger.recordOutput("Forces/RotorInertia/xPropulsionRatio", 0.0);
      Logger.recordOutput("Forces/RotorInertia/yPropulsionRatio", 0.0);
      Logger.recordOutput("Forces/RotorInertia/translationRatio", 0.0);
    }

    Logger.recordOutput("Forces/Propulsion/propulsionTorque", propulsionTorque);
    Logger.recordOutput(
        "Forces/Propulsion/propulsionForce", new XY<>(propulsionForceX, propulsionForceY));

    chassis.applyForce(new Vector2(propulsionForceX.in(Newtons), propulsionForceY.in(Newtons)));
    chassis.applyTorque(propulsionTorque.in(NewtonMeters));
  }

  public SimSwerveModule[] getModules() {
    return moduleSimulations;
  }

  public SimGyro getGyro() {
    return this.gyroSimulation;
  }

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

  public SimEnvTiming timing() {
    return timing;
  }
}
