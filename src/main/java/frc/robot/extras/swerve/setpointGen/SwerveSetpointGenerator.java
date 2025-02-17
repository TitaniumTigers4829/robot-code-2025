package frc.robot.extras.swerve.setpointGen;

import static frc.robot.extras.swerve.setpointGen.SPGCalcs.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Swerve setpoint generatoR that has been passed around so many times its hard to keep track, just
 * know i didn't write most the logic in this code that credit goes to 254 and mjansen
 *
 * <p>Takes a prior setpoint, a desired setpoint, and outputs a new setpoint that respects all the
 * kinematic constraints on module rotation and wheel velocity/torque, as well as preventing any
 * forces acting on a module's wheel from exceeding the force of friction.
 */
public class SwerveSetpointGenerator {
  private static final ChassisSpeeds ZERO_SPEEDS = new ChassisSpeeds();

  private static final int NUM_MODULES = SPGCalcs.NUM_MODULES;

  private final SwerveDriveKinematics kinematics;
  private final Translation2d[] moduleLocations;
  private final DCMotor driveMotor;
  private final double driveCurrentLimitAmps,
      maxDriveVelocityMPS,
      maxSteerVelocityRadsPerSec,
      massKg,
      moiKgMetersSquared,
      wheelRadiusMeters,
      wheelFrictionForce,
      // maxTorqueFriction,
      torqueLoss;

  public SwerveSetpointGenerator(
      final Translation2d[] moduleLocations,
      final DCMotor driveMotor,
      final DCMotor angleMotor,
      final double driveCurrentLimitAmps,
      final double massKg,
      final double moiKgMetersSquared,
      final double wheelDiameterMeters,
      final double wheelCoF,
      final double torqueLoss) {

    if (moduleLocations.length != NUM_MODULES) {
      throw new IllegalArgumentException("Module locations must have 4 elements");
    }

    this.driveMotor = driveMotor;
    this.driveCurrentLimitAmps = driveCurrentLimitAmps;
    this.maxSteerVelocityRadsPerSec = angleMotor.freeSpeedRadPerSec;
    kinematics = new SwerveDriveKinematics(moduleLocations);
    this.moduleLocations = moduleLocations;
    this.massKg = massKg;
    this.moiKgMetersSquared = moiKgMetersSquared;
    this.wheelRadiusMeters = wheelDiameterMeters / 2;
    this.maxDriveVelocityMPS = driveMotor.freeSpeedRadPerSec * wheelRadiusMeters;

    wheelFrictionForce = wheelCoF * ((massKg / 4) * 9.81);
    // maxTorqueFriction = this.wheelFrictionForce * wheelRadiusMeters;
    this.torqueLoss = torqueLoss;
  }

  // alot of work was done to reduce allocations in this hot loop,
  // migrating everything over to a vars object that gets reused
  // was the best way to do this.
  private static final LocalVars VARS_TEMPLATE = new LocalVars();

  /**
   * Generate a new setpoint. Note: Do not discretize ChassisSpeeds passed into or returned from
   * this method. This method will discretize the speeds for you.
   *
   * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the previous
   *     iteration setpoint instead of the actual measured/estimated kinematic state.
   * @param desiredRobotRelativeSpeeds The desired state of motion, such as from the driver sticks
   *     or a path following algorithm.
   * @param dt The loop time.
   * @return A Setpoint object that satisfies all the kinematic/friction limits while converging to
   *     desiredState quickly.
   */
  public SwerveSetpoint generateSetpoint(
      final SwerveSetpoint prevSetpoint, ChassisSpeeds desiredRobotRelativeSpeeds, double dt) {
    // https://github.com/wpilibsuite/allwpilib/issues/7332
    SwerveModuleState[] desiredModuleStates =
        kinematics.toSwerveModuleStates(desiredRobotRelativeSpeeds);
    // Make sure desiredState respects velocity limits.
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, maxDriveVelocityMPS);
    desiredRobotRelativeSpeeds = kinematics.toChassisSpeeds(desiredModuleStates);

    final LocalVars vars = VARS_TEMPLATE.reset();
    vars.dt = dt;
    vars.prevSpeeds = prevSetpoint.chassisSpeeds();
    vars.desiredSpeeds = desiredRobotRelativeSpeeds;
    vars.desiredModuleStates = desiredModuleStates;
    vars.prevModuleStates = prevSetpoint.moduleStates();
    vars.dx =
        desiredRobotRelativeSpeeds.vxMetersPerSecond
            - prevSetpoint.chassisSpeeds().vxMetersPerSecond;
    vars.dy =
        desiredRobotRelativeSpeeds.vyMetersPerSecond
            - prevSetpoint.chassisSpeeds().vyMetersPerSecond;
    vars.dtheta =
        desiredRobotRelativeSpeeds.omegaRadiansPerSecond
            - prevSetpoint.chassisSpeeds().omegaRadiansPerSecond;
    vars.minS = 1.0;

    checkNeedToSteer(vars);
    makeVectors(vars);

    // if (vars.allModulesShouldFlip
    //         && !epsilonEquals(prevSetpoint.chassisSpeeds(), ZERO_SPEEDS)
    //         && !epsilonEquals(desiredRobotRelativeSpeeds, ZERO_SPEEDS)) {
    //     // It will (likely) be faster to stop the robot, rotate the modules in place to the
    // complement
    //     // of the desired angle, and accelerate again.
    //     return generateSetpoint(prevSetpoint, ZERO_SPEEDS, dt);
    // }

    solveSteering(vars);

    solveDriving(vars);

    ChassisSpeeds retSpeeds =
        new ChassisSpeeds(
            vars.prevSpeeds.vxMetersPerSecond + vars.minS * vars.dx,
            vars.prevSpeeds.vyMetersPerSecond + vars.minS * vars.dy,
            vars.prevSpeeds.omegaRadiansPerSecond + vars.minS * vars.dtheta);
    ChassisSpeeds.discretize(retSpeeds, dt);

    double chassisAccelX = (retSpeeds.vxMetersPerSecond - vars.prevSpeeds.vxMetersPerSecond) / dt;
    double chassisAccelY = (retSpeeds.vyMetersPerSecond - vars.prevSpeeds.vyMetersPerSecond) / dt;
    double angularAccel =
        (retSpeeds.omegaRadiansPerSecond - vars.prevSpeeds.omegaRadiansPerSecond) / dt;

    SwerveModuleState[] retStates = kinematics.toSwerveModuleStates(retSpeeds);
    SwerveModuleState[] accelStates =
        kinematics.toSwerveModuleStates(
            new ChassisSpeeds(chassisAccelX, chassisAccelY, angularAccel));

    AdvancedSwerveModuleState[] outputStates = new AdvancedSwerveModuleState[NUM_MODULES];
    for (int m = 0; m < NUM_MODULES; m++) {
      retStates[m].optimize(vars.prevModuleStates[m].angle);
      double steerVelocity =
          (vars.prevModuleStates[m].angle.getRadians() - retStates[m].angle.getRadians()) / dt;
      outputStates[m] =
          new AdvancedSwerveModuleState(
              retStates[m].speedMetersPerSecond,
              retStates[m].angle,
              steerVelocity,
              accelStates[m].speedMetersPerSecond);
    }

    // log("output",
    return new SwerveSetpoint(retSpeeds, outputStates);
  }

  public SwerveSetpoint generateSimpleSetpoint(
      final SwerveSetpoint prevSetpoint, ChassisSpeeds desiredRobotRelativeSpeeds, double dt) {
    AdvancedSwerveModuleState[] outputStates = new AdvancedSwerveModuleState[NUM_MODULES];
    SwerveModuleState[] desiredModuleStates =
        kinematics.toSwerveModuleStates(desiredRobotRelativeSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, maxDriveVelocityMPS);
    for (int m = 0; m < NUM_MODULES; m++) {
      desiredModuleStates[m].optimize(prevSetpoint.moduleStates()[m].angle);
      outputStates[m] = AdvancedSwerveModuleState.fromBase(desiredModuleStates[m]);
    }

    return new SwerveSetpoint(kinematics.toChassisSpeeds(desiredModuleStates), outputStates);
  }

  private static void checkNeedToSteer(LocalVars vars) {
    if (epsilonEquals(vars.desiredSpeeds, ZERO_SPEEDS)) {
      vars.needToSteer = false;
      for (int m = 0; m < NUM_MODULES; m++) {
        vars.desiredModuleStates[m].angle = vars.prevModuleStates[m].angle;
        vars.desiredModuleStates[m].speedMetersPerSecond = 0.0;
      }
    }
  }

  private static double rotateBy(double rad, double otherCos, double otherSin) {
    return Math.atan2(
        Math.sin(rad) * otherCos + Math.cos(rad) * otherSin,
        Math.cos(rad) * otherCos - Math.sin(rad) * otherSin);
  }

  private static double requiredRotation(double prevRadians, double desiredRads) {
    // this looks messy without using Rotation2d methods.
    // this is roughly equivalent to:
    //
    // double r = vars.prev[m].rot2d().unaryMinus()
    //    .rotateBy(vars.desired[m].rot2d()).getRadians();
    double unaryMinusPrevRads = -prevRadians;
    return rotateBy(unaryMinusPrevRads, Math.cos(desiredRads), Math.sin(desiredRads));
  }

  private static void makeVectors(LocalVars vars) {
    for (int m = 0; m < NUM_MODULES; m++) {
      vars.prev[m].applyModuleState(vars.prevModuleStates[m]);
      vars.desired[m].applyModuleState(vars.desiredModuleStates[m]);
      if (vars.allModulesShouldFlip) {
        double requiredRots = requiredRotation(vars.prev[m].radians(), vars.desired[m].radians());
        if (flipHeading(requiredRots)) {
          vars.allModulesShouldFlip = false;
        }
      }
    }
  }

  private void solveSteering(LocalVars vars) {
    // In cases where an individual module is stopped, we want to remember the right steering angle
    // to command (since inverse kinematics doesn't care about angle, we can be opportunistically
    // lazy).
    // Enforce steering velocity limits. We do this by taking the derivative of steering angle at
    // the current angle, and then backing out the maximum interpolant between start and goal
    // states. We remember the minimum across all modules, since that is the active constraint.
    for (int m = 0; m < NUM_MODULES; m++) {
      if (!vars.needToSteer) {
        vars.steeringOverride[m] = vars.prevModuleStates[m].angle;
        continue;
      }

      double maxThetaStep = vars.dt * maxSteerVelocityRadsPerSec;

      if (epsilonEquals(vars.prevModuleStates[m].speedMetersPerSecond, 0.0)) {
        // If module is stopped, we know that we will need to move straight to the final steering
        // angle, so limit based purely on rotation in place.
        if (epsilonEquals(vars.desiredModuleStates[m].speedMetersPerSecond, 0.0)) {
          // Goal angle doesn't matter. Just leave module at its current angle.
          vars.steeringOverride[m] = vars.prevModuleStates[m].angle;
          continue;
        }

        double requiredRots = requiredRotation(vars.prev[m].radians(), vars.desired[m].radians());
        if (flipHeading(requiredRots)) {
          requiredRots =
              rotateBy(requiredRots, Rotation2d.k180deg.getCos(), Rotation2d.k180deg.getSin());
        }

        // radians bounds to +/- Pi.
        final double numStepsNeeded = Math.abs(requiredRots) / maxThetaStep;

        if (numStepsNeeded <= 1.0) {
          // Steer directly to goal angle.
          vars.steeringOverride[m] = vars.desiredModuleStates[m].angle;
        } else {
          // Adjust steering by max_theta_step.
          // there really is no way to avoid this allocation.
          Rotation2d adjusted =
              vars.prevModuleStates[m].angle.rotateBy(
                  Rotation2d.fromRadians(Math.signum(requiredRots) * maxThetaStep));
          vars.steeringOverride[m] = adjusted;
          vars.minS = 0.0;
        }
        continue;
      }
      if (vars.minS == 0.0) {
        // s can't get any lower. Save some CPU.
        continue;
      }

      double maxS =
          findSteeringMaxS(
              vars.prev[m].vx,
              vars.prev[m].vy,
              vars.prev[m].radians(),
              vars.desired[m].vx,
              vars.desired[m].vy,
              vars.desired[m].radians(),
              maxThetaStep);
      vars.minS = Math.min(vars.minS, maxS);
    }
  }

  private void solveDriving(LocalVars vars) {
    // Enforce drive wheel torque limits
    Translation2d chassisForceVec = Translation2d.kZero;
    double chassisTorque = 0.0;
    for (int m = 0; m < NUM_MODULES; m++) {
      double lastVelRadPerSec = vars.prevModuleStates[m].speedMetersPerSecond / wheelRadiusMeters;
      // Use the current battery voltage since we won't be able to supply 12v if the
      // battery is sagging down to 11v, which will affect the max torque output
      double currentDraw =
          driveMotor.getCurrent(Math.abs(lastVelRadPerSec), RobotController.getInputVoltage());
      currentDraw = Math.min(currentDraw, driveCurrentLimitAmps);
      double moduleTorque = driveMotor.getTorque(currentDraw);

      double prevSpeed = vars.prevModuleStates[m].speedMetersPerSecond;
      vars.desiredModuleStates[m].optimize(vars.prevModuleStates[m].angle);
      double desiredSpeed = vars.desiredModuleStates[m].speedMetersPerSecond;

      int forceSign;
      Rotation2d forceAngle = vars.prevModuleStates[m].angle;
      if (epsilonEquals(prevSpeed, 0.0)
          || (prevSpeed > 0 && desiredSpeed >= prevSpeed)
          || (prevSpeed < 0 && desiredSpeed <= prevSpeed)) {
        // Torque loss will be fighting motor
        moduleTorque -= torqueLoss;
        forceSign = 1; // Force will be applied in direction of module
        if (prevSpeed < 0) {
          forceAngle = forceAngle.plus(Rotation2d.k180deg);
        }
      } else {
        // Torque loss will be helping the motor
        moduleTorque += torqueLoss;
        forceSign = -1; // Force will be applied in opposite direction of module
        if (prevSpeed > 0) {
          forceAngle = forceAngle.plus(Rotation2d.k180deg);
        }
      }

      // Limit torque to prevent wheel slip
      moduleTorque = Math.min(moduleTorque, wheelFrictionForce * wheelRadiusMeters);

      double forceAtCarpet = moduleTorque / wheelRadiusMeters;
      Translation2d moduleForceVec = new Translation2d(forceAtCarpet * forceSign, forceAngle);

      // Add the module force vector to the chassis force vector
      chassisForceVec = chassisForceVec.plus(moduleForceVec);

      // Calculate the torque this module will apply to the chassis
      Rotation2d angleToModule = moduleLocations[m].getAngle();
      Rotation2d theta = moduleForceVec.getAngle().minus(angleToModule);
      chassisTorque += forceAtCarpet * moduleLocations[m].getNorm() * theta.getSin();
    }

    Translation2d chassisAccelVec = chassisForceVec.div(massKg);
    double chassisAngularAccel = chassisTorque / moiKgMetersSquared;

    // Use kinematics to convert chassis accelerations to module accelerations
    ChassisSpeeds chassisAccel =
        new ChassisSpeeds(chassisAccelVec.getX(), chassisAccelVec.getY(), chassisAngularAccel);
    var accelStates = kinematics.toSwerveModuleStates(chassisAccel);

    for (int m = 0; m < NUM_MODULES; m++) {
      if (vars.minS == 0.0) {
        // No need to carry on.
        break;
      }

      double maxVelStep = Math.abs(accelStates[m].speedMetersPerSecond * vars.dt);

      double vxMinS;
      double vyMinS;
      if (vars.minS == 1.0) {
        vxMinS = vars.desired[m].vx;
        vyMinS = vars.desired[m].vy;
      } else {
        vxMinS = (vars.desired[m].vx - vars.prev[m].vx) * vars.minS + vars.prev[m].vx;
        vyMinS = (vars.desired[m].vy - vars.prev[m].vy) * vars.minS + vars.prev[m].vy;
      }
      // Find the max s for this drive wheel. Search on the interval between 0 and min_s, because we
      // already know we can't go faster than that.
      double s = findDriveMaxS(vars.prev[m].vx, vars.prev[m].vy, vxMinS, vyMinS, maxVelStep);
      vars.minS = Math.min(vars.minS, s);
    }
  }
}
