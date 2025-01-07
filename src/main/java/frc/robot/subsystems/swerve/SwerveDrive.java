package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extras.setpointGen.SwerveSetpoint;
import frc.robot.extras.setpointGen.SwerveSetpointGenerator;
import frc.robot.extras.simulation.mechanismSim.swerve.SwerveModuleSimulation.WHEEL_GRIP;
import frc.robot.extras.util.DeviceCANBus;
import frc.robot.extras.util.TimeUtil;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.gyro.GyroInterface;
import frc.robot.subsystems.swerve.module.ModuleInterface;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread;
import frc.robot.subsystems.swerve.odometryThread.OdometryThreadInputsAutoLogged;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import java.util.Timer;
import java.util.TimerTask;

public class SwerveDrive extends SubsystemBase {
  private final GyroInterface gyroIO;
  private final GyroInputsAutoLogged gyroInputs;
  private final OdometryThreadInputsAutoLogged odometryThreadInputs;
  private final SwerveModule[] swerveModules;

  private Rotation2d rawGyroRotation;
  private final SwerveModulePosition[] lastModulePositions;
  private final SwerveDrivePoseEstimator poseEstimator;

  private final SwerveSetpointGenerator setpointGenerator =
      new SwerveSetpointGenerator(
          DriveConstants.MODULE_TRANSLATIONS,
          DCMotor.getKrakenX60(1).withReduction(ModuleConstants.DRIVE_GEAR_RATIO),
          DCMotor.getFalcon500(1).withReduction(1),
          60,
          58,
          7,
          ModuleConstants.WHEEL_DIAMETER_METERS,
          WHEEL_GRIP.TIRE_WHEEL.cof,
          0.0);
  private SwerveSetpoint setpoint = SwerveSetpoint.zeroed();

  private final OdometryThread odometryThread;

  private Optional<DriverStation.Alliance> alliance;

  private final Alert gyroDisconnectedAlert =
      new Alert("Gyro Hardware Fault", Alert.AlertType.kError);

  public SwerveDrive(
      GyroInterface gyroIO,
      ModuleInterface frontLeftModuleIO,
      ModuleInterface frontRightModuleIO,
      ModuleInterface backLeftModuleIO,
      ModuleInterface backRightModuleIO) {
    this.gyroIO = gyroIO;
    this.gyroInputs = new GyroInputsAutoLogged();
    this.rawGyroRotation = new Rotation2d();

    swerveModules =
        new SwerveModule[] {
          new SwerveModule(frontLeftModuleIO, "FrontLeft"),
          new SwerveModule(frontRightModuleIO, "FrontRight"),
          new SwerveModule(backLeftModuleIO, "BackLeft"),
          new SwerveModule(backRightModuleIO, "BackRight")
        };

    lastModulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            rawGyroRotation,
            lastModulePositions,
            new Pose2d(),
            VecBuilder.fill(
                DriveConstants.X_POS_TRUST, DriveConstants.Y_POS_TRUST, DriveConstants.ANGLE_TRUST),
            VecBuilder.fill(
                VisionConstants.VISION_X_POS_TRUST,
                VisionConstants.VISION_Y_POS_TRUST,
                VisionConstants.VISION_ANGLE_TRUST));

    this.odometryThread = OdometryThread.createInstance(DeviceCANBus.RIO);
    this.odometryThreadInputs = new OdometryThreadInputsAutoLogged();
    this.odometryThread.start();

    gyroDisconnectedAlert.set(false);
  }

  /*
   * Updates the pose estimator with the pose calculated from the april tags. How much it
   * contributes to the pose estimation is set by setPoseEstimatorVisionConfidence.
   *
   * @param visionMeasurement The pose calculated from the april tags
   * @param currentTimeStampSeconds The time stamp in seconds of when the pose from the april tags
   *     was calculated.
   */
  public void addPoseEstimatorVisionMeasurement(
      Pose2d visionMeasurement, double currentTimeStampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, currentTimeStampSeconds);
  }

  /**
   * Sets the standard deviations of model states, or how much the april tags contribute to the pose
   * estimation of the robot. Lower numbers equal higher confidence and vice versa.
   *
   * @param xStandardDeviation the x standard deviation in meters
   * @param yStandardDeviation the y standard deviation in meters
   * @param thetaStandardDeviation the theta standard deviation in radians
   */
  public void setPoseEstimatorVisionConfidence(
      double xStandardDeviation, double yStandardDeviation, double thetaStandardDeviation) {
    poseEstimator.setVisionMeasurementStdDevs(
        VecBuilder.fill(xStandardDeviation, yStandardDeviation, thetaStandardDeviation));
  }

  @Override
  public void periodic() {
    final double t0 = TimeUtil.getRealTimeSeconds();
    fetchOdometryInputs();
    Logger.recordOutput(
        "SystemPerformance/OdometryFetchingTimeMS", (TimeUtil.getRealTimeSeconds() - t0) * 1000);
    modulesPeriodic();
  }

  /**
   * Runs characterization on voltage
   *
   * @param volts voltage to set
   */
  public void runCharacterization(double volts) {
    for (SwerveModule module : swerveModules) {
      module.setVoltage(Volts.of(-volts));
    }
  }

  /** Returns the average drive velocity in rotations/sec. */
  public double getCharacterizationVelocity() {
    double velocity = 0.0;
    for (SwerveModule module : swerveModules) {
      velocity += module.getCharacterizationVelocity();
    }
    return velocity;
  }

  /** Processes odometry inputs */
  private void fetchOdometryInputs() {
    odometryThread.lockOdometry();
    odometryThread.updateInputs(odometryThreadInputs);
    Logger.processInputs("Drive/OdometryThread", odometryThreadInputs);

    for (SwerveModule module : swerveModules) module.updateOdometryInputs();

    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    gyroDisconnectedAlert.set(!gyroInputs.isConnected);

    odometryThread.unlockOdometry();
  }

  /** Runs the SwerveModules periodic methods */
  private void modulesPeriodic() {
    for (SwerveModule module : swerveModules) module.periodic();
  }

  /**
   * Drives the robot using the joysticks.
   *
   * @param xSpeed Speed of the robot in the x direction, positive being forwards.
   * @param ySpeed Speed of the robot in the y direction, positive being left.
   * @param rotationSpeed Angular rate of the robot in radians per second.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative) {
    ChassisSpeeds desiredSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotationSpeed, getOdometryAllianceRelativeRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);

    setpoint = setpointGenerator.generateSetpoint(setpoint, desiredSpeeds, 0.02);

    setModuleStates(setpoint.moduleStates());
    Logger.recordOutput("SwerveStates/DesiredStates", setpoint.moduleStates());
  }

  /**
   * Returns the heading of the robot in degrees from 0 to 360.
   *
   * @return Value is Counter-clockwise positive.
   */
  public double getHeading() {
    return gyroInputs.yawDegrees;
  }

  /**
   * Gets the rate of rotation of the NavX.
   *
   * @return The current rate in degrees per second.
   */
  public double getGyroRate() {
    return gyroInputs.yawVelocity;
  }

  /** Returns a Rotation2d for the heading of the robot. */
  public Rotation2d getGyroRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  /** Returns a Rotation2d for the heading of the robot. */
  public Rotation2d getGyroFieldRelativeRotation2d() {
    return Rotation2d.fromDegrees(getHeading() + getAllianceAngleOffset());
  }

  /** Returns 0 degrees if the robot is on the blue alliance, 180 if on the red alliance. */
  public double getAllianceAngleOffset() {
    alliance = DriverStation.getAlliance();
    double offset =
        alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red ? 180.0 : 0.0;
    return offset;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyroIO.reset();
  }

  /**
   * Returns the estimated field-relative pose of the robot. Positive x being forward, positive y
   * being left.
   */
  @AutoLogOutput(key = "Odometry/Odometry")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns a Rotation2d for the heading of the robot */
  public Rotation2d getOdometryRotation2d() {
    return getPose().getRotation();
  }

  /**
   * Returns a Rotation2d for the heading of the robot relative to the field from the driver's
   * perspective. This method is needed so that the drive command and poseEstimator don't fight each
   * other. It uses odometry rotation.
   */
  public Rotation2d getOdometryAllianceRelativeRotation2d() {
    return getPose().getRotation().plus(Rotation2d.fromDegrees(getAllianceAngleOffset()));
  }

  /**
   * Sets the modules to the specified states.
   *
   * @param desiredStates The desired states for the swerve modules. The order is: frontLeft,
   *     frontRight, backLeft, backRight (should be the same as the kinematics).
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    for (int i = 0; i < 4; i++) {
      swerveModules[i].runSetPoint((desiredStates[i]));
    }
  }

  /**
   * Updates the pose estimator with the pose calculated from the swerve modules.
   *
   * @param timestampIndex index of the timestamp to sample the pose at
   */
  public void addPoseEstimatorSwerveMeasurement() { // int timestampIndex
    final SwerveModulePosition[] modulePositions = getModulePositions(),
        moduleDeltas = getModulesDelta(modulePositions);

    if (gyroInputs.isConnected) {
      // rawGyroRotation = gyroInputs.odometryYawPositions[timestampIndex];
      rawGyroRotation = getGyroRotation2d();
    } else {
      Twist2d twist = DriveConstants.DRIVE_KINEMATICS.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    poseEstimator.updateWithTime(
        // odometryThreadInputs.measurementTimeStamps[timestampIndex],
        Logger.getTimestamp(), rawGyroRotation, modulePositions);
  }

  /**
   * @param freshModulesPosition Latest module positions
   * @return The change of the module positions between the current and last update
   */
  private SwerveModulePosition[] getModulesDelta(SwerveModulePosition[] freshModulesPosition) {
    SwerveModulePosition[] deltas = new SwerveModulePosition[swerveModules.length];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      final double deltaDistanceMeters =
          freshModulesPosition[moduleIndex].distanceMeters
              - lastModulePositions[moduleIndex].distanceMeters;
      deltas[moduleIndex] =
          new SwerveModulePosition(deltaDistanceMeters, freshModulesPosition[moduleIndex].angle);
      lastModulePositions[moduleIndex] = freshModulesPosition[moduleIndex];
    }
    return deltas;
  }

  /** Returns the module states (turn angles and drive velocities) for all the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
    for (int i = 0; i < states.length; i++) states[i] = swerveModules[i].getMeasuredState();
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < positions.length; i++) positions[i] = swerveModules[i].getPosition();
    return positions;
  }

  /**
   * Sets the pose
   *
   * @param pose pose to set
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }
}
