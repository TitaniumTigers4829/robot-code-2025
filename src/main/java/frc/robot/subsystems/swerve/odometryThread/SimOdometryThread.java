package frc.robot.subsystems.swerve.odometryThread;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;  // AdvantageKit Logger
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Supplier;

/**
 * A simulation odometry thread that updates sensor inputs and logs them via AdvantageKit.
 */
public class SimOdometryThread implements OdometryThreadInterface {
  private final int hz;
  private final AtomicBoolean isRunning = new AtomicBoolean(false);
  private final AtomicLong updateTimeMicros = new AtomicLong();

  private final Notifier notifier;

  @SuppressWarnings("unchecked")
  private final Supplier<SwerveModulePosition>[] positionSuppliers = new Supplier[MODULE_COUNT];

  private Supplier<Rotation2d> rotationSupplier = Rotation2d::new;
  private Supplier<double[]> accelerationSupplier = () -> new double[] {0.0, 0.0};

  // Persistent inputs instance that will be autologged.
  private final OdometryInputs inputs = new OdometryInputs();

  public SimOdometryThread(int hz) {
    this.hz = hz;
    notifier = new Notifier(this::run);
    notifier.setName("SimOdometryThread");
  }

  public void addModulePositionSupplier(int moduleId, Supplier<SwerveModulePosition> sup) {
    positionSuppliers[moduleId] = sup;
  }

  public void addRotationSupplier(Supplier<Rotation2d> sup) {
    rotationSupplier = sup;
  }

  public void addAccelerationSupplier(Supplier<double[]> sup) {
    accelerationSupplier = sup;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[MODULE_COUNT];
    for (int i = 0; i < MODULE_COUNT; i++) {
      positions[i] = (positionSuppliers[i] != null)
          ? positionSuppliers[i].get()
          : new SwerveModulePosition(0.0, new Rotation2d());
    }
    return positions;
  }

  /**
   * Updates the odometry inputs with current sensor values and passes them to AdvantageKit.
   */
  @Override
  public void updateInputs(OdometryInputs inputs) {
    long startTime = RobotController.getFPGATime();

    // Acquire acceleration data and compute magnitude.
    double[] accel = accelerationSupplier.get();
    inputs.accelMagnitude = Math.hypot(accel[0], accel[1]);

    // Get the gyro reading.
    Rotation2d rotation = rotationSupplier.get();
    inputs.gyroAngle = rotation.getDegrees();

    // Retrieve swerve module positions and update their corresponding angles.
    SwerveModulePosition[] positions = getModulePositions();
    for (int i = 0; i < MODULE_COUNT; i++) {
      inputs.drivePositions[i] = positions[i].distanceMeters;
      inputs.turnAngles[i] = positions[i].angle.getDegrees();
    }

    // Compute cycle time in milliseconds.
    long cycleDurationMicros = RobotController.getFPGATime() - startTime;
    inputs.cycleTimeMillis = cycleDurationMicros / 1_000.0;
  }

  // Called periodically by the Notifier.
  private void run() {
    updateInputs(inputs);
    updateTimeMicros.set(RobotController.getFPGATime());
  }

  /**
   * Starts the odometry thread.
   */
  public void start() {
    isRunning.set(true);
    notifier.startPeriodic(1.0 / hz);
    Logger.recordOutput("Odometry/Status", "SimOdometryThread started");
  }

  /**
   * @return The most recent update cycle time in milliseconds.
   */
  public double getUpdateTimeMili() {
    return updateTimeMicros.get() / 1_000.0;
  }

  /**
   * @return True if the odometry thread is running.
   */
  public boolean isRunning() {
    return isRunning.get();
  }
}
