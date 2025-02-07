package frc.robot.subsystems.swerve.odometryThread;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.math.geometry.Rotation2d.fromRotations;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

/**
 * A “real” odometry thread that reads hardware signals and updates a persistent OdometryInputs instance.
 * This instance is updated via updateInputs() and then passed to AdvantageKit’s Logger.
 */
public class RealOdometryThread implements OdometryThreadInterface {
  private final int hz;
  private final AtomicBoolean isRunning = new AtomicBoolean(false);
  private final AtomicLong updateTimeMicros = new AtomicLong();
  private final Thread thread;
  
  // Expecting a total of (MODULE_COUNT * 4) signals for the modules and 4 signals for gyro/acceleration.
  // For each module:
  //  - offset +0: drive position
  //  - offset +1: drive velocity
  //  - offset +2: angle position (in rotations; convert to degrees)
  //  - offset +3: turn velocity (in rotations per second; convert to degrees per second)
  // Then, extra signals:
  //  - Index MODULE_COUNT*4: gyro yaw (in degrees)
  //  - Index MODULE_COUNT*4+2 and +3: X and Y acceleration components.
  private final BaseStatusSignal[] signals = new BaseStatusSignal[(MODULE_COUNT * 4) + 4];
  
  // Filters to smooth out cycle time measurements.
  private final MedianFilter peakRemover = new MedianFilter(3);
  private final LinearFilter lowPass = LinearFilter.movingAverage(50);

  // Persistent inputs instance that AdvantageKit will autolog.
  private final OdometryInputs inputs = new OdometryInputs();

  /**
   * Constructs the real odometry thread.
   *
   * @param hz The update frequency in Hertz.
   */
  public RealOdometryThread(int hz) {
    this.hz = hz;
    thread = new Thread(this::run, "RealOdometryThread");
    // The signals array should be configured externally (e.g., via addSignal methods or during initialization).
  }

  /**
   * Updates the odometry inputs using real sensor signals.
   * For each module:
   * - Reads drive position (offset 0) and drive velocity (offset 1)
   * - Reads angle position (offset 2, converted from rotations to degrees) and turn velocity (offset 3, converted similarly)
   * Then, reads gyro yaw and computes acceleration magnitude.
   */
  @Override
  public void updateInputs(OdometryInputs inputs) {
    long startTime = RobotController.getFPGATime();

    // Wait for all signals to refresh (with timeout based on update frequency).
    BaseStatusSignal.waitForAll(2.0 / hz, signals);
    
    // Compute cycle time.
    long elapsedTime = RobotController.getFPGATime() - startTime;
    double filteredElapsed = lowPass.calculate(peakRemover.calculate(elapsedTime));
    inputs.cycleTimeMillis = filteredElapsed / 1_000.0;
    
    // Update each module's sensor values.
    for (int i = 0; i < MODULE_COUNT; i++) {
      int offset = 4 * i;
      // Drive position.
      double drivePos = signals[offset].getValueAsDouble();
      // Drive velocity.
      double driveVel = signals[offset + 1].getValueAsDouble();
      // Angle position (in rotations) converted to degrees.
      double angleRotations = signals[offset + 2].getValueAsDouble();
      double turnVelRotPerSec = signals[offset + 3].getValueAsDouble();
      
      inputs.drivePositions[i] = drivePos;
      inputs.driveVelocities[i] = driveVel;
      inputs.turnAngles[i] = fromRotations(angleRotations).getDegrees();
      inputs.turnVelocities[i] = fromRotations(turnVelRotPerSec).getDegrees();
    }
    
    // Gyro: assume the first extra signal is the gyro yaw (in degrees).
    int gyroIndex = MODULE_COUNT * 4;
    inputs.gyroAngle = signals[gyroIndex].getValueAsDouble();
    
    // Acceleration: use the next two signals for X and Y acceleration components.
    double xAccel = signals[gyroIndex + 2].getValueAsDouble();
    double yAccel = signals[gyroIndex + 3].getValueAsDouble();
    inputs.accelMagnitude = Math.hypot(xAccel, yAccel);
  }

  /**
   * The main loop for the real odometry thread.
   * Continuously updates the sensor inputs while the thread is running.
   */
  private void run() {
    isRunning.set(true);
    try {
      while (isRunning.get()) {
        updateInputs(inputs);
        updateTimeMicros.set(RobotController.getFPGATime());
      }
    } finally {
      isRunning.set(false);
    }
  }

  /**
   * Starts the real odometry thread.
   */
  public void start() {
    thread.start();
    Logger.recordOutput("Odometry/Status", "RealOdometryThread started");
  }

  /**
   * @return The most recent cycle update time in milliseconds.
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
