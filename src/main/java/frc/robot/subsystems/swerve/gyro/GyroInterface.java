package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroInterface {
  @AutoLog
  public static class GyroInputs {
    public boolean isConnected = false;

    public double yawDegrees = 0.0;
    public double yawVelocity = 0.0;
    public double accelX = 0.0;
    public double accelY = 0.0;
  }

  /**
   * Updates the inputs created in GyroInputs
   *
   * @param inputs inputs to update
   */
  default void updateInputs(GyroInputs inputs) {}

  /** Resets the gyro yaw */
  default void reset() {}
}
