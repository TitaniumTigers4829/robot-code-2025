package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroInterface {

  @AutoLog
  public static class GyroInputs {
    public boolean isConnected = false;

    public double yawDegrees = 0.0;
    public Rotation2d yawDegreesRotation2d = new Rotation2d();
    public double yawVelocityDegreesPerSecond = 0.0;
  }

  /**
   * Updates the inputs created in GyroInputs
   *
   * @param inputs inputs to update
   */
  default void updateInputs(GyroInputs inputs) {}

  /** Resets the gyro yaw to 0 */
  default void reset() {}
}
