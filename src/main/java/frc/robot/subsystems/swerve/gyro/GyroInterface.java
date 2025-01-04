package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroInterface {
  @AutoLog
  public static class GyroInputs {
    public boolean isConnected = false;

    public double yawDegrees = 0.0;
    public Rotation2d yawDegreesRotation2d = new Rotation2d();
    public double yawVelocity = 0.0;

    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public double[] odometryYawTimestamps = new double[] {};
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
