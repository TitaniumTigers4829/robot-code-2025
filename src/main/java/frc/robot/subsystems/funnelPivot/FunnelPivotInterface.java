package frc.robot.subsystems.funnelPivot;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelPivotInterface {
  @AutoLog
  public class FunnelPivotInputs {
    public boolean isConnected = true;
    public double funnelAngle = 0.0;
    public double funnelVoltage = 0.0;
    public double funnelVelocity = 0.0;
    public double funnelTemp = 0.0;
    public double funnelSupplyCurrentAmps = 0.0;
    public double funnelTorqueCurrentAmps = 0.0;
  }

  /**
   * Updates the inputs of the funnel pivot.
   * 
   * @param inputs The inputs to update.
   */
  default void updateInputs(FunnelPivotInputs inputs) {}

  /**
   * Sets the speed of the funnel pivot.
   * 
   * @param speed The speed to set.
   */
  default void setFunnelSpeed(double speed) {}

  /**
   * Sets the angle of the funnel pivot.
   * 
   * @param angle The angle to set (in rotations).
   */
  default void setFunnelAngle(double angle) {}

  /**
   * Sets the voltage of the funnel pivot.
   * 
   * @param voltage The voltage to set.
   */
  default void setFunnelVoltage(double voltage) {}

  /**
   * Checks if the pivot is within an acceptable error range.
   * 
   * @return true if within acceptable error, false otherwise.
   */
  default boolean isPivotWithinAcceptapleError() {
    return true;
  }

  /**
   * Gets the current angle of the funnel pivot.
   * 
   * @return The current angle in rotations.
   */
  default double getFunnelAngle() {
    return 0.0;
  }

  /**
   * Gets the target angle of the funnel pivot.
   * 
   * @return The target angle in rotations.
   */
  default double getFunnelPivotTarget() {
    return 0.0;
  }

  /**
   * Sets the desired PID constants for the funnel pivot.
   * 
   * @param kP Desired proportional gain.
   * @param kI Desired integral gain.
   * @param kD Desired derivative gain.
   */
  default void setPID(double kP, double kI, double kD) {}
}
