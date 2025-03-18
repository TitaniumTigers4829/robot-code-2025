package frc.robot.subsystems.algaePivot;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaePivotInterface {
  @AutoLog
  public static class AlgaePivotInputs {
    public boolean isConnected = true;
    public double algaeAngle = 0.0;
    public double algaeVoltage = 0.0;
    public double algaeVelocity = 0.0;
    public double algaeTemp = 0.0;
    public double algaeSupplyCurrentAmps = 0.0;
    public double algaeTorqueCurrentAmps = 0.0;
  }

  /**
   * Updates the input values for the Algae Pivot subsystem.
   * 
   * @param inputs The input values to update.
   */
  default void updateInputs(AlgaePivotInputs inputs) {}

  /**
   * Sets the speed of the algae pivot motor.
   * 
   * @param speed The desired speed, typically in the range [-1.0, 1.0].
   */
  default void setAlgaeSpeed(double speed) {}

  /**
   * Sets the target angle for the algae pivot.
   * 
   * @param angle The desired angle in degrees.
   */
  default void setAlgaeAngle(double angle) {}

  /**
   * Sets the voltage to be applied to the algae pivot motor.
   * 
   * @param voltage The desired voltage in volts.
   */
  default void setAlgaeVoltage(double voltage) {}

  /**
   * Checks if the pivot is within an acceptable error range.
   * 
   * @return True if the pivot is within the acceptable error range, false otherwise.
   */
  default boolean isPivotWithinAcceptableError() {
    return true;
  }

  /**
   * Gets the current angle of the algae pivot.
   * 
   * @return The current angle in degrees.
   */
  default double getAlgaeAngle() {
    return 0.0;
  }

  /**
   * Gets the target angle of the algae pivot.
   * 
   * @return The target angle in degrees.
   */
  default double getAlgaePivotTarget() {
    return 0.0;
  }
}
