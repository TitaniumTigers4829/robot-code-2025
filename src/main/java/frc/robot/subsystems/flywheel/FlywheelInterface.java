// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface FlywheelInterface {
  @AutoLog
  public static class FlywheelInputs {
    /**
     * Again, all docstrings written by chatgpt. take none of this seriously. The current velocity
     * of the flywheel (in RPM, rad/s, or other relevant units).
     */
    public double flywheelVelocity = 0.0;

    /** The current speed of the flywheel motor (in RPM, percentage, or other relevant units). */
    public double flywheelMotorSpeed = 0.0;

    /** The voltage currently applied to the flywheel motor (in volts). */
    public double flywheelAppliedVolts = 0.0;
  }

  /**
   * Updates the provided `FlywheelInputs` with the current state of the flywheel system. This
   * method can be implemented to update the inputs with the current flywheel velocity, motor speed,
   * and applied voltage.
   *
   * @param inputs The `FlywheelInputs` object to update. This will be populated with the current
   *     state of the flywheel.
   */
  public default void updateInputs(FlywheelInputs inputs) {}

  /**
   * Retrieves the current velocity of the flywheel. This method should return the velocity of the
   * flywheel in an appropriate unit, such as RPM or rad/s.
   *
   * @return The current flywheel velocity.
   */
  public default double getFlywheelVelocity() {
    return 0.0;
  }

  /**
   * Sets the desired speed for the flywheel motor. This method should be implemented to control the
   * flywheel motor to the target speed.
   *
   * @param speed The desired speed for the flywheel motor (e.g., in RPM, percentage, or another
   *     relevant unit).
   */
  public default void setFlywheelSpeed(double speed) {}

  /**
   * Sets the applied voltage for the flywheel motor. This method is used to directly control the
   * motor's voltage.
   *
   * @param volts The voltage to apply to the flywheel motor (in volts).
   */
  public default void setVolts(double volts) {}

  /**
   * Retrieves the current applied voltage to the flywheel system. This method should return the
   * voltage that is currently being applied to the flywheel motor.
   *
   * @return The current applied voltage to the flywheel motor.
   */
  public default double getVolts() {
    return 0.0;
  }
}
