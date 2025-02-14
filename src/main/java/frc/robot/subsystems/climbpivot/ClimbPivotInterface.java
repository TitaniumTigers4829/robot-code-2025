// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climbpivot;

/** Add your docs here. */
public interface ClimbPivotInterface {
  public static class ClimbPivotInputs {
    /**
     * THESE DOCSTRINGS WERE ALL WRITTEN BY CHATGPT, i think they're mostly accurate but idk so TAKE
     * NONE OF THESE SERIOUSLY The current position of the climb pivot (in radians or encoder
     * units).
     */
    public double position = 0.0;

    /** The voltage currently applied to the climb pivot motor (in volts). */
    public double climbPivotAppliedVolts = 0.0;
  }

  /**
   * Updates the provided `ClimbPivotInputs` with the current state of the climb pivot system. This
   * method can be implemented to update the inputs with the current position and applied voltage.
   *
   * @param inputs The `ClimbPivotInputs` object to update. It will be populated with the current
   *     position and voltage.
   */
  public default void updateInputs(ClimbPivotInputs inputs) {}

  /**
   * Retrieves the current position of the climb pivot system. This method should return the current
   * position in radians or any other relevant unit.
   *
   * @return The current position of the climb pivot.
   */
  public default double getClimbPivotPosition() {
    return 0.0;
  }

  /**
   * Sets the desired position for the climb pivot system. This method should be implemented to
   * control the climb pivot motor to reach the target position.
   *
   * @param position The target position to set for the climb pivot.
   */
  public default void setClimbPivotPosition(double position) {}

  /**
   * Sets the applied voltage for the climb pivot motor. This method is used to directly control the
   * motor's voltage.
   *
   * @param volts The voltage to apply to the climb pivot motor.
   */
  public default void setVolts(double volts) {}

  /**
   * Retrieves the current applied voltage to the climb pivot system. This method should return the
   * voltage that is currently being applied to the climb pivot motor.
   *
   * @return The current applied voltage to the climb pivot motor.
   */
  public default double getVolts() {
    return 0.0;
  }
}
