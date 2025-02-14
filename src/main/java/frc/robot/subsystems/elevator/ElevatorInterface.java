// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Creates a new ElevatorInterface. */
public interface ElevatorInterface {
  @AutoLog
  public static class ElevatorInputs {
    public double leaderMotorPosition = 0.0;
    public double followerMotorPosition = 0.0;
    public double leaderMotorVoltage = 0.0;
    public double followerMotorVoltage = 0.0;
    public double leaderDutyCycle = 0.0;
    public double followerDutyCycle = 0.0;
    public boolean limitSwitchActivated = false;
    public double desiredPosition = 0.0;
  }

  /**
   * Updates the inputs for the elevator.
   *
   * @param inputs The inputs for the elevator.
   */
  public default void updateInputs(ElevatorInputs inputs) {}

  /**
   * Sets the position of the elevator.
   *
   * @param position The requested elevator position in meters.
   */
  public default void setElevatorPosition(double position) {}

  /**
   * Returns the current position of the elevator.
   *
   * @return Position of the elevator in meters.
   */
  public default double getElevatorPosition() {
    return 0.0;
  }

  /**
   * Sets the voltage of the elevator.
   *
   * @param volts Requested voltage for the elevator.
   */
  public default void setVolts(double volts) {}

  /**
   * Returns the voltage of the elevator.
   *
   * @return Current voltage of the elevator.
   */
  public default double getVolts() {
    return 0.0;
  }

  /**
   * Returns whether the elevator is past it's limit
   *
   * @return If elevator reaches the top
   */
  public default boolean isLimitSwitchActivated() {
    return false;
  }
}
