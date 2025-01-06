// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorInterface {
  /** Creates a new ElevatorInterface. */
  @AutoLog
  public static class ElevatorInputs { // For values
    public double rightMotorPosition = 0.0;
    public double leftMotorPosition = 0.0;
  }

  public default void updateInputs(ElevatorInputs inputs) {}

  public default double getElevatorPosition() {
    return 0.0;
  }

  public default void setElevatorPosition(double position) {}

  public default void setVolts(double volts) {}

  public default double getVolts() {
    return 0.0;
  }
}
