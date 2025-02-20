// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorInterface {
  /** Creates a new ElevatorInterface. */
  @AutoLog
  public static class ElevatorInputs { // For values
    public double leaderMotorPosition = 0.0;
    public double followerMotorPosition = 0.0;
    public double leaderMotorVoltage = 0.0;
    public double followerMotorVoltage = 0.0;
    public double leaderDutyCycle = 0.0;
    public double followerDutyCycle = 0.0;
    public double desiredPosition = 0.0;
    public double leaderStatorCurrent = 0.0;
    public double followerStatorCurrent = 0.0;
    public double leaderVelocity = 0.0;
    public double elevatorError = 0.0;
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

  public default void openLoop(double output) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setFF(double kS, double kV, double kA, double kG) {}
}
