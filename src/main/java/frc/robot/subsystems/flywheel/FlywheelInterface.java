// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface FlywheelInterface {
  @AutoLog
  public static class FlywheelInputs {
    public double flywheelVelocity = 0.0;
    public double flywheelMotorSpeed = 0.0;
    public double flywheelAppliedVolts = 0.0;
  }

  // are these docs strings?? ishan said to get the velocity but idk how :(

  public default void updateInputs(FlywheelInputs inputs) {}

  public default double getFlywheelVelocity() {
    return 0.0;
  }

  public default void setFlywheelSpeed(double speed) {}

  public default void setVolts(double volts) {}

  public default double getVolts() {
    return 0.0;
  }
}
