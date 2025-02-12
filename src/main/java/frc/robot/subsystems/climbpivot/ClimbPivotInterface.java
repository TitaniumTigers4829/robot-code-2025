// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climbpivot;

/** Add your docs here. */
  public interface ClimbPivotInterface {
    public static class ClimbPivotInputs {
      public double position = 0.0;
    }

    public default void updateInputs(ClimbPivotInputs inputs) {}

    public default double getClimbPivotPosition() {
      return 0.0;
    }

    public default void setClimbPivotPosition(double position) {}

    public default void setVolts(double volts) {}

    public default double getVolts() {
      return 0.0;
    }
  }
