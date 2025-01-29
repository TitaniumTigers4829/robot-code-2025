// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.pivot.ClimbPivotInterface.ClimbPivot.ClimbPivotInputs;

/** Add your docs here. */
public class PhysicalClimbPivot {
    private PhysicalClimbPivot physicalClimbPivot = new PhysicalClimbPivot();
  private PIDController PID;
  private double currentVolts;

  public PhysicalClimbPivot() {
    PID =
        new PIDController(
            PivotConstants.CLIMB_PIVOT_P,
            PivotConstants.CLIMB_PIVOT_I,
            PivotConstants.CLIMB_PIVOT_D);
  }

  public void updateInputs(ClimbPivotInputs inputs) {
    inputs.position = getClimbPivotPosition();
  }

  public void setClimbPivotPosition(double position) {
    setClimbPivotPosition(position);
  }

  public double getClimbPivotPosition() {
    return physicalClimbPivot.getClimbPivotPosition();
  }
}
