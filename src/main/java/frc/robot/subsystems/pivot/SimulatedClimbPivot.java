// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.pivot.ClimbPivotInterface.ClimbPivotInputs;

/** Add your docs here. */
public class SimulatedClimbPivot {
    private ClimbPivotSim simulatedClimbPivot = new ClimbPivotSim(null, DCMotor.getFalcon500(2), 0);
  private PIDController simPID;
  private double currentVolts;

  public SimulatedClimbPivot() {
    simPID =
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
    return simulatedClimbPivot.getClimbPivotPosition();
  }
}
