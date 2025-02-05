// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.pivot.ClimbPivotInterface.ClimbPivot.ClimbPivotInputs;

public class SimulatedClimbPivot {
  private SingleJointedArmSim simulatedClimbPivot =
      new SingleJointedArmSim(DCMotor.getFalcon500(2), 1, 0.1, 1.0, 0, 1.0, false, 0.0, 0.001);
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
    return simulatedClimbPivot.getAngleRads();
  }
}
