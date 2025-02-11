// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.pivot.ClimbPivotInterface.ClimbPivot.ClimbPivotInputs;

/** Add your docs here. */
public class PhysicalClimbPivot {
  private PhysicalClimbPivot physicalClimbPivot = new PhysicalClimbPivot();
  private VoltageOut voltageOut = new VoltageOut(0.0);
  public TalonFXConfiguration config;
  public PIDController climbPivotPIDController;
  public SimpleMotorFeedforward climbMotorFeedforward;
  private TalonFX climbMotor = new TalonFX(PivotConstants.CLIMB_PIVOT_MOTOR_ID);

  public PhysicalClimbPivot() {
    PIDController climbPivotPIDController = new PIDController(PivotConstants.CLIMB_PIVOT_P, PivotConstants.CLIMB_PIVOT_I, PivotConstants.CLIMB_PIVOT_D);
    SimpleMotorFeedforward climbMotorFeedforward = new SimpleMotorFeedforward(PivotConstants.FF_CLIMB_PIVOT_S, PivotConstants.FF_CLIMB_PIVOT_V, PivotConstants.FF_CLIMB_PIVOT_A);
  }

  public void updateInputs(ClimbPivotInputs inputs) {
    inputs.position = getClimbPivotPosition();
  }

  public void setClimbPivotPosition(double position) {
    setClimbPivotPosition(climbPivotPIDController.calculate(getClimbPivotPosition(), position) + climbMotorFeedforward.calculate(position));
  }

  public double getClimbPivotPosition() {
    return physicalClimbPivot.getClimbPivotPosition();
  }
}
