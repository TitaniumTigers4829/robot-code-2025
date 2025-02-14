// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climbpivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class PhysicalClimbPivot implements ClimbPivotInterface {
  public TalonFXConfiguration config;
  private TalonFX climbMotor = new TalonFX(PivotConstants.CLIMB_PIVOT_MOTOR_ID);
  MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);

  public PhysicalClimbPivot() {
    config.Slot0.kP = PivotConstants.CLIMB_PIVOT_P;
    config.Slot0.kI = PivotConstants.CLIMB_PIVOT_I;
    config.Slot0.kD = PivotConstants.CLIMB_PIVOT_D;

    config.Slot0.kS = PivotConstants.FF_CLIMB_PIVOT_S;
    config.Slot0.kV = PivotConstants.FF_CLIMB_PIVOT_G;
    config.Slot0.kA = PivotConstants.FF_CLIMB_PIVOT_V;

    climbMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ClimbPivotInputs inputs) {
    inputs.position = getClimbPivotPosition();
    inputs.climbPivotAppliedVolts = climbMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setClimbPivotPosition(double position) {
    climbMotor.setControl(motionMagicVoltage.withPosition(position));
  }

  @Override
  public double getClimbPivotPosition() {
    return climbMotor.getPosition().getValueAsDouble();
  }
}
