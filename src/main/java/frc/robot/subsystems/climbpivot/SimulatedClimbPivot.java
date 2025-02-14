// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climbpivot;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimulatedClimbPivot implements ClimbPivotInterface {
  private SingleJointedArmSim simulatedClimbPivot =
      new SingleJointedArmSim(DCMotor.getFalcon500(2), 1, 0.1, 1.0, 0, 1.0, false, 0.0, 0.001);
  private PIDController simPID;
  private SimpleMotorFeedforward simFFPID;
  MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);
  private double currentVolts;

  public SimulatedClimbPivot() {
    simPID =
        new PIDController(
            PivotConstants.CLIMB_PIVOT_P,
            PivotConstants.CLIMB_PIVOT_I,
            PivotConstants.CLIMB_PIVOT_D);

    simFFPID =
        new SimpleMotorFeedforward(
            PivotConstants.FF_CLIMB_PIVOT_S,
            PivotConstants.FF_CLIMB_PIVOT_G,
            PivotConstants.FF_CLIMB_PIVOT_V);
  }

  @Override
  public void updateInputs(ClimbPivotInputs inputs) {
    inputs.position = getClimbPivotPosition();
    inputs.climbPivotAppliedVolts = currentVolts;
  }

  @Override
  public void setClimbPivotPosition(double targetPosition) {
    double pidOutput = simPID.calculate(getClimbPivotPosition(), targetPosition);
    double feedforwardOutput =
        simFFPID.calculate(getClimbPivotPosition(), simulatedClimbPivot.getVelocityRadPerSec());
    currentVolts = pidOutput + feedforwardOutput;
    simulatedClimbPivot.setInputVoltage(currentVolts);
  }

  @Override
  public double getClimbPivotPosition() {
    return simulatedClimbPivot.getAngleRads();
  }

  @Override
  public double getVolts() {
    return currentVolts;
  }

  @Override
  public void setVolts(double volts) {
    currentVolts = volts;
    simulatedClimbPivot.setInputVoltage(volts);
  }
}
