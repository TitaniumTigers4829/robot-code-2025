// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.flywheel.FlywheelInterface.FlywheelInputs;
import frc.robot.subsystems.pivot.PivotConstants;

/** Add your docs here. */
public class PhysicalFlywheel implements FlywheelInterface {
  private double currentVolts;
  public TalonFXConfiguration config;

  private VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private TalonFX flywheelMotor = new TalonFX(FlywheelConstants.FLYWHEEL_MOTOR_ID);

  public PhysicalFlywheel() {
      config.Slot0.kP = FlywheelConstants.FLYWHEEL_P;
      config.Slot0.kI = FlywheelConstants.FLYWHEEL_I;
      config.Slot0.kD = FlywheelConstants.FLYWHEEL_D;

      config.Slot0.kS = FlywheelConstants.FF_FLYWHEEL_S;
      config.Slot0.kV = FlywheelConstants.FF_FLYWHEEL_V;
      config.Slot0.kA = FlywheelConstants.FF_FLYWHEEL_A;


  }

  public void updateInputs(FlywheelInputs inputs) {
    inputs.flywheelMotorSpeed = getFlywheelSpeed();
  }

  public void setFlywheelSpeed(double speed) {
    flywheelMotor.setControl(velocityRequest.withVelocity(speed));
  }

  public double getFlywheelSpeed(double speed) {
    return flywheelMotor.getVelocity(velocityRequest.withVelocity(speed));
  }

  public void setVolts(double volts) {
    currentVolts = velocityRequest.calculate(volts);
    flywheelMotor.setVoltage(currentVolts);
  }

  public double getVolts() {
    return currentVolts;
  }
}
