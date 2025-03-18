// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class PhysicalFlywheel implements FlywheelInterface {
  private TalonFX flywheelMotor = new TalonFX(FlywheelConstants.FLYWHEEL_MOTOR_ID);
  public TalonFXConfiguration config;

  private VoltageOut voltageOut = new VoltageOut(0.0);
  private VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  public PhysicalFlywheel() {
    config.Slot0.kP = FlywheelConstants.FLYWHEEL_P;
    config.Slot0.kI = FlywheelConstants.FLYWHEEL_I;
    config.Slot0.kD = FlywheelConstants.FLYWHEEL_D;

    config.Slot0.kS = FlywheelConstants.FF_FLYWHEEL_S;
    config.Slot0.kV = FlywheelConstants.FF_FLYWHEEL_V;
    config.Slot0.kA = FlywheelConstants.FF_FLYWHEEL_A;

    flywheelMotor.getConfigurator().apply(config);
  }

  public void updateInputs(FlywheelInputs inputs) {
    inputs.flywheelMotorSpeed = getFlywheelVelocity();
  }

  public void setFlywheelVelocity(double velocity) {
    flywheelMotor.setControl(velocityRequest.withVelocity(velocity));
  }

  public double getFlywheelVelocity() {
    return flywheelMotor.getVelocity().getValueAsDouble();
  }

  public void setVolts(double volts) {
    flywheelMotor.setControl(voltageOut.withOutput(volts));
  }

  public double getVolts() {
    return flywheelMotor.getMotorVoltage().getValueAsDouble();
  }
}
