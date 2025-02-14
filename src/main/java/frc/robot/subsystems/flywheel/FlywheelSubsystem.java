// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends SubsystemBase {
  /** Creates a new FlywheelSubsystem. */
  FlywheelInterface flywheelInterface;
  FlywheelInputsAutoLogged inputsAutoLogged;
  FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();
  
  public FlywheelSubsystem(FlywheelInterface flywheelInterface) {
    this.flywheelInterface = flywheelInterface;
    inputsAutoLogged = new FlywheelInputsAutoLogged();
  }

  public double getFlywheelSpeed() {
    return flywheelInterface.getFlywheelVelocity();
  }

  public double getVolts() {
    return flywheelInterface.getVolts();
  }

  public void setFlywheelSpeed(double speed) {
    flywheelInterface.setFlywheelSpeed(speed);
  }

  public void setVolts(double volts) {
    flywheelInterface.setVolts(volts);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    flywheelInterface.updateInputs(inputs);
    Logger.processInputs("Flywheel/", inputs);
  }

  public Command setFlyhweelSpeed(double speed) {
    return this.runOnce(() -> setFlyhweelSpeed(speed));
  }
}
