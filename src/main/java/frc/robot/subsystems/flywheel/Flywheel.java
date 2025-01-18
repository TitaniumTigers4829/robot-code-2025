// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlywheelSubsystem extends SubsystemBase {
  /** Creates a new FlywheelSubsystem. */
   private FlywheelInterface flywheelInterface;

  private FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();

  public Elevator(ElevatorInterface elevatorInterface) {
    this.flywheelInterface = flywheelInterface;
  }

  public double getElevatorPosition() {
    return flywheelInterface.getElevatorPosition();
  }

  public double getVolts() {
    return flywheelInterface.getVolts();
  }

  public void setElevatorPosition(double position) {
    flywheelInterface.setElevatorPosition(position);
  }

  public void setVolts(double volts) {
    flywheelInterface.setVolts(volts);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    flywheelInterface.updateInputs(inputs);
    Logger.processInputs("Elevator/", inputs);
  }
}