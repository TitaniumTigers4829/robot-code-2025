// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  /** Creates a new elevator. */
  private ElevatorInterface elevatorInterface;

  private ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

  public Elevator(ElevatorInterface elevatorInterface) {
    this.elevatorInterface = elevatorInterface;
  }

  public double getElevatorPosition() {
    return elevatorInterface.getElevatorPosition();
  }

  public double getVolts() {
    return elevatorInterface.getVolts();
  }

  public void setElevatorPosition(double position) {
    elevatorInterface.setElevatorPosition(position);
    Logger.recordOutput("Elevator/Position", getElevatorPosition());
  }

  public void setVolts(double volts) {
    elevatorInterface.setVolts(volts);
    Logger.recordOutput("Elevator/Volts", getVolts());
  }

  @Override
  public void periodic() {
    elevatorInterface.updateInputs(inputs);
    Logger.processInputs("Elevator/", inputs);
  }
}
