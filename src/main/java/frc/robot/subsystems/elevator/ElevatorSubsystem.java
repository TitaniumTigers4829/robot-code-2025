// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  private ElevatorInterface elevatorInterface;
  private ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ElevatorInterface elevatorInterface) {
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
  }

  public void setVolts(double volts) {
    elevatorInterface.setVolts(volts);
  }

  public void setPercentOutput(double output){
    elevatorInterface.setPercentOutput(output);
  }

  @Override
  public void periodic() {
    elevatorInterface.updateInputs(inputs);
    Logger.processInputs("Elevator/", inputs);
  }

  public Command ManualElevator(DoubleSupplier joystickY){
    return new StartEndCommand(
      //does this while command is active
      () -> this.setPercentOutput(joystickY.getAsDouble()), 
      //does this when command ends
      () -> this.setPercentOutput(0),
      //requirements for command
       this);
  }

  public Command SetElevationPosition(double position){
    return new StartEndCommand(
      //does this while command is active
      () -> this.setElevatorPosition(position), 
      //does this when command ends
      () -> this.setElevatorPosition(0),
      //requirements for command
       this);
  }
}
