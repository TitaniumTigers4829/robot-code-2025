// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  private ElevatorInterface elevatorInterface;
  private ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ElevatorInterface elevatorInterface) {
    this.elevatorInterface = elevatorInterface;
  }

  /**
   * Returns the current position of the elevator.
   *
   * @return Position of the elevator in meters.
   */
  public double getElevatorPosition() {
    return elevatorInterface.getElevatorPosition();
  }

  /**
   * Returns the voltage of the elevator.
   *
   * @return Current voltage of the elevator.
   */
  public double getVolts() {
    return elevatorInterface.getVolts();
  }

  /**
   * Sets the position of the elevator.
   *
   * @param position The requested elevator position in meters.
   */
  public void setElevatorPosition(double position) {
    elevatorInterface.setElevatorPosition(position);
  }

  /**
   * Sets the voltage of the elevator.
   *
   * @param volts Requested voltage for the elevator.
   */
  public void setVolts(double volts) {
    elevatorInterface.setVolts(volts);
  }

  @Override
  public void periodic() {
    elevatorInterface.updateInputs(inputs);
    Logger.processInputs("Elevator/", inputs);
  }
}
