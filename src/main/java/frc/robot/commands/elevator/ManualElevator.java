// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class ManualElevator extends Command {
  ElevatorSubsystem elevatorSubsystem;
  DoubleSupplier joystickY;

  /**
   * Creates a new ManualElevator.
   *
   * @param elevatorSubsystem Elevator subsystem
   * @param position Position in meters
   * @param joystickY = Double Supplier for the joystick
   */
  public ManualElevator(ElevatorSubsystem elevatorSubsystem, DoubleSupplier joystickY) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.joystickY = joystickY;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setElevatorPosition(joystickY.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setVolts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
