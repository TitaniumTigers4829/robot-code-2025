// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import java.util.function.BooleanSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPosition extends Command {
  ElevatorSubsystem elevatorSubsystem;
  BooleanSupplier test;
  double position;

  /** Creates a new SetElevatorPosition. */
  public SetElevatorPosition(ElevatorSubsystem elevatorSubsystem, BooleanSupplier test) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.test = test;
    this.position = 1;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (test.getAsBoolean()) {
      position = 2;
    } else {
      position = 0;
    }

    elevatorSubsystem.setElevatorPosition(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setElevatorPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
