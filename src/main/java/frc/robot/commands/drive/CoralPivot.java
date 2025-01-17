// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivotforcoralorsomeshit.Cpivot;

public class CoralPivot extends Command {
  /** Creates a new CoralPivot. */
  private final Cpivot cpivot;
  private final double targetPosition = 0;

  public CoralPivot(CPivot cpivot, double targetPosition) {
    this.cpivot = cpivot;
    this.targetPosition = targetPosition;
    addRequirements(cpivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     pivotSubsystem.setPivotPosition(targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      pivotSubsystem.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error = Math.abs(pivotSubsystem.getCurrentPosition() - targetPosition);
        return error < 1.0;
  }
}
