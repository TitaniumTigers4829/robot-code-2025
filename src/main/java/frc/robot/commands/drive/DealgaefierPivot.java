// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivotforcoralorsomeshit.CoralPivot;

public class CoralPivot extends Command {
  /** Creates a new CoralPivot. */
  private final CoralPivot coralPivot;
  private final double targetPosition = 0;

  public CoralPivot(CoralPivot coralPivot, double targetPosition) {
    this.coralPivot = coralPivot;
    this.targetPosition = targetPosition;
    addRequirements(coralPivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     coralPivot.setPivotPosition(targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      coralPivot.stop();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error = Math.abs(coralPivot.getCurrentPosition() - targetPosition);
        return error < 1.0;
  }
}
