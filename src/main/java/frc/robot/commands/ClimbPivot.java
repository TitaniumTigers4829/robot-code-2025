// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimbPivot extends Command {
  /** Creates a new ClimbPivot. */
  private final ClimbPivot climbPivot;
  private final double targetPosition = 0;

  public ClimbPivot(CoralPivot climbPivot, double targetPosition) {
    this.climbPivot = climbPivot;
    this.targetPosition = targetPosition;
    addRequirements(climbPivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbPivot.setPivotPosition(targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      climbPivot.stop();
  
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error = Math.abs(climbPivot.getCurrentPosition() - targetPosition);
        return error < 1.0;
  }
}
