// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.dealgaefierPivot;

import edu.wpi.first.wpilibj2.command.Command;

public class DealgaefierPivot extends Command {
  /** Creates a new CoralPivot. */
  private final frc.robot.subsystems.pivot.DealgaefierPivot dealgaefierPivot;

  private double targetPosition = 0;

  public DealgaefierPivot(
      frc.robot.subsystems.pivot.DealgaefierPivot dealgaefierPivot, double targetPosition) {
    this.dealgaefierPivot = dealgaefierPivot;
    this.targetPosition = targetPosition;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dealgaefierPivot.setPivotPosition(targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error = Math.abs(dealgaefierPivot.getCurrentPosition() - targetPosition);
    return error < 1.0;
  }
}
