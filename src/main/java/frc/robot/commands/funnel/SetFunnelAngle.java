// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.funnelPivot.FunnelConstants;
import frc.robot.subsystems.funnelPivot.FunnelSubsystem;
import java.util.function.DoubleSupplier;

public class SetFunnelAngle extends Command {

  FunnelSubsystem funnelSubsystem;
  DoubleSupplier joystickY;

  public SetFunnelAngle(FunnelSubsystem funnelSubsystem, DoubleSupplier joystickY) {
    this.funnelSubsystem = funnelSubsystem;
    this.joystickY = joystickY;
    addRequirements(funnelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    funnelSubsystem.setFunnelAngle(joystickY.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnelSubsystem.setFunnelAngle(FunnelConstants.MAX_ANGLE); // whatever the default angle is
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
