package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;

public class Intake extends Command {
  private final CoralIntakeSubsystem coralIntakeSubsystem;

  public Intake(CoralIntakeSubsystem coralIntakeSubsystem) {
    this.coralIntakeSubsystem = coralIntakeSubsystem;
    addRequirements(this.coralIntakeSubsystem);
  }

  @Override
  public void execute() {
    coralIntakeSubsystem.setIntakeSpeed(CoralIntakeConstants.INTAKE_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    coralIntakeSubsystem.setIntakeSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
