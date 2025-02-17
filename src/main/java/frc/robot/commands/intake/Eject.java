package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;

public class Eject extends Command {
  private final CoralIntakeSubsystem coralIntakeSubsystem;

  public Eject(CoralIntakeSubsystem coralIntakeSubsystem) {
    this.coralIntakeSubsystem = coralIntakeSubsystem;
    addRequirements(this.coralIntakeSubsystem);
  }

  @Override
  public void execute() {
    coralIntakeSubsystem.setIntakeSpeed(coralIntakeConstants.EJECT_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
