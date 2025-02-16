package frc.robot.commands.coralIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;

public class EjectCoral extends Command {
  private final CoralIntakeSubsystem coralIntakeSubsystem;

  public EjectCoral(CoralIntakeSubsystem coralIntakeSubsystem) {
    this.coralIntakeSubsystem = coralIntakeSubsystem;
    addRequirements(this.coralIntakeSubsystem);
  }

  @Override
  public void execute() {
    coralIntakeSubsystem.setIntakeSpeed(CoralIntakeConstants.EJECT_SPEED);
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
