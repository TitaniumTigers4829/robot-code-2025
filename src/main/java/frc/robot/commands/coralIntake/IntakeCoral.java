package frc.robot.commands.coralIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;

public class IntakeCoral extends Command {
  private final CoralIntakeSubsystem coralIntakeSubsystem;

  public IntakeCoral(CoralIntakeSubsystem intakeSubsystem) {
    this.coralIntakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void execute() {
    if (!coralIntakeSubsystem.hasCoral()) {
      coralIntakeSubsystem.setIntakeSpeed(CoralIntakeConstants.INTAKE_SPEED);
    } else {
      coralIntakeSubsystem.setIntakeSpeed(0.0);
    }
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
