package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;

public class Intake extends Command {
  private final CoralIntakeSubsystem intakeSubsystem;

  public Intake(CoralIntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(this.intakeSubsystem);
  }

  @Override
  public void execute() {
    intakeSubsystem.setIntakeSpeed(CoralIntakeConstants.INTAKE_SPEED);
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
