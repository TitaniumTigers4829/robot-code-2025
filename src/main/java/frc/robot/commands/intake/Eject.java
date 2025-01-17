package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;


public class Eject extends Command{
  private final IntakeSubsystem intakeSubsystem;

  public Eject(IntakeSubsystem intakeSubsystem){
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(this.intakeSubsystem);
  }

  public void execute(){
    intakeSubsystem.setIntakeSpeed(IntakeConstants.ejectSpeed);
  }

  public void end(){
    intakeSubsystem.setIntakeSpeed(0);
  }

  public boolean isFinished(){
    return false;
  }
}
