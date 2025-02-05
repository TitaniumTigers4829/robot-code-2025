package frc.robot.commands.algaePivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePivot.AlgaeConstants;
import frc.robot.subsystems.AlgaePivot.AlgaePivotSubsystem;

public class ManualAlgaePivot extends Command{
  private final AlgaePivotSubsystem algaePivotSubsystem;
  private final DoubleSupplier speed;

  public ManualAlgaePivot(AlgaePivotSubsystem algaePivotSubsystem, DoubleSupplier speed){
    this.algaePivotSubsystem = algaePivotSubsystem;
    this.speed = speed;
    addRequirements(algaePivotSubsystem); 
  }

  @Override
  public void initialize(){

  }

  @Override
  public void execute(){
    algaePivotSubsystem.setAlgaeSpeed(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted){
   algaePivotSubsystem.setAlgaeSpeed(AlgaeConstants.ALGAE_NEUTRAL_SPEED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
