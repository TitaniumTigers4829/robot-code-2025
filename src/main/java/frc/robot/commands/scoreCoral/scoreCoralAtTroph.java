package frc.robot.commands.scoreCoral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaePivot.AlgaePivotConstants;
import frc.robot.subsystems.algaePivot.AlgaePivotSubsystem;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ScoreCoralAtTroph extends Command{
  private final AlgaePivotSubsystem algaePivotSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final CoralIntakeSubsystem coralIntakeSubsystem;;

  public ScoreCoralAtTroph(
      AlgaePivotSubsystem algaePivotSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      CoralIntakeSubsystem coralIntakeSubsystem) {
    this.algaePivotSubsystem = algaePivotSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.coralIntakeSubsystem = coralIntakeSubsystem;
    addRequirements(algaePivotSubsystem, elevatorSubsystem, coralIntakeSubsystem);
  }

  @Override
  // Called when command is initially scheduled
  public void initialize() {}

  @Override
  // Called every time the scheduler runs while the command is scheduled
  public void execute(){
    if(coralIntakeSubsystem.hasCoral()){
      elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_TROPH_LEVEL_HEIGHT);
      algaePivotSubsystem.setAlgaeAngle(AlgaePivotConstants.ALGAE_TROPH_LEVEL_ANGLE);
      if(elevatorSubsystem.getElevatorPosition() == ElevatorConstants.ELEVATOR_TROPH_LEVEL_HEIGHT){
        coralIntakeSubsystem.setIntakeSpeed(CoralIntakeConstants.EJECT_SPEED);
      }
    }
  }

  public void end(boolean interrupted){
    elevatorSubsystem.setElevatorPosition(0);
    coralIntakeSubsystem.setIntakeSpeed(0);
    algaePivotSubsystem.setAlgaeAngle(AlgaePivotConstants.ANGLE_ZERO);
  }

  public boolean isFinished(){
    return false;
  }
}
