package frc.robot.commands.scoreCoral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaePivot.AlgaePivotConstants;
import frc.robot.subsystems.algaePivot.AlgaePivotSubsystem;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class scoreCoralAtL2 extends Command{
  private final AlgaePivotSubsystem algaePivotSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final CoralIntakeSubsystem coralIntakeSubsystem;;

  public scoreCoralAtL2(
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
    elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_L2_HEIGHT);
    coralIntakeSubsystem.setIntakeSpeed(CoralIntakeConstants.EJECT_SPEED);
    algaePivotSubsystem.setAlgaeAngle(AlgaePivotConstants.ALGAE_L2_ANGLE);
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
