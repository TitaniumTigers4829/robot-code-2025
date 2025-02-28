package frc.robot.commands.scoreCoral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaePivot.AlgaePivotConstants;
import frc.robot.subsystems.algaePivot.AlgaePivotSubsystem;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class GetCoralFeedingStation extends Command {
  private final CoralIntakeSubsystem coralIntakeSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final AlgaePivotSubsystem algaePivotSubsystem;

  public GetCoralFeedingStation(
      CoralIntakeSubsystem coralIntakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      AlgaePivotSubsystem algaePivotSubsystem) {
    this.coralIntakeSubsystem = coralIntakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.algaePivotSubsystem = algaePivotSubsystem;
    addRequirements(coralIntakeSubsystem, elevatorSubsystem, algaePivotSubsystem);
  }

  @Override
  // Called when command is initially scheduled
  public void initialize() {}

  @Override
  // Called every time the scheduler runs while the command is scheduled
  public void execute() {
    elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_FEEDING_STATION_HEIGHT);
    coralIntakeSubsystem.setIntakeSpeed(CoralIntakeConstants.INTAKE_SPEED);
    algaePivotSubsystem.setAlgaeAngle(AlgaePivotConstants.ALGAE_FEEDING_STATION_ANGLE);
    if(coralIntakeSubsystem.hasCoral()){
      coralIntakeSubsystem.setIntakeSpeed(0);
    }
  }

  @Override
  // Called once the command ends or is interrupted
  public void end(boolean interrupted) {
    elevatorSubsystem.setElevatorPosition(0);
    coralIntakeSubsystem.setIntakeSpeed(0);
    algaePivotSubsystem.setAlgaeAngle(AlgaePivotConstants.ANGLE_ZERO);
  }

  @Override
  // Returns true when the command should end
  public boolean isFinished() {
    return false;
  }
}
