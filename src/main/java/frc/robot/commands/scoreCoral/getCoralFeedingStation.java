package frc.robot.commands.scoreCoral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaePivot.AlgaePivotConstants;
import frc.robot.subsystems.algaePivot.AlgaePivotSubsystem;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class getCoralFeedingStation extends Command {
  private final CoralIntakeSubsystem coralIntakeSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final AlgaePivotSubsystem algaePivotSubsystem;

  public getCoralFeedingStation(
      CoralIntakeSubsystem coralIntakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      AlgaePivotSubsystem algaePivotSubsystem) {
    this.coralIntakeSubsystem = coralIntakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.algaePivotSubsystem = algaePivotSubsystem;
    addRequirements(coralIntakeSubsystem, elevatorSubsystem);
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
  }
}
