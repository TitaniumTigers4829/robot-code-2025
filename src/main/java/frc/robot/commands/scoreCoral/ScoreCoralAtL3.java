package frc.robot.commands.scoreCoral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaePivot.AlgaePivotConstants;
import frc.robot.subsystems.algaePivot.AlgaePivotSubsystem;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveModule;

public class ScoreCoralAtL3 extends Command {
  private final AlgaePivotSubsystem algaePivotSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final CoralIntakeSubsystem coralIntakeSubsystem;
  private final SwerveModule swerveModule;

  public ScoreCoralAtL3(
      AlgaePivotSubsystem algaePivotSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      CoralIntakeSubsystem coralIntakeSubsystem,
      SwerveModule swerveModule) {
    this.algaePivotSubsystem = algaePivotSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.coralIntakeSubsystem = coralIntakeSubsystem;
    this.swerveModule = swerveModule;
    addRequirements(algaePivotSubsystem, elevatorSubsystem, coralIntakeSubsystem);
  }

  @Override
  // Called when command is initially scheduled
  public void initialize() {}

  @Override
  // Called every time the scheduler runs while the command is scheduled
  public void execute() {
    if (coralIntakeSubsystem.hasCoral()
        && swerveModule.getDrivePositionMeters() == SwerveConstants.L3_POSITION) {
      elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_L3_HEIGHT);
      algaePivotSubsystem.setAlgaeAngle(AlgaePivotConstants.ALGAE_L3_ANGLE);
      if (Math.abs(elevatorSubsystem.getElevatorPosition() - ElevatorConstants.ELEVATOR_L3_HEIGHT)
          < ElevatorConstants.ELEVATOR_POSITION_THRESHOLD) {
        coralIntakeSubsystem.ejectCoral();
      }
    }
  }

  public void end(boolean interrupted) {
    elevatorSubsystem.setElevatorPosition(0);
    coralIntakeSubsystem.setIntakeSpeed(0);
    algaePivotSubsystem.setAlgaeAngle(AlgaePivotConstants.ANGLE_ZERO);
  }

  public boolean isFinished() {
    return false;
  }
}
