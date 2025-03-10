package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPosition extends Command {
  private SwerveDrive swerveDrive;
  private ElevatorSubsystem elevatorSubsystem;
  private double position;

  /** Creates a new SetElevatorPosition. */
  public SetElevatorPosition(
      SwerveDrive swerveDrive, ElevatorSubsystem elevatorSubsystem, double position) {
    this.swerveDrive = swerveDrive;
    this.elevatorSubsystem = elevatorSubsystem;
    this.position = position;

    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(swerveDrive.getGyroPitch()) < ModuleConstants.GYRO_MAX_PITCH
        || Math.abs(swerveDrive.getGyroRoll()) < ModuleConstants.GYRO_MAX_ROLL) {
      elevatorSubsystem.setElevatorPosition(position);
    } else {
      elevatorSubsystem.setElevatorPosition(ElevatorConstants.LIMIT);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isAtSetpoint(position);
  }
}
