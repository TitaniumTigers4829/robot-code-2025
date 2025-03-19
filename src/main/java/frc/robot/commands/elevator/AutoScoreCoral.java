package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class AutoScoreCoral extends SequentialCommandGroup {
  /** Creates a new ScoreL4. */
  public AutoScoreCoral(ElevatorSubsystem elevatorSubsystem, CoralIntakeSubsystem coralIntakeSubsystem, double elevatorPosition) {
    addCommands(
        elevatorSubsystem.setElevationPosition(elevatorPosition).withTimeout(6.0),
        Commands.run(
                () -> coralIntakeSubsystem.setIntakeVelocity(CoralIntakeConstants.EJECT_SPEED),
                coralIntakeSubsystem)
            .withTimeout(10.0));
  }
}
