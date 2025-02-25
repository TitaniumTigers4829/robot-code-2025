package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ScoreL4 extends SequentialCommandGroup {
  /** Creates a new ScoreL4. */
  public ScoreL4(ElevatorSubsystem elevatorSubsystem, CoralIntakeSubsystem coralIntakeSubsystem) {
    addCommands(
        elevatorSubsystem.setElevationPosition(ElevatorSetpoints.L4.getPosition()),
        coralIntakeSubsystem.ejectCoral());
  }
}
