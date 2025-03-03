package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ScoreL3 extends SequentialCommandGroup {
  /** Creates a new ScoreL2. */
  public ScoreL3(ElevatorSubsystem elevatorSubsystem, CoralIntakeSubsystem coralIntakeSubsystem) {
    addCommands(
        elevatorSubsystem.setElevationPosition(ElevatorSetpoints.L3.getPosition()),
        coralIntakeSubsystem.ejectCoral());
  }
}
