package frc.robot.commands.algaePivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaePivot.AlgaePivotConstants;
import frc.robot.subsystems.algaePivot.AlgaePivotSubsystem;

public class SetAlgaePivotWithPID extends Command {
  private final AlgaePivotSubsystem algaePivotSubsystem;

  public SetAlgaePivotWithPID(AlgaePivotSubsystem algaePivotSubsystem) {
    this.algaePivotSubsystem = algaePivotSubsystem;
    addRequirements(algaePivotSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    algaePivotSubsystem.setAlgaeAngle(AlgaePivotConstants.ALGAE_PIVOT_ANGLE);
  }

  @Override
  public void end(boolean interrupted) {
    algaePivotSubsystem.setAlgaeAngle(AlgaePivotConstants.ANGLE_ZERO);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
