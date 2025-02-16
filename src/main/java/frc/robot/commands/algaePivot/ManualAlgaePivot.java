package frc.robot.commands.algaePivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaePivot.AlgaePivotConstants;
import frc.robot.subsystems.algaePivot.AlgaePivotSubsystem;
import java.util.function.DoubleSupplier;

public class ManualAlgaePivot extends Command {
  private final AlgaePivotSubsystem algaePivotSubsystem;
  private final DoubleSupplier speed;

  public ManualAlgaePivot(AlgaePivotSubsystem algaePivotSubsystem, DoubleSupplier speed) {
    this.algaePivotSubsystem = algaePivotSubsystem;
    this.speed = speed;
    addRequirements(algaePivotSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    algaePivotSubsystem.setAlgaeSpeed(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    algaePivotSubsystem.setAlgaeSpeed(AlgaePivotConstants.ALGAE_NEUTRAL_SPEED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
