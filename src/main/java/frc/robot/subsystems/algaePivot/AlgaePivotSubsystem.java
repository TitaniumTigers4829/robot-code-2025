package frc.robot.subsystems.algaePivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.algaePivot.ManualAlgaePivot;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class AlgaePivotSubsystem extends SubsystemBase {
  private final AlgaePivotInterface algaePivotInterface;
  private final AlgaePivotInputsAutoLogged inputs = new AlgaePivotInputsAutoLogged();

  public AlgaePivotSubsystem(AlgaePivotInterface algaePivotInterface) {
    this.algaePivotInterface = algaePivotInterface;
  }

  public void setAlgaeSpeed(double speed) {
    algaePivotInterface.setAlgaeSpeed(speed);
  }

  public void setAlgaeAngle(double angle) {
    algaePivotInterface.setAlgaeAngle(angle);
  }

  public void setAlgaeVoltage(double voltage) {
    algaePivotInterface.setAlgaeVoltage(voltage);
  }

  public void periodic() {
    algaePivotInterface.updateInputs(inputs);
    Logger.processInputs("AlgaePivotSubsystem/", inputs);
  }

  private Command manualAlgaePivot(DoubleSupplier speed) {
    return new StartEndCommand(
      //does this on start
      () -> this.setAlgaeSpeed(speed.getAsDouble()),
      //does this when command ends
      () -> this.setAlgaeSpeed(AlgaeConstants.ALGAE_NEUTRAL_SPEED),
      //requirements for command
      this);
  }
}
