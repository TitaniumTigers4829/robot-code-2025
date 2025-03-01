package frc.robot.subsystems.funnelPivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class FunnelSubsystem extends SubsystemBase {
  private final FunnelPivotInterface funnelPivotInterface;
  private final FunnelPivotInputsAutoLogged inputs = new FunnelPivotInputsAutoLogged();

  public FunnelSubsystem(FunnelPivotInterface funnelPivotInterface) {
    this.funnelPivotInterface = funnelPivotInterface;
  }

  public void setFunnelSpeed(double speed) {
    funnelPivotInterface.setFunnelSpeed(speed);
  }

  public void setFunnelAngle(double angle) {
    funnelPivotInterface.setFunnelAngle(angle);
  }

  public void setFunnelVoltage(double voltage) {
    funnelPivotInterface.setFunnelVoltage(voltage);
  }

  public void periodic() {
    funnelPivotInterface.updateInputs(inputs);
    Logger.processInputs("FunnelPivotSubsystem/", inputs);
  }

  public Command manualFunnel(DoubleSupplier position) {
    return new RunCommand(() -> funnelPivotInterface.setFunnelSpeed(position.getAsDouble()), this);
  }
}
