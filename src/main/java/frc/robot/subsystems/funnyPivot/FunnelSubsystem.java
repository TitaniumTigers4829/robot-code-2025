package frc.robot.subsystems.funnyPivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class FunnelSubsystem extends SubsystemBase {
  private final FunnelPivotInterface funnelPivotInterface;
  private final AlgaePivotInputsAutoLogged inputs = new AlgaePivotInputsAutoLogged();

  public FunnelSubsystem(FunnelPivotInterface funnelPivotInterface) {
    this.funnelPivotInterface = funnelPivotInterface;
  }

  public void setAlgaeSpeed(double speed) {
    funnelPivotInterface.setAlgaeSpeed(speed);
  }

  public void setAlgaeAngle(double angle) {
    funnelPivotInterface.setAlgaeAngle(angle);
  }

  public void setAlgaeVoltage(double voltage) {
    funnelPivotInterface.setAlgaeVoltage(voltage);
  }

  public void periodic() {
    funnelPivotInterface.updateInputs(inputs);
    Logger.processInputs("AlgaePivotSubsystem/", inputs);
  }
}
