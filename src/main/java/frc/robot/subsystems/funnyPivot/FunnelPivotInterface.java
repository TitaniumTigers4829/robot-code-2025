package frc.robot.subsystems.funnyPivot;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelPivotInterface {
  @AutoLog
  public static class AlgaePivotInputs {
    public boolean isConnected = true;
    public double funnelAngle = 0.0;
    public double funnelVoltage = 0.0;
    public double funnelVelocity = 0.0;
    public double funnelTemp = 0.0;
    public double funnelSupplyCurrentAmps = 0.0;
    public double funnelTorqueCurrentAmps = 0.0;
  }

  default void updateInputs(AlgaePivotInputs inputs) {}

  default void setAlgaeSpeed(double speed) {}

  default void setAlgaeAngle(double angle) {}

  default void setAlgaeVoltage(double voltage) {}

  default boolean isPivotWithinAcceptapleError() {
    return true;
  }

  default double getAlgaeAngle() {
    return 0.0;
  }

  default double getAlgaePivotTarget() {
    return 0.0;
  }
}
