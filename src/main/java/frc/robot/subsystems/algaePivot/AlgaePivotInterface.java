package frc.robot.subsystems.algaePivot;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaePivotInterface {
  @AutoLog
  public static class AlgaePivotInputs {
    public boolean isConnected = true;
    public double algaeAngle = 0.0;
    public double algaeVoltage = 0.0;
    public double algaeVelocity = 0.0;
    public double algaeTemp = 0.0;
    public double algaeSupplyCurrentAmps = 0.0;
    public double algaeTorqueCurrentAmps = 0.0;
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
