package frc.robot.subsystems.funnelPivot;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelPivotInterface {
  @AutoLog
  public class FunnelPivotInputs {
    public boolean isConnected = true;
    public double funnelAngle = 0.0;
    public double funnelVoltage = 0.0;
    public double funnelVelocity = 0.0;
    public double funnelTemp = 0.0;
    public double funnelSupplyCurrentAmps = 0.0;
    public double funnelTorqueCurrentAmps = 0.0;
  }

  default void updateInputs(FunnelPivotInputs inputs) {}

  default void setFunnelSpeed(double speed) {}

  default void setFunnelAngle(double angle) {}

  default void setFunnelVoltage(double voltage) {}

  default boolean isPivotWithinAcceptapleError() {
    return true;
  }

  default double getFunnelAngle() {
    return 0.0;
  }

  default double getFunnelPivotTarget() {
    return 0.0;
  }

  default void setPID(double kP, double kI, double kD) {}
}
