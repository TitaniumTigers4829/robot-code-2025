package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeInterface {

  @AutoLog
  public static class CoralIntakeInputs {
    public boolean isConnected = true;
    public double intakeVelocity = 0.0;
    public double intakeTemp = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeStatorCurrentAmps = 0.0;
    public double intakePosition = 0.0;
    public double intakeSupplyCurrentAmps = 0.0;
    public boolean hasCoral = false;
  }

  default void updateInputs(CoralIntakeInputs inputs) {}

  default void setIntakeSpeed(double speed) {}

  default double getIntakeSpeed() {
    return 0.0;
  }
}
