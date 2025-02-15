package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeInterface {

  @AutoLog
  public static class IntakeInputs {
    public boolean isConnected = true;
    public double intakeVelocity = 0.0;
    public double intakeTemp = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(IntakeInputs inputs) {}

  default void setIntakeSpeed(double speed) {}

  default double getIntakeSpeed() {
    return 0.0;
  }
}
