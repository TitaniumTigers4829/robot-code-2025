package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeInterface {
  @AutoLog
  public static class IntakeInputs {
    public boolean isConnected = true; //

    // intake motor
    public double intakeVelocity = 0.0;
    public double intakeTemp = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;
  }

  /**
   * Updates intake values
   *
   * @param inputs
   */
  default void updateInputs(IntakeInputs inputs) {}

  /**
   * Sets the intake speed
   *
   * @param speed The intake speed value from -1 to 1 (negative being reversed)
   */
  default void setIntakeSpeed(double speed) {}

  /**
   * Gets the current speed of the intake
   *
   * @return Current velocity of the intake
   */
  default double getIntakeSpeed() {
    return 0.0;
  }
}
