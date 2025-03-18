package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeInterface {

  @AutoLog
  public static class CoralIntakeInputs {
    public boolean isMotorConnected = false;
    public boolean isInnerSensorConnected = false;
    public boolean isOuterSensorConnected = false;
    public double intakeVelocity = 0.0;
    public double intakeTemp = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeStatorCurrentAmps = 0.0;
    public double intakePosition = 0.0;
    public double intakeSupplyCurrentAmps = 0.0;
    public boolean hasCoral = false;
    public boolean hasControl = false;
    public double intakeDutyCycle = 0.0;
    public double intakeReference = 0.0;
  }

  /**
   * Sets the voltage to be applied to the intake motor.
   * 
   * @param volts The voltage to apply to the intake motor.
   */
  default void setIntakeVoltage(double volts) {}

  /**
   * Updates the input values for the Coral Intake subsystem.
   * 
   * @param inputs The input values to update.
   */
  default void updateInputs(CoralIntakeInputs inputs) {}

  /**
   * Sets the speed of the intake motor.
   * 
   * @param speed The desired speed, typically in the range [-1.0, 1.0].
   */
  default void setIntakeSpeed(double speed) {}

  /**
   * Retrieves the current speed of the intake motor.
   * 
   * @return The current speed of the intake motor.
   */
  default double getIntakeSpeed() {
    return 0.0;
  }

  /**
   * Sets the velocity of the intake motor.
   * 
   * @param velocity The desired velocity in appropriate units.
   */
  default void setIntakeVelocity(double velocity) {}

  /**
   * Sets the proportional gain (P) for the intake motor's PID controller.
   * 
   * @param P The proportional gain value to set.
   */
  default void setPID(double P) {}
}
