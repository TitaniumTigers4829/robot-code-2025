package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorInterface {
  /** Creates a new ElevatorInterface. */
  @AutoLog
  public static class ElevatorInputs { // For values
    public double leaderMotorPosition = 0.0;
    public double followerMotorPosition = 0.0;
    public double leaderMotorVoltage = 0.0;
    public double followerMotorVoltage = 0.0;
    public double leaderDutyCycle = 0.0;
    public double followerDutyCycle = 0.0;
    public double desiredPosition = 0.0;
    public double leaderStatorCurrent = 0.0;
    public double followerStatorCurrent = 0.0;
    public double leaderVelocity = 0.0;
    public double elevatorError = 0.0;

    public double leaderTemp = 0.0;
    public double followerTemp = 0.0;
  }

  /**
   * Updates the input values for the Elevator subsystem.
   * 
   * @param inputs The input values to update.
   */
  public default void updateInputs(ElevatorInputs inputs) {}

  public default double getElevatorPosition() {
    return 0.0;
  }

  /**
   * Sets the desired position for the elevator.
   * 
   * @param position The target position to set for the elevator.
   */
  public default void setElevatorPosition(double position) {}

  /**
   * Sets the voltage to be applied to the elevator motors.
   * 
   * @param volts The voltage to apply to the elevator motors.
   */
  public default void setVolts(double volts) {}

  /**
   * Sets the percent output for the elevator motors.
   * 
   * @param output The percent output to set, typically in the range [-1.0, 1.0].
   */
  public default void setPercentOutput(double output) {}

  public default double getVolts() {
    return 0.0;
  }

  /**
   * Enables or disables the forward and reverse limit switches.
   * 
   * @param forward True to enable the forward limit switch, false to disable it.
   * @param reverse True to enable the reverse limit switch, false to disable it.
   */
  public default void enableLimits(boolean forward, boolean reverse) {}

  /**
   * Sets the elevator motors to open-loop control with the specified output.
   * 
   * @param output The output to apply in open-loop control, typically in the range [-1.0, 1.0].
   */
  public default void openLoop(double output) {}

  /**
   * Sets the PID controller gains for the elevator.
   * 
   * @param kP The proportional gain.
   * @param kI The integral gain.
   * @param kD The derivative gain.
   */
  public default void setPID(double kP, double kI, double kD) {}

  /**
   * Sets the feedforward gains for the elevator.
   * 
   * @param kS The static gain.
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   * @param kG The gravity gain.
   */
  public default void setFF(double kS, double kV, double kA, double kG) {}

  /**
   * Resets the elevator position to the specified value.
   * 
   * @param position The position to reset the elevator to.
   */
  public default void resetElevatorPosition(double position) {}

  public default boolean getReverseLimit() {
    return false;
  }

  public default boolean getForwardLimit() {
    return false;
  }
}
