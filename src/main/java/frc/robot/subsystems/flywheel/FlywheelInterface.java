// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;
import org.littletonrobotics.junction.AutoLog;
/** Add your docs here. */
public class FlywheelInterface {
    @AutoLog
  public static class ElevatorInputs {
    public double leaderMotorPosition = 0.0;

    public double followerMotorPosition = 0.0;
  }

  /**
   * Updates inputs for elevator for AdvantageKit to log
   *
   * @param inputs values related to the elevator
   */
  public void updateInputs(FlywheelInputs inputs) {}

  /**
   * Gets the current position of the elevator
   *
   * @return
   */
  public double getFlywheelPosition() {
    return 0.0;
  }

  public void setFlywheelPosition(double position) {}

  public void setVolts(double volts) {}

  public double getVolts() {
    return 0.0;
  }

}
