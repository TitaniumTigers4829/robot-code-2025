// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import frc.robot.subsystems.flywheel.FlywheelInterface.FlywheelInputs;

/** Add your docs here. */
public class PhysicalFlywheel {
  private PhysicalFlywheel physicalFlywheel =
      new PhysicalFlywheel(null, DCMotor.getFalcon500(2), 0);
  private PIDController PID;
  private double currentVolts;

  public PhysicalFlywheel() {
    PID =
        new PIDController(
            FlywheelConstants.FLYWHEEL_P,
            FlywheelConstants.FLYWHEEL_I,
            FlywheelConstants.FLYWHEEL_D);
  }

  public void updateInputs(FlywheelInputs inputs) {
    inputs.leaderMotorPosition = getFlywheelSpeed();
    inputs.followerMotorPosition = getFlywheelSpeed();
  }

  public void setFlywheelSpeed(double speed) {
    setVolts(PID.calculate(getFlywheelSpeed(), speed));
  }

  public double getFlywheelSpeed() {
    return physicalFlywheel.getAngularVelocityRPM();
  }

  public void setVolts(double volts) {
    currentVolts = PID.calculate(volts);
    physicalFlywheel.setInputVoltage(currentVolts);
  }

  public double getVolts() {
    return currentVolts;
  }
}
