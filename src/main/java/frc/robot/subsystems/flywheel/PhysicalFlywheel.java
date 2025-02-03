// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.flywheel.FlywheelInterface.FlywheelInputs;

/** Add your docs here. */
public class PhysicalFlywheel {
  private PhysicalFlywheel physicalFlywheel =
      new PhysicalFlywheel();
  private PIDController flywheelPID;
  private double currentVolts;

  public PhysicalFlywheel() {
    flywheelPID =
        new PIDController(
            FlywheelConstants.FLYWHEEL_P,
            FlywheelConstants.FLYWHEEL_I,
            FlywheelConstants.FLYWHEEL_D);
  }

  public void updateInputs(FlywheelInputs inputs) {
    inputs.flywheelMotorPosition = getFlywheelSpeed();
    inputs.flywheelVelocity = getFlywheelVelocity();
  }

  public void setFlywheelSpeed(double speed) {
    setVolts(flywheelPID.calculate(getFlywheelSpeed(), speed));
  }

  public double getFlywheelSpeed() {
    return physicalFlywheel.getFlywheelSpeed();
  }

  public void setVolts(double volts) {
    currentVolts = flywheelPID.calculate(volts);
    physicalFlywheel.setVolts(currentVolts);
  }

  public double getVolts() {
    return currentVolts;
  }
}
