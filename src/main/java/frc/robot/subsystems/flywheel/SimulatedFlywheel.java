// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import frc.robot.subsystems.flywheel.FlywheelInterface.FlywheelInputs;

/** Add your docs here. */
public class SimulatedFlywheel {
    private FlywheelSim flywheelSim =
      new FlywheelSim(
          DCMotor.getFalcon500(2),
          FlywheelConstants.FLYWHEEL_GEAR_RATIO,
          true,
          0.0);
  private PIDController simPID;
  private double currentVolts;

  public SimulatedFlywheel() {
    simPID =
        new PIDController(
            FlywheelConstants.FLYWHEEL_P,
            FlywheelConstants.FLYWHEEL_I,
            FlywheelConstants.FLYWHEEL_D);
  }

  public void updateInputs(FlywheelInputs inputs) {
    inputs.leaderMotorPosition = getFlywheelPosition();
    inputs.followerMotorPosition = getFlywheelPosition();
  }

  public void setFlywheelspeed(double speed) {
    setVolts(simPID.calculate(getFlywheelSpeed(), speed));
  }

  public double getFlywheelSpeed() {
    // return 
  }

  public void setVolts(double volts) {
    currentVolts = simPID.calculate(volts);
    elevatorSim.setInputVoltage(currentVolts);
  }

  public double getVolts() {
    return currentVolts;
  }
}
